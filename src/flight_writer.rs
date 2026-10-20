// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! One dropout writer for the daemon's lifetime. Storage never holds the queue lock: if it
//! stalls, at most two recordings wait, with gaps that began in use kept ahead of still gaps.

use std::collections::VecDeque;
use std::io;
use std::path::PathBuf;
use std::sync::{Arc, Condvar, Mutex};
use std::thread::{self, JoinHandle};

use crate::flight::{self, Dropout};

const PENDING: usize = 2;
type Job = (Dropout, u64);

#[derive(Default)]
struct Queue {
    jobs: VecDeque<Job>,
    closed: bool,
}

#[derive(Default)]
struct Shared {
    queue: Mutex<Queue>,
    ready: Condvar,
}

pub struct Writer {
    shared: Arc<Shared>,
    worker: JoinHandle<()>,
}

impl Writer {
    pub fn start(dir: PathBuf, report: impl Fn(String) + Send + 'static) -> io::Result<Self> {
        Self::with_writer(move |dropout, unix_s| {
            let name = ["left", "right"][dropout.hand];
            match flight::save(&dir, &dropout, unix_s) {
                Ok(path) => report(format!(
                    "{name}: recorded the radio from {} s before its dropout to {} s after: {}",
                    flight::BEFORE_NS / 1_000_000_000,
                    flight::AFTER_NS / 1_000_000_000,
                    path.display()
                )),
                Err(e) => report(format!("{name}: could not record its dropout: {e}")),
            }
        })
    }

    fn with_writer(mut write: impl FnMut(Dropout, u64) + Send + 'static) -> io::Result<Self> {
        let shared = Arc::new(Shared::default());
        let pending = Arc::clone(&shared);
        let worker = thread::Builder::new()
            .name("cv1t-dropout".into())
            .spawn(move || loop {
                let job = {
                    let mut queue = pending.queue.lock().unwrap();
                    while queue.jobs.is_empty() && !queue.closed {
                        queue = pending.ready.wait(queue).unwrap();
                    }
                    let Some(job) = queue.jobs.pop_front() else {
                        return;
                    };
                    job
                };
                write(job.0, job.1);
            })?;
        Ok(Self { shared, worker })
    }

    /// Queues without waiting for storage. Returns the recording lost when the queue is full:
    /// the oldest still gap first, then the oldest in-use gap, or the new still gap when only
    /// in-use gaps are pending. The recording already being written is never replaced.
    pub fn submit(&self, dropout: Dropout, unix_s: u64) -> io::Result<Option<Dropout>> {
        if self.worker.is_finished() {
            return Err(io::Error::other("the dropout recording thread stopped"));
        }
        let mut queue = self.shared.queue.lock().unwrap();
        let dropped = if queue.jobs.len() == PENDING {
            let lower = queue.jobs.iter().position(|job| !job.0.began_in_use);
            if !dropout.began_in_use && lower.is_none() {
                return Ok(Some(dropout));
            }
            queue.jobs.remove(lower.unwrap_or(0)).map(|job| job.0)
        } else {
            None
        };
        queue.jobs.push_back((dropout, unix_s));
        drop(queue);
        self.shared.ready.notify_one();
        Ok(dropped)
    }
}

impl Drop for Writer {
    fn drop(&mut self) {
        self.shared.queue.lock().unwrap().closed = true;
        self.shared.ready.notify_one();
        // A filesystem call can remain blocked indefinitely. This single worker ends with
        // the process; shutdown never waits for it or creates a replacement writer.
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;
    use std::time::Duration;

    fn dropout(n: u64) -> Dropout {
        Dropout {
            hand: 0,
            last_report_ns: n,
            began_in_use: true,
            records: Vec::new(),
        }
    }

    #[test]
    fn in_use_recordings_take_priority_when_storage_is_stalled() {
        let (seen, written) = mpsc::channel();
        let (release, stalled) = mpsc::channel();
        let writer = Writer::with_writer(move |d, _| {
            seen.send(d.last_report_ns).unwrap();
            if d.last_report_ns == 1 {
                let _ = stalled.recv();
            }
        })
        .unwrap();
        let still = |n| Dropout {
            began_in_use: false,
            ..dropout(n)
        };
        assert!(writer.submit(dropout(1), 0).unwrap().is_none());
        assert_eq!(written.recv_timeout(Duration::from_secs(5)).unwrap(), 1);
        assert!(writer.submit(dropout(2), 0).unwrap().is_none());
        assert!(writer.submit(still(3), 0).unwrap().is_none());
        // A later still gap replaces the older still gap, never the in-use gap ahead of it.
        assert_eq!(writer.submit(still(4), 0).unwrap(), Some(still(3)));
        assert_eq!(writer.submit(dropout(5), 0).unwrap(), Some(still(4)));
        // Both pending recordings began in use: reject a still gap, then keep the newer in-use.
        assert_eq!(writer.submit(still(6), 0).unwrap(), Some(still(6)));
        assert_eq!(writer.submit(dropout(7), 0).unwrap(), Some(dropout(2)));
        drop(writer);
        release.send(()).unwrap();
        assert_eq!(written.recv_timeout(Duration::from_secs(5)).unwrap(), 5);
        assert_eq!(written.recv_timeout(Duration::from_secs(5)).unwrap(), 7);
        assert!(written.recv_timeout(Duration::from_secs(5)).is_err());
    }

    #[test]
    fn stalled_storage_keeps_one_writer_and_the_newest_pending_recordings() {
        let (seen, written) = mpsc::channel();
        let (release, stalled) = mpsc::channel();
        let writer = Writer::with_writer(move |d, unix| {
            seen.send((d.last_report_ns, unix)).unwrap();
            if d.last_report_ns == 1 {
                stalled.recv().unwrap();
            }
        })
        .unwrap();
        assert!(writer.submit(dropout(1), 101).unwrap().is_none());
        assert_eq!(
            written.recv_timeout(Duration::from_secs(5)).unwrap(),
            (1, 101)
        );
        assert!(writer.submit(dropout(2), 102).unwrap().is_none());
        assert!(writer.submit(dropout(3), 103).unwrap().is_none());
        assert_eq!(writer.submit(dropout(4), 104).unwrap(), Some(dropout(2)));
        assert_eq!(writer.shared.queue.lock().unwrap().jobs.len(), 2);
        assert!(
            written.try_recv().is_err(),
            "another writer ran while storage was stalled"
        );
        let (closed, stopped) = mpsc::channel();
        let closing = thread::spawn(move || {
            drop(writer);
            closed.send(()).unwrap();
        });
        let did_stop = stopped.recv_timeout(Duration::from_secs(5)).is_ok();
        release.send(()).unwrap();
        closing.join().unwrap();
        assert!(did_stop, "shutdown waited for stalled storage");
        assert_eq!(
            written.recv_timeout(Duration::from_secs(5)).unwrap(),
            (3, 103)
        );
        assert_eq!(
            written.recv_timeout(Duration::from_secs(5)).unwrap(),
            (4, 104)
        );
        assert!(written.recv_timeout(Duration::from_secs(5)).is_err());
    }
}
