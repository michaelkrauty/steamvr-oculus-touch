// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Raw-universe capture tracks, confirmed by the next inventory observation of each index.
use std::collections::BTreeMap;

use cv1touch::{eval, shm, vr};
use nalgebra::Isometry3;

const INVENTORY_STEP_NS: u64 = 500_000_000 / vr::MAX_DEVICES as u64;

#[derive(Default)]
struct Slot {
    device: Option<vr::Device>,
    pending: Vec<[f64; 8]>,
    valid: Option<bool>,
    suspended: bool,
}

pub(super) struct Frame {
    pub t_ns: u64,
    pub poses: Vec<Option<Isometry3<f64>>>,
}

pub(super) struct Sampler {
    pub tracks: BTreeMap<String, Vec<[f64; 8]>>,
    pub kinds: BTreeMap<String, String>,
    pub hmd_index: Option<usize>,
    slots: [Slot; vr::MAX_DEVICES],
    next_index: usize,
    next_probe: u64,
}

impl Default for Sampler {
    fn default() -> Self {
        Self {
            tracks: BTreeMap::new(),
            kinds: BTreeMap::new(),
            hmd_index: None,
            slots: std::array::from_fn(|_| Slot::default()),
            next_index: 0,
            next_probe: 0,
        }
    }
}

impl Sampler {
    /// The initial inventory is read before recording starts, outside the sampling loop.
    pub fn new(devices: Vec<vr::Device>) -> Self {
        let mut sampled = Self::default();
        for device in devices {
            let index = device.index as usize;
            if index < vr::MAX_DEVICES {
                sampled.confirm(index, Some(device));
            }
        }
        sampled
    }

    fn confirm(&mut self, index: usize, device: Option<vr::Device>) {
        let device = device.filter(|d| {
            d.index as usize == index
                && !d.serial.is_empty()
                && (d.tracking_system == "lighthouse"
                    || d.class == vr::CLASS_HMD
                    || d.serial.starts_with("CV1T-"))
        });
        let slot = &mut self.slots[index];
        let same = slot.device.as_ref().map(|d| &d.serial) == device.as_ref().map(|d| &d.serial);
        if same && !slot.suspended && !slot.pending.is_empty() {
            if let Some(d) = &device {
                self.tracks
                    .entry(d.serial.clone())
                    .or_default()
                    .append(&mut slot.pending);
            }
        }
        // A changed identity invalidates the whole interval, including poses from either device.
        slot.pending.clear();
        slot.device = device;
        slot.suspended = false;
        if let Some(d) = &slot.device {
            let kind = match d.class {
                _ if d.serial.starts_with("CV1T-") => "touch",
                vr::CLASS_CONTROLLER => "controller",
                vr::CLASS_HMD => "hmd",
                vr::CLASS_TRACKING_REFERENCE => "base",
                _ => "other",
            };
            self.kinds.insert(d.serial.clone(), kind.into());
        }
        self.hmd_index = self.slots.iter().position(|slot| {
            slot.device
                .as_ref()
                .is_some_and(|d| d.class == vr::CLASS_HMD)
        });
    }

    pub fn poll(&mut self, client: &vr::Client) -> Frame {
        self.sample(shm::now_ns, || client.raw_poses(), |i| client.device(i))
    }

    pub fn sample(
        &mut self,
        timestamp: impl FnOnce() -> u64,
        poses: impl FnOnce() -> Vec<Option<Isometry3<f64>>>,
        mut device: impl FnMut(u32) -> Option<vr::Device>,
    ) -> Frame {
        let raw = poses();
        let now = timestamp();
        for (index, slot) in self.slots.iter_mut().enumerate() {
            let pose = raw.get(index).copied().flatten();
            let valid = pose.is_some();
            if slot.valid.is_some_and(|was| was != valid) {
                slot.pending.clear();
                // Tracking restarting may be the only sign that the index changed hands.
                // Wait for a fresh identity observation before accepting more poses.
                slot.suspended = true;
            }
            slot.valid = Some(valid);
            if slot.device.is_some() && !slot.suspended {
                if let Some(pose) = pose {
                    slot.pending.push(eval::truth_row(now, &pose));
                }
            }
        }
        // One index per step, after the timestamped pose read: never a full scan in the hot loop.
        if now >= self.next_probe {
            self.confirm(self.next_index, device(self.next_index as u32));
            self.next_index = (self.next_index + 1) % vr::MAX_DEVICES;
            self.next_probe = now.saturating_add(INVENTORY_STEP_NS);
        }
        Frame {
            t_ns: now,
            poses: raw,
        }
    }

    /// Confirm the tail after sampling stops; this scan cannot open a gap in the pose stream.
    pub fn finish(mut self, mut device: impl FnMut(u32) -> Option<vr::Device>) -> Self {
        for index in 0..vr::MAX_DEVICES {
            self.confirm(index, device(index as u32));
        }
        self
    }

    pub fn into_paced_tracks(mut self, reference: &str) -> (Vec<[f64; 8]>, Vec<eval::RefTrack>) {
        let samples = self.tracks.remove(reference).unwrap_or_default();
        let others = self
            .tracks
            .into_iter()
            .map(|(serial, samples)| eval::RefTrack {
                kind: self.kinds.get(&serial).cloned().unwrap_or_default(),
                serial,
                samples,
            })
            .collect();
        (samples, others)
    }
}
