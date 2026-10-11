// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Pure setup dependency planning and a private completion journal.
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Stage {
    Camera,
    Gravity,
    Render,
    Registration,
    Inputs,
}
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Artifact {
    Missing,
    Invalid(String),
    Valid(String),
}
#[derive(Debug, Clone)]
pub struct Snapshot {
    pub camera: Artifact,
    pub registration: Artifact,
    pub render: Artifact,
    pub render_sources: Artifact,
    pub inputs: Artifact,
    pub recordings: BTreeSet<PathBuf>,
    pub blocked: BTreeMap<Stage, String>,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum State {
    Missing,
    Invalid,
    Unknown,
    Stale,
    Blocked,
    Ready,
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct StageReport {
    pub stage: Stage,
    pub state: State,
    pub reason: String,
}
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Binding {
    pub artifact_sha256: String,
    pub camera_sha256: Option<String>,
    pub sources_sha256: Option<String>,
    pub recordings: Vec<PathBuf>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Checkpoint {
    pub version: u32,
    pub completed: BTreeMap<Stage, Binding>,
    pub pending: BTreeSet<Stage>,
    pub recordings: BTreeMap<Stage, Vec<PathBuf>>,
}
impl Default for Checkpoint {
    fn default() -> Self {
        Self {
            version: 1,
            completed: BTreeMap::new(),
            pending: BTreeSet::new(),
            recordings: BTreeMap::new(),
        }
    }
}
fn artifact(s: &Snapshot, stage: Stage) -> &Artifact {
    match stage {
        Stage::Camera | Stage::Gravity => &s.camera,
        Stage::Registration => &s.registration,
        Stage::Render => &s.render,
        Stage::Inputs => &s.inputs,
    }
}
fn valid(a: &Artifact) -> Option<&str> {
    if let Artifact::Valid(hash) = a {
        Some(hash)
    } else {
        None
    }
}
fn binding(s: &Snapshot, stage: Stage, recordings: Vec<PathBuf>) -> Result<Binding, String> {
    let own = valid(artifact(s, stage)).ok_or("stage artifact is missing or invalid")?;
    if !is_hash(own) {
        return Err("invalid SHA-256 fingerprint".into());
    }
    let camera = if matches!(stage, Stage::Gravity | Stage::Registration) {
        Some(
            valid(&s.camera)
                .ok_or("camera calibration is missing or invalid")?
                .to_owned(),
        )
    } else {
        None
    };
    let sources = if stage == Stage::Render {
        Some(
            valid(&s.render_sources)
                .ok_or("render source geometry is missing or invalid")?
                .to_owned(),
        )
    } else {
        None
    };
    if camera.iter().chain(sources.iter()).any(|h| !is_hash(h)) {
        return Err("invalid dependency SHA-256 fingerprint".into());
    }
    if recordings
        .iter()
        .any(|p| !p.is_absolute() || !s.recordings.contains(p))
    {
        return Err("exact stage recording is unavailable".into());
    }
    Ok(Binding {
        artifact_sha256: own.to_owned(),
        camera_sha256: camera,
        sources_sha256: sources,
        recordings,
    })
}
fn assessment(s: &Snapshot, c: &Checkpoint, stage: Stage) -> (State, String) {
    use State::*;
    if c.version != 1 {
        return (Invalid, "unsupported checkpoint version".into());
    }
    if matches!(stage, Stage::Gravity | Stage::Registration) && valid(&s.camera).is_none() {
        return (Blocked, "requires valid camera calibration".into());
    }
    if stage == Stage::Render && valid(&s.render_sources).is_none() {
        return (
            Blocked,
            "requires valid controller geometry and source meshes".into(),
        );
    }
    if c.pending.contains(&stage) {
        return (Stale, "previous stage attempt did not complete".into());
    }
    match artifact(s, stage) {
        Artifact::Missing => return (Missing, "artifact is missing".into()),
        Artifact::Invalid(reason) => return (Invalid, reason.clone()),
        Artifact::Valid(_) => {}
    }
    if stage == Stage::Inputs {
        return (
            Ready,
            "input calibration validates against current controller identities".into(),
        );
    }
    let Some(previous) = c.completed.get(&stage) else {
        return (
            Unknown,
            "valid artifact requires explicit provenance adoption or calibration".into(),
        );
    };
    let Ok(current) = binding(s, stage, vec![]) else {
        return (Invalid, "invalid fingerprint or dependency".into());
    };
    if previous.artifact_sha256 != current.artifact_sha256
        || previous.camera_sha256 != current.camera_sha256
        || previous.sources_sha256 != current.sources_sha256
    {
        return (Stale, "artifact or dependency fingerprint changed".into());
    }
    (
        Ready,
        "saved configuration matches its recorded dependencies".into(),
    )
}
/// Classify saved work without opening hardware or files. Live blockers only replace
/// stages requiring work: an asleep controller does not invalidate saved work.
pub fn plan(snapshot: &Snapshot, checkpoint: &Checkpoint) -> Vec<StageReport> {
    [
        Stage::Camera,
        Stage::Gravity,
        Stage::Render,
        Stage::Registration,
        Stage::Inputs,
    ]
    .into_iter()
    .map(|stage| {
        let (mut state, mut reason) = assessment(snapshot, checkpoint, stage);
        if state != State::Ready {
            if let Some(blocker) = snapshot.blocked.get(&stage) {
                reason = format!("{blocker}; {reason}");
                state = State::Blocked;
            }
        }
        StageReport {
            stage,
            state,
            reason,
        }
    })
    .collect()
}
/// Explicit visual verification of existing artifacts. Does not alter artifacts.
pub fn adopt(stage: Stage, snapshot: &Snapshot, checkpoint: &mut Checkpoint) -> Result<(), String> {
    if checkpoint.version != 1 {
        return Err("unsupported checkpoint version".into());
    }
    let next = binding(snapshot, stage, vec![])?;
    checkpoint.completed.insert(stage, next);
    checkpoint.pending.remove(&stage);
    Ok(())
}
/// Persist this invalidation BEFORE starting any operation that may write an artifact.
/// Failure/cancellation simply leaves the stage pending. Working artifacts are untouched.
pub fn begin_stage(stage: Stage, checkpoint: &mut Checkpoint) {
    checkpoint.completed.remove(&stage);
    checkpoint.pending.insert(stage);
    if stage == Stage::Camera {
        for dependent in [Stage::Gravity, Stage::Registration] {
            checkpoint.completed.remove(&dependent);
            checkpoint.pending.insert(dependent);
        }
    }
}
/// Remember an exact successful capture before running its solver. Saving this
/// checkpoint does not claim that calibration succeeded.
pub fn record_capture(
    stage: Stage,
    snapshot: &Snapshot,
    checkpoint: &mut Checkpoint,
    path: PathBuf,
) -> Result<(), String> {
    if checkpoint.version != 1 {
        return Err("unsupported checkpoint version".into());
    }
    if !path.is_absolute() || !snapshot.recordings.contains(&path) {
        return Err("exact stage recording is unavailable".into());
    }
    let paths = checkpoint.recordings.entry(stage).or_default();
    if !paths.contains(&path) {
        paths.push(path);
    }
    Ok(())
}

/// Call only after solver acceptance, successful artifact writes and final validation.
pub fn record_success(
    stage: Stage,
    snapshot: &Snapshot,
    checkpoint: &mut Checkpoint,
    recordings: Vec<PathBuf>,
) -> Result<(), String> {
    if checkpoint.version != 1 {
        return Err("unsupported checkpoint version".into());
    }
    let next = binding(snapshot, stage, recordings)?;
    checkpoint.recordings.insert(stage, next.recordings.clone());
    checkpoint.completed.insert(stage, next);
    checkpoint.pending.remove(&stage);
    Ok(())
}
/// Carry bindings only for an intentional frame-preserving update, or a gravity
/// rotation with a correspondingly compensated registration. The old snapshot must
/// describe the exact pre-operation artifacts. Save a begin_stage checkpoint first,
/// then apply this to a clone of the pre-operation checkpoint after success.
pub fn preserve_camera_update(
    old: &Snapshot,
    new: &Snapshot,
    checkpoint: &mut Checkpoint,
    compensated_registration: bool,
) -> Result<(), String> {
    if checkpoint.version != 1 {
        return Err("unsupported checkpoint version".into());
    }
    let next_camera = binding(new, Stage::Camera, vec![])?;
    let proven_camera = assessment(old, checkpoint, Stage::Camera).0 == State::Ready;
    let mut next = checkpoint.clone();
    next.completed.insert(Stage::Camera, next_camera);
    next.pending.remove(&Stage::Camera);
    for stage in [Stage::Gravity, Stage::Registration] {
        let own_unchanged = valid(artifact(old, stage)) == valid(artifact(new, stage));
        let allow = stage == Stage::Gravity || own_unchanged || compensated_registration;
        if proven_camera && allow && assessment(old, checkpoint, stage).0 == State::Ready {
            let recordings = checkpoint.completed[&stage].recordings.clone();
            let mut candidate = binding(new, stage, vec![])?;
            candidate.recordings = recordings;
            next.completed.insert(stage, candidate);
            next.pending.remove(&stage);
        } else {
            next.completed.remove(&stage);
            next.pending.insert(stage);
        }
    }
    *checkpoint = next;
    Ok(())
}

const MAX_CHECKPOINT_BYTES: u64 = 64 * 1024;
fn is_hash(hash: &str) -> bool {
    hash.len() == 64
        && hash
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}
fn validate(c: &Checkpoint) -> std::io::Result<()> {
    if c.version != 1 {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "unsupported setup checkpoint version",
        ));
    }
    if c.recordings.values().flatten().any(|p| !p.is_absolute()) {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "invalid setup recording path",
        ));
    }
    for (stage, b) in &c.completed {
        if c.pending.contains(stage)
            || !is_hash(&b.artifact_sha256)
            || b.camera_sha256
                .iter()
                .chain(b.sources_sha256.iter())
                .any(|h| !is_hash(h))
            || matches!(stage, Stage::Gravity | Stage::Registration) != b.camera_sha256.is_some()
            || (*stage == Stage::Render) != b.sources_sha256.is_some()
            || b.recordings.iter().any(|p| !p.is_absolute())
        {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "invalid setup stage binding",
            ));
        }
    }
    Ok(())
}
/// SHA-256 of an artifact (up to 64 MiB). Caller independently validates semantics.
pub fn fingerprint(path: &std::path::Path) -> std::io::Result<String> {
    use sha2::{Digest, Sha256};
    use std::io::Read;
    let file = open_regular(path)?;
    const MAX_ARTIFACT_BYTES: u64 = 64 * 1024 * 1024;
    if file.metadata()?.len() > MAX_ARTIFACT_BYTES {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "artifact exceeds fingerprint size limit",
        ));
    }
    let mut file = file.take(MAX_ARTIFACT_BYTES + 1);
    let mut total = 0;
    let mut hasher = Sha256::new();
    let mut buffer = [0u8; 8192];
    loop {
        let count = file.read(&mut buffer)?;
        if count == 0 {
            break;
        }
        total += count as u64;
        if total > MAX_ARTIFACT_BYTES {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "artifact exceeds fingerprint size limit",
            ));
        }
        hasher.update(&buffer[..count]);
    }
    Ok(format!("{:x}", hasher.finalize()))
}
fn open_regular(path: &std::path::Path) -> std::io::Result<std::fs::File> {
    use std::os::unix::fs::OpenOptionsExt;
    let file = std::fs::OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK)
        .open(path)?;
    if !file.metadata()?.is_file() {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "setup path is not a regular file",
        ));
    }
    Ok(file)
}
/// Missing is distinct from malformed/unsupported state; never silently discard errors.
pub fn load(path: &std::path::Path) -> std::io::Result<Option<Checkpoint>> {
    use std::io::Read;
    let file = match open_regular(path) {
        Ok(file) => file,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(e),
    };
    if file.metadata()?.len() > MAX_CHECKPOINT_BYTES {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "setup checkpoint exceeds size limit",
        ));
    }
    let mut bytes = Vec::new();
    file.take(MAX_CHECKPOINT_BYTES + 1)
        .read_to_end(&mut bytes)?;
    if bytes.len() as u64 > MAX_CHECKPOINT_BYTES {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "setup checkpoint exceeds size limit",
        ));
    }
    let checkpoint: Checkpoint = serde_json::from_slice(&bytes)?;
    validate(&checkpoint)?;
    Ok(Some(checkpoint))
}
/// Atomic private write in the same directory; fsync both content and directory.
/// A failed save must stop setup rather than continuing with unrecorded success.
pub fn save(path: &std::path::Path, checkpoint: &Checkpoint) -> std::io::Result<()> {
    use std::io::Write;
    use std::os::unix::fs::OpenOptionsExt;
    use std::sync::atomic::{AtomicU64, Ordering};
    static NEXT: AtomicU64 = AtomicU64::new(0);
    validate(checkpoint)?;
    let bytes = serde_json::to_vec_pretty(checkpoint)?;
    if bytes.len() as u64 > MAX_CHECKPOINT_BYTES {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "setup checkpoint exceeds size limit",
        ));
    }
    match std::fs::symlink_metadata(path) {
        Ok(meta) if !meta.is_file() => {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "setup checkpoint is not a regular file",
            ))
        }
        Err(e) if e.kind() != std::io::ErrorKind::NotFound => return Err(e),
        _ => {}
    }
    let parent = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or_else(|| std::path::Path::new("."));
    let temp = parent.join(format!(
        ".cv1touch-setup-{}-{}.tmp",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .mode(0o600)
        .custom_flags(libc::O_NOFOLLOW)
        .open(&temp)?;
    let result = (|| {
        file.write_all(&bytes)?;
        file.sync_all()?;
        std::fs::rename(&temp, path)?;
        std::fs::File::open(parent)?.sync_all()
    })();
    if result.is_err() {
        let _ = std::fs::remove_file(&temp);
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    fn hash(c: char) -> String {
        std::iter::repeat_n(c, 64).collect()
    }
    fn snapshot() -> Snapshot {
        Snapshot {
            camera: Artifact::Missing,
            registration: Artifact::Missing,
            render: Artifact::Missing,
            render_sources: Artifact::Valid(hash('d')),
            inputs: Artifact::Missing,
            recordings: BTreeSet::new(),
            blocked: BTreeMap::new(),
        }
    }
    fn states(s: &Snapshot, c: &Checkpoint) -> Vec<(Stage, State)> {
        plan(s, c).into_iter().map(|r| (r.stage, r.state)).collect()
    }
    #[test]
    fn fresh_and_legacy_states_are_explicit() {
        use Stage::*;
        use State::*;
        let mut s = snapshot();
        assert_eq!(
            states(&s, &Checkpoint::default()),
            vec![
                (Camera, Missing),
                (Gravity, Blocked),
                (Render, Missing),
                (Registration, Blocked),
                (Inputs, Missing)
            ]
        );
        s.camera = Artifact::Valid(hash('a'));
        s.registration = Artifact::Valid(hash('b'));
        s.render = Artifact::Valid(hash('c'));
        s.inputs = Artifact::Valid(hash('e'));
        assert_eq!(
            states(&s, &Checkpoint::default()),
            vec![
                (Camera, Unknown),
                (Gravity, Unknown),
                (Render, Unknown),
                (Registration, Unknown),
                (Inputs, Ready)
            ]
        );
    }
    fn complete() -> (Snapshot, Checkpoint) {
        let mut s = snapshot();
        s.camera = Artifact::Valid(hash('a'));
        s.registration = Artifact::Valid(hash('b'));
        s.render = Artifact::Valid(hash('c'));
        s.inputs = Artifact::Valid(hash('e'));
        let mut c = Checkpoint::default();
        for stage in [
            Stage::Camera,
            Stage::Gravity,
            Stage::Render,
            Stage::Registration,
        ] {
            adopt(stage, &s, &mut c).unwrap();
        }
        (s, c)
    }
    #[test]
    fn unchanged_and_sleeping_require_no_captures() {
        let (mut s, c) = complete();
        for stage in [
            Stage::Camera,
            Stage::Gravity,
            Stage::Render,
            Stage::Registration,
            Stage::Inputs,
        ] {
            s.blocked.insert(stage, "asleep".into());
        }
        assert!(plan(&s, &c).iter().all(|r| r.state == State::Ready));
        s.inputs = Artifact::Missing;
        assert_eq!(plan(&s, &c)[4].state, State::Blocked);
    }
    #[test]
    fn external_camera_and_geometry_changes_invalidate_only_dependencies() {
        let (mut s, c) = complete();
        s.camera = Artifact::Valid(hash('f'));
        assert_eq!(
            states(&s, &c),
            vec![
                (Stage::Camera, State::Stale),
                (Stage::Gravity, State::Stale),
                (Stage::Render, State::Ready),
                (Stage::Registration, State::Stale),
                (Stage::Inputs, State::Ready)
            ]
        );
        s.render_sources = Artifact::Valid(hash('f'));
        assert_eq!(plan(&s, &c)[2].state, State::Stale);
    }
    #[test]
    fn full_world_begin_and_failed_finish_cannot_claim_success() {
        let (s, mut c) = complete();
        begin_stage(Stage::Camera, &mut c);
        assert_eq!(
            states(&s, &c),
            vec![
                (Stage::Camera, State::Stale),
                (Stage::Gravity, State::Stale),
                (Stage::Render, State::Ready),
                (Stage::Registration, State::Stale),
                (Stage::Inputs, State::Ready)
            ]
        );
        let before = serde_json::to_vec(&c).unwrap();
        assert!(record_success(
            Stage::Camera,
            &s,
            &mut c,
            vec![PathBuf::from("/missing.jsonl")]
        )
        .is_err());
        assert_eq!(serde_json::to_vec(&c).unwrap(), before);
        record_success(Stage::Camera, &s, &mut c, vec![]).unwrap();
        assert_eq!(plan(&s, &c)[0].state, State::Ready);
        assert_eq!(plan(&s, &c)[3].state, State::Stale);
    }
    #[test]
    fn preserved_world_and_compensation_require_both_previous_hashes() {
        let (old, c) = complete();
        let mut new = old.clone();
        new.camera = Artifact::Valid(hash('f'));
        let mut retained = c.clone();
        preserve_camera_update(&old, &new, &mut retained, false).unwrap();
        assert!(plan(&new, &retained)
            .iter()
            .all(|r| r.state == State::Ready));
        new.registration = Artifact::Valid(hash('e'));
        let mut uncompensated = c.clone();
        preserve_camera_update(&old, &new, &mut uncompensated, false).unwrap();
        assert_eq!(plan(&new, &uncompensated)[3].state, State::Stale);
        let mut compensated = c.clone();
        preserve_camera_update(&old, &new, &mut compensated, true).unwrap();
        assert_eq!(plan(&new, &compensated)[3].state, State::Ready);
        for stage in [Stage::Camera, Stage::Registration] {
            let mut unproven = c.clone();
            unproven.completed.get_mut(&stage).unwrap().artifact_sha256 = hash('0');
            preserve_camera_update(&old, &new, &mut unproven, true).unwrap();
            assert_eq!(plan(&new, &unproven)[3].state, State::Stale);
        }
    }
    struct Temp(PathBuf);
    impl Temp {
        fn new() -> Self {
            use std::sync::atomic::{AtomicUsize, Ordering};
            static NEXT: AtomicUsize = AtomicUsize::new(0);
            let path = std::env::temp_dir().join(format!(
                "cv1touch-setup-test-{}-{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            ));
            std::fs::create_dir(&path).unwrap();
            Self(path)
        }
    }
    impl Drop for Temp {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }
    #[test]
    fn exact_recordings_required_at_completion_but_not_for_completed_fit() {
        let temp = Temp::new();
        let recording = temp.0.join("capture.jsonl");
        std::fs::write(&recording, b"capture").unwrap();
        let (mut s, mut c) = complete();
        s.recordings.insert(recording.clone());
        record_success(Stage::Camera, &s, &mut c, vec![recording.clone()]).unwrap();
        std::fs::remove_file(&recording).unwrap();
        s.recordings.clear();
        assert_eq!(plan(&s, &c)[0].state, State::Ready);
        assert_eq!(
            c.completed[&Stage::Camera].recordings,
            vec![recording.clone()]
        );
        assert!(record_success(Stage::Camera, &s, &mut c, vec![recording]).is_err());
    }
    #[test]
    fn private_atomic_journal_round_trip_and_failed_save_preserves_previous() {
        use std::os::unix::fs::PermissionsExt;
        let temp = Temp::new();
        let path = temp.0.join("journal.json");
        let (_, mut c) = complete();
        assert!(load(&path).unwrap().is_none());
        save(&path, &c).unwrap();
        assert_eq!(
            std::fs::metadata(&path).unwrap().permissions().mode() & 0o777,
            0o600
        );
        assert_eq!(
            serde_json::to_vec(&load(&path).unwrap().unwrap()).unwrap(),
            serde_json::to_vec(&c).unwrap()
        );
        let previous = std::fs::read(&path).unwrap();
        c.version = 2;
        assert!(save(&path, &c).is_err());
        assert_eq!(std::fs::read(&path).unwrap(), previous);
        assert_eq!(std::fs::read_dir(&temp.0).unwrap().count(), 1);
    }
    #[test]
    fn rejects_malformed_unknown_oversized_symlink_fifo_and_directory() {
        use std::os::unix::fs::symlink;
        let temp = Temp::new();
        let path = temp.0.join("journal.json");
        for content in [
            "{",
            r#"{"version":2,"completed":{},"pending":[],"recordings":{}}"#,
            r#"{"version":1,"completed":{},"pending":[],"recordings":{},"future":true}"#,
            r#"{"version":1,"completed":{"camera":{"artifact_sha256":"bad","camera_sha256":null,"sources_sha256":null,"recordings":[]}},"pending":[],"recordings":{}}"#,
        ] {
            std::fs::write(&path, content).unwrap();
            assert!(load(&path).is_err(), "{content}");
        }
        std::fs::write(&path, vec![b' '; MAX_CHECKPOINT_BYTES as usize + 1]).unwrap();
        assert!(load(&path).is_err());
        let link = temp.0.join("link");
        symlink(&path, &link).unwrap();
        assert!(load(&link).is_err());
        assert!(save(&link, &Checkpoint::default()).is_err());
        let fifo = temp.0.join("fifo");
        let name = std::ffi::CString::new(fifo.as_os_str().as_encoded_bytes()).unwrap();
        assert_eq!(unsafe { libc::mkfifo(name.as_ptr(), 0o600) }, 0);
        assert!(load(&fifo).is_err());
        assert!(save(&fifo, &Checkpoint::default()).is_err());
        assert!(load(&temp.0).is_err());
    }
    #[test]
    fn sha256_matches_known_content() {
        let temp = Temp::new();
        let path = temp.0.join("artifact");
        std::fs::write(&path, b"abc").unwrap();
        assert_eq!(
            fingerprint(&path).unwrap(),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }
    #[test]
    fn pending_capture_survives_failed_fit_and_checkpoint_round_trip() {
        let temp = Temp::new();
        let path = temp.0.join("capture.jsonl");
        std::fs::write(&path, b"capture").unwrap();
        let (mut s, mut c) = complete();
        s.recordings.insert(path.clone());
        begin_stage(Stage::Camera, &mut c);
        record_capture(Stage::Camera, &s, &mut c, path.clone()).unwrap();
        let journal = temp.0.join("journal.json");
        save(&journal, &c).unwrap();
        let recovered = load(&journal).unwrap().unwrap();
        assert_eq!(recovered.recordings[&Stage::Camera], vec![path]);
        assert_eq!(plan(&s, &recovered)[0].state, State::Stale);
    }
}
