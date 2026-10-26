//! Engine state on disk: written atomically (temporary file, fsync, rename),
//! and in the background during normal running so a slow volume (CephFS can
//! stall for seconds, 2026-10-23) never holds up the engine thread. At
//! shutdown the engine writes synchronously, since that is the last chance.

use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

pub fn write_atomic(path: &str, bytes: &[u8]) -> anyhow::Result<()> {
    let tmp = format!("{path}.tmp");
    let mut f = std::fs::File::create(&tmp)?;
    f.write_all(bytes)?;
    f.sync_all()?;
    std::fs::rename(tmp, path)?;
    Ok(())
}

/// One background writer per file: a write that finds the previous one
/// still going is skipped (the next one brings the newer state anyway).
#[derive(Default, Clone)]
pub struct Writer {
    busy: Arc<AtomicBool>,
}

impl Writer {
    pub fn spawn(&self, path: String, bytes: Vec<u8>) {
        if self.busy.swap(true, Ordering::AcqRel) {
            tracing::warn!(path, "previous save still running: this one skipped");
            return;
        }
        let busy = self.busy.clone();
        let r = std::thread::Builder::new().name("save".into()).spawn(move || {
            if let Err(e) = write_atomic(&path, &bytes) {
                tracing::warn!(path, error = %e, "state not saved");
            }
            busy.store(false, Ordering::Release);
        });
        if let Err(e) = r {
            tracing::warn!(error = %e, "save thread not started");
            self.busy.store(false, Ordering::Release);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn writes_atomically_and_skips_while_busy() {
        let dir = std::env::temp_dir().join(format!("mlat-ng-persist-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("x.bin").to_string_lossy().to_string();
        write_atomic(&p, b"one").unwrap();
        assert_eq!(std::fs::read(&p).unwrap(), b"one");
        assert!(!std::path::Path::new(&format!("{p}.tmp")).exists());
        let w = Writer::default();
        w.busy.store(true, Ordering::Release);
        w.spawn(p.clone(), b"two".to_vec());
        assert_eq!(std::fs::read(&p).unwrap(), b"one", "skipped while busy");
        w.busy.store(false, Ordering::Release);
        w.spawn(p.clone(), b"three".to_vec());
        for _ in 0..100 {
            if std::fs::read(&p).unwrap() == b"three" {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        assert_eq!(std::fs::read(&p).unwrap(), b"three");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
