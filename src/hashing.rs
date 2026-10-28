//! Hashing for every map and set: foldhash, seeded once per process.
//!
//! std's `RandomState` gives each map its own random seed, so iteration order
//! changed from run to run, and with it the order of some floating-point sums:
//! two replays of the same capture differed in ~0.1% of the published lines
//! (2026-10-28). Maps now take the process seed (`set_seed`), and the code
//! that iterates them no longer depends on the order (sorted where it
//! matters), so the output is the same for any seed. Replays and tests use a
//! fixed seed; a server draws a random one, which keeps crafted collisions in
//! keys that come from feeders (message bytes) out of reach. foldhash is also
//! several times cheaper than SipHash on the small keys used here.

use std::hash::BuildHasher;
use std::sync::atomic::{AtomicU64, Ordering};

use foldhash::fast::{FixedState, FoldHasher};

pub type HashMap<K, V> = std::collections::HashMap<K, V, Seeded>;
pub type HashSet<K> = std::collections::HashSet<K, Seeded>;

static SEED: AtomicU64 = AtomicU64::new(0);

/// The seed of every map created from now on.
pub fn set_seed(seed: u64) {
    SEED.store(seed, Ordering::Relaxed);
}

#[derive(Clone, Copy, Debug)]
pub struct Seeded(FixedState);

impl Default for Seeded {
    #[inline]
    fn default() -> Self {
        Seeded(FixedState::with_seed(SEED.load(Ordering::Relaxed)))
    }
}

impl BuildHasher for Seeded {
    type Hasher = FoldHasher;

    #[inline]
    fn build_hasher(&self) -> FoldHasher {
        self.0.build_hasher()
    }
}
