# mlat-ng on many cores: study

Status: study, 2026-10-28.
- **Step 0 is implemented** (mlat-ng #17): the output does not depend on map
  order (`hashing.rs`).
- **Step 1 is implemented:** stage 1 plus aircraft workers (`worker.rs`),
  still on the engine thread. The output is byte-identical to step 0 for any
  number of workers ("Step 1 as implemented").
- **Step 2 is implemented:** the workers can run on their own threads
  (`MLAT_NG_WORKERS`). Threaded output is identical to inline output; see
  "Step 2 as implemented".
- **Step 3 is implemented:** the coverage maps run on their own thread
  (`covworker.rs`), with Lightcone's questions and the saves. It starts with
  the worker threads. See "Step 3 as implemented".
- Still open: the hybrid router (one tokio task), and result lines in serve
  mode.
- The experiments under "Measured" used a prototype with a fixed Fx hasher.
- The implementation uses foldhash, seeded once per process: random in the
  server, fixed in replays and tests. Its output was checked to be the same
  for any seed ("Step 0 as implemented").

## Why

PLAN.md chose "network on all cores, one engine thread owning all state". Only
the network runs on many cores. Everything that makes a position runs on one
thread: clock sync, grouping, normalisation, solving, tracks, coverage, survey
and the altitude model. That was fine at 0.36 core for 3,203 receivers
(24/10). Since then coverage (Lightcone), survey, the altitude model, 2-site
and the hybrid have been added to the same thread.

Live, 2026-10-28 ~09:00Z (`top -H` on nesso-worker-9, EPYC 7542, image
sha-5d7dba6), before the EU peak:

| process | `engine` thread | `net` threads | |
|---|---|---|---|
| mlat-ng (3,221 receivers) | 72% | 3–5% each | headroom shrinking |
| mlat-ng-feed (2,461) | 77% | 8–11% each | headroom shrinking |
| mlat-ng-hybrid (4,870) | **99.4%** | 86%, 26%, 24%, 20% | **saturated, losing data** |

The hybrid's losses since its start 16 h earlier:
- `hybrid_engine_full_total` 63.9 M;
- `msgs_dropped_queue_full_total` 46.8 M;
- engine queue 60–180 k;
- `engine_late_ms_max` 4,025.

One hybrid `net` thread at 86% is very probably the hybrid router. The router is
a single tokio task, so it is a second one-core ceiling (to be confirmed, see
"Open points").

A faster core buys a constant factor, once. The ceiling is in the design.

## Where the engine's time goes

The profiles are from macOS `sample` on `origin/main` (5d7dba6), replaying cap3
(600 s of the real Beast feed, 2026-10-24). Shares are of engine time only. The
capture inflate, Beast parsing and hybrid router are left out; they run on
`net` threads live.

| | feed path | hybrid path (70% stand-in clients, 2-site publish, survey apply, mature coverage) |
|---|---|---|
| Coverage::observe (on ADS-B sync points) | 28.3% | 23.0% |
| **group processing** | **43.9%** | **47.0%** |
| · normalise | 15.3% | 12.0% |
| · solver | 19.8% | 14.4% |
| · 2-site try (`twosite::solve`) | – | 13.3% |
| · track | 5.0% | 3.8% |
| sync / clock pairs / grouping (rest of `handle_at`) | ~14% | ~14.5% |
| survey observe | 4.8% | 4.0% |
| altitude model insert + estimate | 4.8% | 4.7% |
| expire + housekeeping | 2.3% | 1.8% |
| edge prediction + build (would stay on the sequential thread) | 1.7% | 1.5% |
| *SipHash, all of the above (default `HashMap` hasher)* | *13.3%* | *13.8%* |

Two thirds of the engine's time is not sequential by nature:
- **Group processing** depends on other aircraft only through read-only state.
- **Coverage** is a side product for Lightcone and 2-site.

What is truly sequential is the clock-pair Kalman filters. A pair's
observations must be applied in time order. Together with grouping, that is
~15% of today's load.

## The rule: no loss of quality

1. **Reference = today's engine, made reproducible.** Today the same input gives
   slightly different output from run to run (see "Measured"), because some
   floating-point sums and a few first-match choices follow `HashMap`
   iteration order. Step 0 makes every such place go in key order, so the
   output no longer depends on the hash seed. Quality is unchanged, to the last
   bits.
2. **The parallel engine must give BIT-IDENTICAL output to the reference** on
   every replay capture, for any number of workers. "Bit-identical" means:
   - the same SBS lines, sorted by (time, address);
   - the same bench lines;
   - the same holdout / ADS-B-check / 2-site error sets.

   A yes/no test with no statistics. Any race or ordering bug shows up as a
   diff.
3. **Where exactness would cost a lock-step,** we change the reference itself
   in a deterministic way, and validate that change once, statistically,
   against today. In the end there is one such place: 2-site coverage reads
   (see Design). It was measured. Mode A/C association turned out to stay
   exact (step 2).

## What the single thread gives for free (and must be kept)

- **Tick semantics.**
  - Groups are processed only at the 50 ms tick, in due order.
  - Each group reads clock pairs, receivers, the altitude model and coverage as
    they are at that tick.
  - Events arriving between two ticks are all applied before the next tick's
    groups.
- **Per-aircraft order.** An aircraft's ADS-B truth, Comm-B velocities, fixes
  and track updates happen in one total order.
- **Within a tick, Mode S groups of different aircraft don't interact.** Their
  cross-aircraft writes are commutative counters: stats, `r.used`, coverage
  `count_fix`, diagnostics deques. The one exception is Mode A/C association,
  which reads every aircraft's track.

## State map

| state | written by | read by | goes to |
|---|---|---|---|
| receivers (`rxs`: pos, jitter, noise, suspect, peers, WallMap, serve, tx; parked ones too) | connect/disconnect (park, resume), sync, mlat, housekeeping, survey apply, expire | build, normalise, bridges, emit, 2-site, serve | **stage 1**; jobs carry what they need |
| clock pairs (with any set-aside model), sync table | sync, housekeeping, clock reset, position update, park expiry | normalise (edges) | **stage 1**; edges frozen into the job |
| groups being collected | mlat | process_due | **stage 1** |
| aircraft, sync half: tracking, seen_by, sync_ok/bad, last_adsb, geo_baro, alt_ft/squawk/callsign* | seen/lost, sync, GeoBaro, housekeeping, frame decode* | serve, sync validation, altitude-model insert, existence checks | **stage 1** |
| aircraft, track half: track, last_fix, last_out, ADS-B truth + history, Comm-B, 2-site records, alt_ft/squawk/callsign* | sync (forwarded), group processing, 2-site | group processing, emit, ADS-B check, 2-site | **worker owning the ICAO** |
| altitude model | sync (insert), housekeeping (expire) | sync validation (stage 1), altitude constraint (worker) | stage 1; workers read a per-tick `Arc` snapshot (small: 1,913 cells live) |
| coverage maps | sync (observe), connect (place), fix (count) | 2-site (near, log_lik), HTTP | **coverage thread**; 2-site reads a snapshot |
| survey | pair updates | housekeeping (apply), HTTP | stage 1 (or its own thread later) |
| Mode A/C tracks | Mode A/C groups | Mode A/C groups | **Mode A/C step** (see below) |
| diagnostics (holdout, ADS-B check, 2-site errors, diag_bad) | groups | /metrics, /debug | per-worker buffers, merged in sequence order |

\* alt_ft, squawk and callsign are written from each group's decoded frame and
read by serve mode, so stage 1 keeps them. Stage 1 decodes the frame when it
freezes the job, which is exactly when `process_group` does it today. The job
carries the frame, so the worker applies the same update.

## Design

```
net threads ──► STAGE 1 (one thread, sequential; replaces the engine thread)
                 · sync: validate, clock pairs, survey, altitude-model insert
                 · mlat: collect groups; at the tick, FREEZE each due group into a job
                 · seen/lost, serve, connect/disconnect, receiver health
                 │ one ordered channel per worker            │ observe(...) messages
                 ▼                                           ▼
        AIRCRAFT WORKERS ×N (icao hash → worker)      COVERAGE THREAD
         · own the aircraft "track half"                 · owns coverage maps
         · normalise → clusters → solve → track          · publishes a snapshot every 60 s
         · 2-site, ADS-B check, emit                     · Lightcone HTTP reads
```

- **The frozen job is exact by construction.** At the tick, stage 1 does what
  `process_group` does today before any heavy work:
  - build the nodes;
  - compute the member edges (`Engine::edge`) and, for disconnected groups,
    the bridge edges;
  - map the WallMap time;
  - set the `known`/`last_adsb` flags;
  - apply the frame decode.

  The job carries those numbers: they are the ones today's code would read at
  that tick. Normalisation is a pure function of them. The heavy work is
  normalise, solve, track and 2-site (~45% of today, up to 2/3 of the
  remaining time). It moves to the workers.
- **Per-aircraft order is kept.** Stage 1 sends a worker, in event order, three
  kinds of messages: the ADS-B updates of its aircraft, the seen/lost changes,
  and the jobs. A worker therefore sees its aircraft's history exactly as the
  single thread did.
- **Altitude model.** It is written between ticks. Workers read an `Arc`
  snapshot published at each tick (cheap: ~2 k cells), which is exactly the
  state the single thread had at that tick.
- **Housekeeping every 10 s is a short barrier.** Whether an aircraft survives
  depends on both halves: tracking (stage 1) and a non-stale track (worker).
  Stage 1 sends the housekeeping marker. Each worker answers with what it
  drops when it reaches the marker. Stage 1 waits for the answers (a few ms),
  so existence stays exactly as today.
- **Coverage moves off the sequential path.** That is 23–28% of today's load,
  off stage 1.
  - Coverage `observe` needs "who sees this aircraft". The coverage thread
    gets the same seen/lost stream, so it keeps its own exact copy.
  - *Change 1 to the reference:* 2-site reads coverage as of the last 60 s
    boundary instead of the current tick. Coverage maps evolve with a 24 h
    half-life, so this is measured below. It is the only input to a position
    that is not exact by construction.
- **Mode A/C association** reads every aircraft's track. This stays exact,
  with no change to the reference:
  - Stage 1 asks every worker, in its message stream, for its nearest
    matching track. A worker answers only after doing the jobs sent before
    the question, so all of them answer about the tracks as they are at that
    point of the tick.
  - The associated TDOA update goes back as a message to the track's owner.
  - Stage 1 waits for those answers; Mode A/C groups are a few per second at
    most.
  - (The first draft deferred Mode A/C groups to the end of the tick: change
    2. It is not needed.)
- **Outputs.** Each worker sends to the same broadcast channels. Within an
  aircraft the order is as today; lines of different aircraft may interleave
  differently, which none of the consumers depend on.
- **Backpressure.** The channels to the workers are bounded and block. A
  worker that falls behind slows stage 1, which slows input exactly as today.
  Nothing is dropped inside the engine.
- **Map order decides nothing** (step 0, done). Every map takes a per-process
  seed. Every place where the order reached a result now goes in key order.
  It is needed for reproducibility, and it removes most of the 13–14% SipHash
  cost.

## Expected capacity (same hardware)

Hybrid shares of today's engine time:

| thread | load | headroom |
|---|---|---|
| stage 1 | ~15% (sync, pairs, grouping) + 4% (survey) + 3% (altitude insert) + 3% (freezing jobs, decode) + 2% (expire/housekeeping) ≈ **27%**, before step 0's hashing gain | ~3.7× |
| coverage thread | ~23% | ~4× (its own algorithm can be made much cheaper later) |
| workers | ~47% spread over N | scales with cores |

A later step 2 could split stage 1 by sharding the sync table and clock pairs.
It is not needed for the growth we expect.

## Verification

1. Step 0, single thread, reproducible:
   - Two runs give byte-identical output on cap2, cap3 and the hybrid replay.
   - Against today's main, all metrics are identical up to the last-bit noise
     we already have between today's own runs.
2. Step 1, still single thread, split into stage 1, jobs and an inline worker:
   bit-identical to step 0.
3. Reference changes 1 and 2 behind flags: a replay comparison with the paired
   bootstrap already used in the algorithm study (2026-10-25). Inputs:
   - cap3 hybrid replay with 2-site publish;
   - the 2-site bench (`--bench-two-sites`);
   - a client-path capture for Mode A/C.
4. Threads: workers = 1, 2, 4, 8, on every capture, 20 repetitions each.
   Everything must be bit-identical to step 2's reference. A CI test does the
   same on the simulator (sim.rs).
5. Live:
   - The hybrid goes first. It is the saturated one, so its quality can only
     improve when the drops stop.
   - The judge compares it before and after.
   - Then mlat-ng and the feed.
   - Not before the head-to-head verdict (31/10).

## Open points

- **The hybrid router.** It is one tokio task (`hybrid::run`). Its state is per
  receiver key, so it can be sharded by key. Before doing that, confirm the
  86% `net` thread is the router, with a CPU profile of the live pod.
- ~~Fx is not collision-resistant~~. Done in step 0: foldhash, with a random
  seed per server process. The output does not depend on the seed, so keeping
  it secret costs nothing.
- Survey observe could get its own thread like coverage. It is not needed for
  the capacity above.
- The serve mode is off live. Its reads are all stage-1 state; check them again
  when it rolls out.

## Measured (2026-10-28, cap3 hybrid replay)

Setup:
- Capture: cap3, 600 s.
- Hybrid path: 70% stand-in clients (0.8 s delay, 1 s batches), 2-site publish,
  survey apply every 300 s.
- Starting coverage: maps from an earlier cap3 replay.
- Output: 17,808 SBS lines.

| comparison | SBS lines differing | summary metrics differing | 2-site accepted (n, p50, p90) |
|---|---|---|---|
| main vs main (same binary, same input) | **20** | 14 (each ±1–3, e.g. published 110,374 / 110,373) | 150, 161 m, 390 m — same |
| Fx vs Fx (step 0: deterministic hasher) | **0** (byte-identical, same order) | 0 (only solve timing) | same |
| Fx vs main | 55 (the main-vs-main noise) | 10, each ±1–3 | same |
| Fx + coverage snapshot 60 s vs Fx | **0** (byte-identical) | 2, diagnostics of *rejected* 2-site solutions (over 5 km 177→179) | same |

- **Today's engine is not reproducible.** The same binary on the same input
  gives a slightly different output.
  - Cause: std `HashMap` iteration order, which is seeded per process.
  - Mechanism: that order feeds floating-point sums (receiver noise scale,
    reject rate) and the survey's window handling.
  - Quality is unaffected; the differences are last-bit chaos. But it rules
    out "bit-identical" as a test until it is fixed.
- **Step 0 works.** With a fixed-seed Fx hasher everywhere, two runs are
  byte-identical, and all 240 tests pass. This is the reference the parallel
  engine must match.
- **Change 1 (2-site reads 60 s-old coverage) changes nothing that is
  published on this capture.** The SBS output is byte-identical. The same 150
  2-site solutions are accepted, with the same errors. Only two counters
  of rejected solutions move.
  - Caveat: one 10-minute capture. Repeat on cap2 and on a fresh capture with
    live coverage maps before relying on it.
- **Speed of step 0 alone (Fx instead of SipHash).** Measured on the cap3 feed
  replay, run sequentially, `/usr/bin/time` user CPU:
  - main: 104.4 s and 102.6 s;
  - Fx: 77.4 s and 79.7 s;
  - **−24%**, with the same published positions (109,965), the same published
    error (p50 97 m, p90 307 m) and the same holdout (p50 50 m).
  - The figure covers the whole process, ingest included. The engine thread's
    own share is at least the 13–14% SipHash it spent.

### Step 0 as implemented (foldhash, seeded per process)

Seed independence: cap3 replayed with `--hash-seed 0` and `--hash-seed 1`.

| replay | SBS output, seed 0 vs seed 1 | summary metrics differing |
|---|---|---|
| feed path | byte-identical (17,465 lines) | 0 |
| hybrid path (2-site publish, survey apply, coverage) | byte-identical (17,809) | 0 (only solve timing) |
| survey perturbation test (5% of receivers moved 300 m, apply every 300 s) | byte-identical (17,637) | 0 |
| serve simulation (`--serve-sim select`) | byte-identical (23,782) | 0 |

What had to change for that: every place where map order reached a result
now goes in key order.
- Clock pairs, for the receiver noise scale and reject rate.
- Survey windows, as they close.
- Feed events: registry, Seen/Lost, idle receivers.
- Mode A/C association. It now takes the nearest matching track, then the
  lowest address, instead of the map's first.
- Mode A/C tracks.
- Ties in the survey corrections.
- Serve selection and the serve simulator.

Against main on the hybrid replay, every metric moves by ±1–3: the same noise
main shows between two of its own runs. Published p50 and p90 are unchanged.

CPU, cap3 feed replay, sequential, `/usr/bin/time` user:
- main: 104.1 s and 114.7 s today (102.6 and 104.4 s earlier);
- foldhash: 81.3 s and 81.2 s;
- **about −22%**.

All 240 tests pass, with the default seed and with another one.

### Step 1 as implemented (stage 1 + inline aircraft workers)

The engine is now in two stages.
- `engine.rs` is stage 1. It keeps the receivers, clock pairs, sync points,
  grouping, Mode A/C, and the sync half of every aircraft: who sees it, its
  ADS-B timing, its altitude, and what the serve mode reads.
- At the tick, stage 1 freezes each due Mode S group into a `GroupJob`:
  - its nodes and copies;
  - the clock edges, exactly as `normalize` would read them at that moment,
    bridge edges included;
  - the receivers' identities;
  - which receivers report seeing the aircraft (2-site).
- Holdout groups become a `HoldoutJob`. It carries the altitude model's
  estimate, and the worker counts it only when the group normalises, as
  before.
- `worker.rs` owns the track half of each aircraft, sharded by address, and
  does what follows: normalisation, clustering, solving, tracks, Comm-B,
  2-site, the ADS-B check, output.
  - Besides its jobs and the ADS-B updates of its aircraft, it reads only the
    altitude model and the coverage maps.
  - It writes back only commutative counters (`Effects`: receivers' use,
    coverage fix counts, replay dump lines).
  - Every 10 s, housekeeping asks the workers which aircraft still have a
    track before both halves forget an aircraft.
- Mode A/C association still reads every worker's tracks from stage 1. That
  is fine inline; step 2 asks the workers instead (see Design).

Checked: cap3 replayed with `--workers 1` and `--workers 4` against step 0
(a975524) on the feed path, the hybrid path, the survey perturbation test and
the serve simulation.
- The SBS output and the 2-site dump are byte-identical.
- The summaries are identical except the solve timing.
- All 240 tests pass.


### Step 2 as implemented (workers on their own threads)

- **Views as messages.** The workers no longer read stage 1's state. The
  altitude model reaches them as a snapshot at every tick that follows a
  change. The coverage maps for the 2-site path arrive as a snapshot every
  `COVERAGE_SNAPSHOT_S` = 60 s (change 1). Each view comes in the same
  message stream as the jobs, so a job always sees the view of its own tick.
- **Questions with replies.** Housekeeping and Mode A/C association send each
  worker a question and wait for the reply. A worker answers once it has done
  everything sent before the question.
- **Effects go back on a channel.** Receivers' use, coverage fix counts and
  replay dump lines are applied at the next tick. `count_fix` never rolls a
  map back to an earlier day.
- **`Pool`** runs the workers either inline or on threads, with the same
  code on the same messages. Each thread has a bounded queue of 65,536; when
  it is full, stage 1 waits.
- **Configuration.** `MLAT_NG_WORKERS=N` gives N worker threads; 0, the
  default, keeps one worker on the engine thread. For replays, use
  `--workers N --threads`.

Checked, cap3:

| comparison | SBS output | summary |
|---|---|---|
| inline vs step 1: feed, survey perturbation, serve | identical, same order | identical |
| inline vs step 1: hybrid (2-site publish) | identical, same order | 2 counters of *rejected* 2-site solutions (change 1) |
| threads (1, 2, 3, 4, 8 workers) vs inline: feed, hybrid, survey perturbation | identical as sorted lines (aircraft interleave differently) | identical but solve timing; 2-site dump identical as sorted lines |
| threads (4) vs inline: serve simulation | identical as sorted lines | ±1–3 of 14.6 M events |

The serve-simulation difference comes from shared client queues. Result
lines to served clients now leave from the workers. They share each client's
bounded queue with stage 1's traffic requests, and the simulated client reads
that queue on its own timer, so the interleaving of the two is not fixed.
Serve mode is off live. If it matters, the fix is to send result lines from
stage 1 at the next tick, which keeps their order fixed.

240 tests pass.

### Step 3 as implemented (coverage maps on their own thread)

- **The maps are off stage 1.** Counting ADS-B samples into the maps was
  ~25% of the engine thread. The maps now live in `covworker.rs` and get, in
  stage-1 order:
  - samples, each with the coverage ids of the receivers that report seeing
    the aircraft at that moment;
  - receivers placed, removed and named;
  - fixes to count;
  - Lightcone's questions;
  - saves.
- **Stage 1 does the per-aircraft sampling** (`SAMPLE_EVERY`), so only
  samples that count are sent.
- **The 2-site snapshot** is taken by the maps where stage 1's request sits
  in their stream. Each worker waits for it where the matching message sits
  in its own stream, so it is the same snapshot as in step 2.
- **Lightcone** is answered by the maps' thread, no longer by the engine.
- **The thread** starts together with the worker threads
  (`MLAT_NG_WORKERS` > 0).

Checked, cap3. The coverage maps saved at the end of each replay were
compared after decompression.

| comparison | SBS output | coverage maps saved | summary |
|---|---|---|---|
| inline vs step 2: feed, hybrid, survey perturbation, serve | identical, same order | identical | identical (timing aside) |
| threads (4, 8) vs inline: feed, hybrid, survey perturbation | identical (sorted) | identical | identical (timing aside) |
| threads (4) vs inline: serve simulation | identical (sorted) | identical | ±1–3 of 14.6 M events (step 2) |

One exception remains. Against step 2, 3 of the 2.6 M lines of the hybrid
replay's 2-site dump differ, in the 15th digit of one rejected solution's
error estimate; the positions and decisions are the same. The maps (the
only 2-site input step 3 touches) are identical, and step 3 itself gives the
same dump inline and threaded. The cause is not pinned down. 240 tests pass.

### Step 4 (fewer wake-ups, altitude model by changes)

After step 3 the hybrid's engine thread was still at ~75%. A profile of the
threaded hybrid replay showed where its own work went:
- ~half of it sending messages to the workers, almost all of that in
  `semaphore_signal_trap`. Each message woke a sleeping worker: one system
  call, tens of thousands of times a second;
- ~6% copying the altitude model for the workers at every tick.

The two changes:
- **Batches.** A threaded worker gets its messages in batches of up to 512.
  Stage 1 sends what it has at the end of each tick, before every question,
  and when a batch is full. The order is unchanged.
- **Changes, not copies.** Stage 1 sends each insert and expiry of the
  altitude model, and every worker keeps its own copy, identical at every
  tick. Every 10 s stage 1 sends an order-independent fingerprint of its
  model; a worker whose copy differs counts it in
  `alt_replica_mismatch_total` and logs a warning.

Checked, cap3:
- **Against step 3 + router fix (inline):** byte-identical SBS output,
  2-site dump and saved coverage maps on the feed path, the hybrid path, the
  survey perturbation test and the serve simulation.
- **Threads (4, 8) against inline:** identical on all but the serve
  simulation, which keeps its ±1–2 from shared client queues.
- **Fingerprint:** no mismatch in any run. 241 tests pass, including one for
  the fingerprint.

In the threaded hybrid replay, stage 1's own work fell 41% (15.7 k → 9.3 k
samples), and sending messages fell 92% (8.1 k → 0.7 k).
