# mlat-ng — plan and 7-day head-to-head test

Status: v0.1, written 2026-10-23/24. Test window: 7 days from the moment all
three contenders receive traffic.

## Why a new server

The production server (flyitalyadsb/mlat-server, Python + Cython, a fork of
wiedehopf/mutability) works, but:

- **One core per process, no way around it.** It is a single-threaded asyncio
  process. The network is split into 5 geographic shards; the continental EU
  core cannot be cut again without losing ~7% of the receiver pairs, and it
  already runs at ~0.8 core off-peak.
- **Synchronous I/O on the event loop.** On 2026-10-23 a restart left the
  shards blocked up to 53% of the time on ceph-fuse writes for ~20 minutes.
- **Known defects** (from the code study of 2026-10-23): the fake-Mode-A/C
  output filter hides real African/Qatari/Mexican aircraft; a NaN covariance
  drops a whole batch of groups; the Gillham (Q=0) altitude decoder is wrong
  for 768 of 1280 codes; MST timestamp normalisation uses n-1 of up to
  n(n-1)/2 clock pairs.

## Design (what makes it better, not just faster)

| Stage | mlat-server | mlat-ng |
|---|---|---|
| Process model | 1 thread, 5 shards | network on all cores, one engine thread owning all state, **one process for the whole network** |
| Clock pairs | windowed fit + PI drift loop | **2-state Kalman filter** (offset, drift) per pair; variance grows with age |
| Timestamp normalisation | minimum spanning tree (n-1 edges) | **joint weighted least squares over every usable pair**, robust edge rejection, full covariance |
| Solver | scipy leastsq, diagonal errors, numeric Jacobian | Gauss-Newton/LM with **full time covariance** (whitened), analytic Jacobian, multi-start + mirror-lobe check, leave-one-out |
| Tracking | UKF fed with fixes only | EKF fed with fixes **and directly with TDOAs** (2-3 receiver groups keep tracks alive) |
| Mode A/C | client-side false_ac only, no association | **native**: false_ac decoded, each solution **associated with a Mode S track** (squawk / Mode C altitude + position) where it becomes an extra measurement; only consistent unassociated solutions become Mode A/C tracks (`~F0xxxx`) |
| Self-measurement | none | **5% of ADS-B sync points held out** of clock sync and multilaterated against their own ADS-B position: a continuous, honest accuracy figure |
| Output filter | drops every address <= 0xF423F | exact false_ac pattern only |
| I/O | sync file writes on the loop | no file I/O in the engine; state over HTTP |

## The test

```
feeder ──► Spectrum ──► mlat-proxy ──► mlat shard (prod, unchanged)
                            │ tee (same bytes, replies discarded)
                            ▼
                    mlat-shadow-hub ×3 ──► mlat-ng   (ours)
                    (inflate, re-handshake) └──► mlatd  (flightportrait, reference)

prod shards :3040 ┐
mlat-ng     :31003├──► mlat-judge ◄── ADS-B truth (readsb aircraft.json, ADS-B-only)
mlatd       :31003┘
```

- **Same input**: every contender sees exactly what each feeder sends to its
  production shard. Limitation: feeders send only what *production* asks for
  (selective traffic), so contenders compete on production's data selection.
- **Zero user impact**: contender results go only to the judge, never to readsb
  or the public map; the tee never blocks or delays the primary path; the hub
  lets contenders restart without losing feeders.
- **Metrics** (judge, per hour and cumulative): accuracy vs ADS-B truth
  (p50/p90/p99, > 1 km, > 5 km), **head-to-head on the same (aircraft, 2 s)
  keys**, coverage of Mode-S-only aircraft (aircraft-minutes), plausibility
  (implied speed > 1,100 kt / 10 km jumps), latency, Mode A/C separately;
  plus each contender's own CPU/RAM.

## Decision rule at day 7

mlat-ng replaces production only if, over the 7 days:

1. head-to-head median error <= production's, and p90 not worse by > 10%;
2. Mode-S-only coverage (aircraft-minutes) >= production's;
3. implausible-fix rate <= production's;
4. no crash loops; CPU for the whole network < 4 cores at peak.

Otherwise: keep production, fix what the numbers show, and run again.

## Rollout after a win (not part of this test)

Serve mode (selective traffic + results to clients) per region through
`PROXY_SHARD_MAP`: `rest` first, then `eu-w`, …; rollback is one line.

## Feed input

Added 2026-10-24 (v0.2), as a second contender, `mlat-ng-feed`: the same
engine fed from readsb-ingest's receiver-tagged Beast output (port 4906, one
per cell) instead of mlat-client. It answers the test's main limitation:
contenders only see what production asks feeders for.

Why it works (measured on two 60 s production captures, 11:00 and 11:44 UTC):

- the feeders' 12 MHz timestamps reach 4906 untouched (readsb source: no hop
  rewrites them); pairwise clock residuals p50 0.15 us, the same at 10 km and
  800 km;
- the receiver id is the first 64 bits of the feeder's UUID: 89% match an
  mlat-client handshake, which gives the position (`/receivers.json`);
- feeders' reduced output still forwards nearly every DF17 position, so clock
  sync is dense: 4-6% of receivers without a sync partner, against 26% on the
  mlat-client path.

What the replay of those captures gives with this code (`mlat-ng replay`):

| | capture 1 | capture 2 |
|---|---|---|
| bench, published error (ADS-B aircraft from non-ADS-B replies) | p50 74 m, p90 270 m, >1 km 1.6% | p50 76 m, p90 270 m, >1 km 1.5% |
| Mode-S-only aircraft located in the minute | 134 | 125 |
| of which production had no result for | 89 | 61 |
| implausible jumps in the Mode-S-only output | 0.5% | 1.0% |
| CPU | 4.1 s per 60 s | 4.1 s per 60 s |

Limits: non-ADS-B replies are only the subset each feeder's reduce timers
forward, so copies of one transmission are sparser than on the mlat-client
path; 11% of feed ids have no position (no or different UUID); Radarcape GPS
and zero timestamps arrive unflagged (the adapter checks each clock's rate
before using it); feeders move between cells (both are read; receivers are
keyed by UUID).

## Open items for v0.2

- Serve mode: interest selection (sync aircraft by rate, MLAT targets),
  results back to clients.
- Per-receiver bias learning (antenna position errors) from holdout residuals.
- Hybrid: feed and mlat-client in one engine, linking each receiver's two
  clocks (same hardware, constant offset).
- Mode A/C: the feed carries none today (only `0000` heartbeats); it needs
  readsb to forward Mode A/C on reduced output when asked (`1J`) and
  `--modeac-auto` on feeders.
