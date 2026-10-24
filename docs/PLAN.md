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

## Open items for v0.2

- Serve mode: interest selection (sync aircraft by rate, MLAT targets),
  results back to clients.
- Per-receiver bias learning (antenna position errors) from holdout residuals.
- Optional native Mode A/C intake from readsb-ingest's Beast relay (today it
  carries only `0000` frames, so it is off).
