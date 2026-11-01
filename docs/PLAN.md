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

## Serve mode

Added 2026-10-26, off by default. mlat-ng becomes the real server for the
feeders that connect to `--serve-listen` (`MLAT_NG_SERVE_LISTEN`): it asks
them for traffic and answers them. `--listen` keeps taking the shadow copy of
everyone else, so one engine can serve one region and still use the whole
network's data (mixed mode).

- **Selection** (`src/interest.rs`): mlat-server's rules ported as they are,
  so feeders send what they already send to production. mlat: an aircraft
  seen by >= 2 receivers without an ADS-B position for 120 s (or with >10%
  of its sync points rejected, or in its periodic forced window) is asked
  from every receiver that sees it (the forced window comes every
  `--serve-force-mlat-interval` seconds, default 600 as mlat-server's;
  production's shards set MLAT_SERVER_FORCE_MLAT_INTERVAL=1200). sync: from
  each receiver's rate report, aircraft scored by the product of the two
  receivers' rates, greedy until every peer has enough shared rate or 12
  positions/s, at least 3. A receiver with a distrusted clock gets 3 sync
  aircraft and no mlat.
  Beyond mlat-server: an aircraft is asked for the moment it becomes wanted,
  not at each receiver's next rate report (~18 s).
- **Results**: every published position goes back to the served receivers
  whose copies made it, in the layout each client asked for ("old" or
  "ecef"), and a `{"stats": ...}` line every 15 s (sync peers, bad-sync
  timeout, outlier share) for clients that asked.
- **Feeder list**: `/clients.json` on the HTTP port is mlat-server's
  clients.json (same keys, types and order) for the served receivers only,
  rewritten every 10 s: name, uuid, declared position, privacy, source
  address and port (from the PROXY line), lines per second, clock pairs,
  bad-sync timeout and outlier share (the stats line's values), and the
  sync / mlat aircraft chosen. `bad_peer_list` holds the peers whose pair
  rejects more than 35% (mlat-server also counts jumps and large offset
  errors, which mlat-ng's pairs do not flag); a name already listed comes
  back as `<name>_<uid>` (mlat-server renames such a newcomer). Shadow
  receivers are not in it: their address is the hub's.
- **Hybrid**: the router already makes a feeder's mlat-client the owner of
  its key, so a served client merged with its own Beast feed is the one that
  is asked and answered.

Measured (2026-10-26):

| | serve, mlat-server rules | every client sends everything |
|---|---|---|
| messages/s per feeder, p50 / p90 / p99 | 6.8 / 19 / 35 | 9.6 / 64 / 142 |
| Mode-S-only aircraft (never ADS-B in the run) | 102 | 115 |
| Mode-S-only positions | 23.6k | 25.1k |

`mlat-ng replay --serve-sim select|all` on the 10-minute cap3 capture: every
receiver of the registry (2,733) is an emulated mlat-client
(`src/clientsim.rs`, a port of mlat-client 0.4.x's coordinator and reader
filter), driven by what its receiver heard. Absolute rates are lower than
production's (the 4906 feed carries only part of each receiver's replies);
the comparison is the point.

End to end with the real mlat-client 0.4.2 (10 clients near Milan replaying
their own frames from the capture, the rest of the network on the hybrid
feed, all on localhost): handshakes accepted, start/stop requests honoured
("12 of 51 Mode S, 29 of 96 ADS-B used"), stats shown by the client
("peer_count: 8 outlier_percent: 0.0 bad_sync_timeout: 0"), 2,757 results
returned and passed on by the clients to their own outputs (178
positions/minute on the busiest), no client errors.
`mlat-ng replay --extract-dir` writes the per-receiver streams for such tests.

Rollout (not done; needs the test's verdict first): per region through
mlat-proxy's region map, `rest` first. That region's feeders go to the
hybrid's `:30101`, and they stay teed to the shadow hub: the standby needs
that copy, and the hybrid router drops it on the instance that serves them
(see "Redundancy" below). readsb gets a `sbs_in_mlat` connector to mlat-ng's
SBS output in place of the region's shard, with `--served-output-only` so
that output carries only positions a served feeder contributed to (what the
region's shard would have published; the other shards keep theirs).
Rollback is one line in the map.

Not yet: Mode A/C requests (`start_sending` with Mode A/C codes), the UDP
transport (production listens TCP-only), and a per-aircraft cap on how many
receivers are asked (mlat-server asks all of them; EU aircraft can be seen by
80).

## Redundancy: a warm standby

Added 2026-11-01, M1, M2 and M5 of the redundancy design (nesso-infra:
mlat-proxy's backend pools; design A). A served region runs on two hybrid
instances, each fed the hub's shadow copy of every feeder and the Beast
feed, so both have the whole network warm. mlat-proxy serves the region
from one of them and moves its feeders to the other when the first one
fails its health check or announces a drain. The README has the `/readyz`
contract, the flags and the metrics.

- **M1, readiness and drain** (`src/ready.rs`): `/readyz` answers 200 only
  when the instance is warm (uptime, a live engine, usable clock pairs, and
  with the feed a delivering cell and a registry). The serve listener is
  bound only then, so a cold instance refuses and the proxy tries the other
  one within the same client connection; after `--serve-gate-max-s` it is
  bound anyway, so two cold instances cannot wait on each other. On SIGTERM
  `/readyz` answers `503 draining`, the serve listener closes, and the
  served feeders are kept until the proxy has moved them (at most
  `--drain-max-s`); then the usual shutdown saves the state.
  The warming criteria gate only the first 200 (`--ready-latch`, on by
  default); after it, only draining and stalled answer 503. The 2026-11-01
  failover harness (30 mlat-client 0.4.2, two hybrids, the hub, two proxies)
  found why: the member that has just taken over answered `warming: usable
  clock pairs 0.37-0.44 < 0.600` for about 4 s, from 4-6 s after the switch,
  because the reconnecting feeders reset their pairs; the proxy marked it
  down in 4 of 15 events. With a usable peer that moves every served feeder
  back, and the next takeover can do the same.
- **M2, one receiver per served feeder** (`src/hybrid.rs`): the served
  connection and the hub's copy of it have the same key. The router tells
  them apart by the listener they came through, gives the key to the served
  one (served > shadow > feed), and drops everything from the copy: the
  served stream has the same lines and more. When the served connection
  goes, the copy takes over and the feeder stays a receiver. That handover
  is still a Disconnect and a Connect; M3 (below) is what keeps the
  receiver's clock pairs across it. The router exists only with `--hybrid`,
  and `--listen` must run with `--shadow` (otherwise its connections look
  served too): an instance without them must not get the copy of a feeder
  it serves.
- **M5, the gap from the server side** (`src/proto/session.rs`): every
  served session's time to its first `start_sending` and first result
  line, as histograms, and the sessions opened and closed. In a handover
  each moved feeder starts a new session on the instance taking over.
- **State**: two instances sharing a volume use their own state file paths
  (`MLAT_NG_COVERAGE_FILE`, `MLAT_NG_SURVEY_FILE`); one writer per path. No
  code change was needed for this.

### M3: a takeover keeps the receiver's clock state

Added 2026-11-01. Before it, the end of a connection was the end of the
receiver: its clock pairs were dropped, and a pair needs 10 observations
before it is used again. A served feeder moved to the standby went through
that twice (the hub copy's leg ends, the feed takes the key, the served
connection takes it back), and its new session was asked for nothing until
its first aircraft report, 5-22 s after connecting.

- **Park, then resume** (`src/engine.rs`, `disconnect` / `resume`). A
  receiver whose connection ends leaves everything a connected receiver
  takes part in (served count, coverage placement, the aircraft counts,
  groups, bridges, `/receivers.json`, `/clients.json`), but keeps its clock
  pairs and its own state for `--reconnect-grace-s` (20 s). A connection
  with the same identity (the UUID key, which the feed receiver shares;
  else the user name), the same clock type and the same place (within 1 m,
  survey correction included) continues it: same engine receiver, same
  pairs, epoch, noise scale, health, wall map; new connection info, nothing
  requested. Anything else with that identity is a new receiver. After the
  grace the parked receiver is dropped exactly as a disconnect dropped it.
  This needs no router change: every handover the router makes (M2's
  serve > shadow > feed, both ways) is a Disconnect followed by a Connect,
  and so is a quick reconnect to the same instance. It replaces the
  design's `Event::Rebind`.
- **Re-anchoring, not blind reuse** (`src/clock.rs`, `reset_offset`). A
  pair is two counters and two crystals; a reconnect does not say whether
  the counter ran on. mlat-client cannot know either, so every new session
  announces its input twice ("input connected", then a synthesised
  "Decoder mode changed" clock reset; coordinator.py server_connected,
  receiver.py start_connection). Each reset used to forget the offset and
  keep the drift: 10 more observations. Now a converged pair is also set
  aside, and the observations after the reset decide: if the first 4 (two
  sync points) all pass its own 4-sigma gate, the counter did not move and
  the whole model comes back, usable at once; the first one that does not
  pass discards it, and the fresh offset goes on as before. A set-aside
  model older than 45 s (`MAX_AGE`) is never restored. A sync observation
  made before its receiver's last reset no longer pairs with one made
  after it (`Rx::epoch`): old observations in the 10 s sync table must not
  confirm a set-aside model, nor seed a fresh one. A second reset before
  the pair re-converges now keeps the drift (it used to be lost). A
  handover between a feeder's inputs (same readsb, same counter, no new
  session) has no reset: the pairs simply go on, still gated as always.
- **What the receiver reported** (`Inherited`). Its aircraft and rate
  report come back with it and count at once, so a resumed served receiver
  is asked for traffic in the same instant (sync from the kept rates, mlat
  for wanted aircraft it sees) instead of at its first report. The new
  session confirms them: an aircraft it does not report within 45 s (two
  of mlat-client's reports) is dropped as lost, and the kept rates last
  until its first real rate report (mlat-client's empty report right after
  the handshake means "reports will follow", jsonclient.py, and no longer
  replaces them).

Expected effect on a served feeder's gap after a takeover (results are
back when its pairs are usable and it is asked for traffic):

| | before | with M3 |
|---|---|---|
| asked for traffic | at its first report, 5-22 s after reconnecting | on reconnecting |
| its pairs usable | 10 fresh observations each, twice on a standby (feed, then served) | 4 observations each, if its counter ran on; 10 if it really restarted |
| simulation (`sim::takeover_results_resume_sooner_and_no_worse`, every receiver reconnecting after 3 s, sparse sync) | first fix 10.3 s after the return | 0.3 s (1.3 s with every counter restarted) |

Accuracy in that simulation does not change: once both runs have positions
again (from 10 s after the return) p50 / p90 are 22 / 37 m before and
22 / 34 m with M3, 21 / 33 m with restarted counters; the whole run 23 /
37 m in both. The early positions only M3 has (the first 10 s) are 26 /
49 m. Three other seeds agree (p50 the same or lower, p90 within 5 m
either way). The replay of real
captures (`mlat-ng replay --reconnect-grace-s 0|20`) has no scripted
reconnects yet; the live measure is M5's `serve_first_result_seconds` on
the instance taking over.

Metrics: `mlat_ng_receivers_parked` (gauge, not in `mlat_ng_receivers`),
`mlat_ng_receiver_parks_total`, `_resumes_total`, `_resumed_pairs_total`,
`_resumes_refused_total` (same identity, another clock or place),
`_parks_expired_total`, and `mlat_ng_pair_reanchors_total`.

## Receiver position calibration (survey)

Added 2026-10-26, diagnostic only (nothing moves a receiver). `src/survey.rs`:
every sync observation is an ADS-B transmission from a known place heard by
two receivers; a receiver declared d metres away from its real antenna sees
every observation shifted by the projection of d on the direction of the
aircraft. Each pair's clock offset and drift is a line inside 60 s windows,
projected out in closed form against the receiver's own clock (as in the
2026-10-25 prototype); what is left is linear least squares in d, kept as
sums (a few dozen additions per observation, a 3x3 solve per question),
forgotten over an hour, windows that fit badly dropped whole. Height has a
tight prior (barely observable), the horizontal a weak one. Keyed by the
receiver's coverage identity (its UUID), so reconnects keep the estimate.

Replay of the 10-minute cap3 capture (`--perturb-share`, `--positions-from`):

| | holdout p50 / p90 | published p99 |
|---|---|---|
| declared positions | 50 / 1132 m | 989 m |
| 964 reliable estimates applied | 42 / 862 m | 970 m |
| 10% of receivers declared 300 m off | 60 / 1580 m | 1311 m |
| ... then their estimates applied | 45 / 1010 m | 1138 m |

The 300 m moves came back to 23 m p50 / 64 m p90 for estimates with >= 200
windows and a weakest horizontal sigma < 5 m ("reliable"; 81 of 87 within
100 m). Estimated and scored on the same capture, so optimistic; live
estimates would be applied out of sample. 77 of 877 reliable unmoved
receivers come out > 100 m from their declared position (rounded
coordinates?).

Live: `/survey.json` (cluster-internal: identities and corrections, never an
absolute position) and the gauges `survey_receivers`, `survey_reliable`,
`survey_reliable_over_100m`. Not yet: applying corrections in the engine
(it means re-converging the receiver's clock pairs), partners' own errors
(modelled as noise today), placing Beast-only receivers with no declared
position (needs the prototype's grid search, not a linearisation).

## Open items for v0.2

- Apply survey corrections live (see above), once its live numbers are in.
- Mode A/C: the feed carries none today (only `0000` heartbeats); it needs
  readsb to forward Mode A/C on reduced output when asked (`1J`) and
  `--modeac-auto` on feeders.
