# mlat-ng

Next-generation Mode S / Mode A/C multilateration server for the
flyitalyadsb network. Rust, one process for the whole network, wire-compatible
with every mlat-client from 0.2.13 to 0.4.2.

Status: **v0.1, shadow test** — it receives a copy of every feeder's stream
(through mlat-proxy's tee and mlat-shadow-hub in nesso-infra) and is compared
for 7 days against the production mlat-server and against flightportrait/mlatd
by `mlat-judge`. Its results reach nobody but the judge. See
[docs/PLAN.md](docs/PLAN.md) for the design, the test and the decision rule.

## Build, test, run

```sh
cargo test                 # unit tests + an end-to-end simulation (src/sim.rs)
cargo build --release
./target/release/mlat-ng --shadow --listen 0.0.0.0:30100 --sbs-listen 0.0.0.0:31003 --http-listen 0.0.0.0:9100
```

Every flag also reads an environment variable (`MLAT_NG_*`, see `--help`).

| Port | What |
|---|---|
| 30100 | mlat-client connections (PROXY v1 line accepted) |
| 31003 | BaseStation (SBS) MSG,3 results, mlat-server's layout; Mode A/C tracks as `~F0xxxx` |
| 9100 | `/metrics` (Prometheus), `/state.json`, `/healthz`, `/receivers.json` (receiver positions by UUID; cluster-internal), `/clients.json` (served feeders in mlat-server's clients.json layout, source addresses included; cluster-internal) |
| 31004 | bench SBS output, feed mode only (`--bench-sbs-listen`) |

### Feed mode

`--feed a=readsb-ingest-a:4906,b=readsb-ingest-b:4906` adds a second input:
readsb-ingest's receiver-tagged Beast output, multilaterated with no
mlat-client involved (receiver positions from `--feed-receivers-url`, the
primary instance's `/receivers.json`). With `--bench-sbs-listen`, positions of
aircraft that also broadcast ADS-B go to a separate port, so they can be
scored against their own ADS-B. See [docs/PLAN.md](docs/PLAN.md#feed-input).

### Serving with a warm standby

With `--serve-listen` ([serve mode](docs/PLAN.md#serve-mode)) a region can
run on two hybrid instances: mlat-proxy serves it from one and moves its
feeders to the other when that one fails or restarts. Both get the hub's
shadow copy of every feeder, so the standby stays warm. See
[docs/PLAN.md](docs/PLAN.md#redundancy-a-warm-standby).

`GET /readyz` on the HTTP port is what the proxy and the readiness probe
poll. The proxy reads only the status, and whether the body starts with
`draining`:

| Answer | When |
|---|---|
| `200 ok` | warm: up at least `--ready-min-uptime-s`, an engine snapshot younger than 10 s, at least `--ready-min-usable` of the clock pairs usable, and with `--feed` a cell that delivered frames in the last 30 s and a non-empty registry. After the first 200, everything but draining and stalled (the latch) |
| `503 draining` | after SIGTERM: the proxy leaves this instance at once |
| `503 stalled: <why>` | the engine's snapshot is more than 10 s old |
| `503 warming: <why>` | before the first 200, any other criterion not met |

The warming criteria gate only the first 200, which also opens the serve
listener: a takeover's reconnect wave resets the clock pairs of the feeders
moving here, and their usable share dips for a few seconds. Re-checking it
would make the proxy mark the member that just took over down, and move
every served feeder back. The first 200 logs `ready: warm for the first
time`. `/healthz` stays a constant `ok` (liveness).

| Flag / env | Default | |
|---|---|---|
| `--ready-min-uptime-s` / `MLAT_NG_READY_MIN_UPTIME_S` | 120 | /readyz is 200 only after this many seconds |
| `--ready-min-usable` / `MLAT_NG_READY_MIN_USABLE` | 0.6 | share of clock pairs that must be usable (0 = no check) |
| `--ready-latch` / `MLAT_NG_READY_LATCH` | true | after the first 200, only draining and stalled answer 503; `false` re-checks the warming criteria every time |
| `--serve-gate-max-s` / `MLAT_NG_SERVE_GATE_MAX_S` | 240 | `--serve-listen` is bound once /readyz would be 200, or after this many seconds anyway (0 = at start) |
| `--drain-max-s` / `MLAT_NG_DRAIN_MAX_S` | 30 | after SIGTERM, how long the served feeders are kept while the proxy moves them |
| `--reconnect-grace-s` / `MLAT_NG_RECONNECT_GRACE_S` | 20 | a receiver whose connection ends keeps its clock pairs this long for the same receiver to come back (a takeover, a handover between its inputs, a quick reconnect); 0 = dropped at once |

On SIGTERM: `/readyz` answers `503 draining`, the serve listener closes (new
connects are refused), and the served feeders keep being served until none
is left or `--drain-max-s` passes. Then the process stops as before and
saves its state. Two instances on one volume need their own
`MLAT_NG_COVERAGE_FILE` and `MLAT_NG_SURVEY_FILE` paths.

A served feeder also arrives as the hub's shadow copy. With `--hybrid`, and
`--shadow` on `--listen` (that is how the copy is told from the served
connection), the router counts the two as one receiver and drops the copy.
Without `--hybrid` there is no router: do not tee a served feeder to the
instance that serves it.

When a feeder moves (to the standby, between its inputs, or back after a
quick reconnect) the engine keeps its receiver: its clock pairs are set
aside at the new session's clock reset and come back after two sync points
if its counter ran on, and it is asked for traffic at once from what it
reported before ([M3](docs/PLAN.md#m3-a-takeover-keeps-the-receivers-clock-state)).

| Metric | |
|---|---|
| `mlat_ng_serve_first_start_seconds`, `mlat_ng_serve_first_result_seconds` | histograms: seconds from a served session's start to its first `start_sending` / result line |
| `mlat_ng_serve_sessions_opened_total`, `_closed_total`, `_closed_no_result_total`; `mlat_ng_serve_sessions_active` | served connections |
| `mlat_ng_hybrid_owner_serve`, `mlat_ng_hybrid_served_with_shadow` | keys owned by a served connection, and of those the ones whose shadow copy is connected too |
| `mlat_ng_hybrid_shadow_of_served_dropped_total` | events from the shadow copy of a served feeder, dropped |
| `mlat_ng_receivers_parked`; `mlat_ng_receiver_parks_total`, `_resumes_total`, `_resumed_pairs_total`, `_resumes_refused_total`, `_parks_expired_total` | receivers waiting for their next connection, and what became of them (M3) |
| `mlat_ng_pair_reanchors_total` | clock pairs set aside by a clock reset and restored because the counter had not moved |
| `mlat_ng_ready_latched`; `mlat_ng_ready_latch_held_total` | 1 once /readyz has answered 200 (with the latch on); verdicts the latch kept at 200 while a warming criterion was not met |

Each served session also logs `served: first result` with `first_start_ms`
and `first_result_ms`.

### Replay

```sh
mlat-ng replay \
  --capture capA.bin.gz@<start unix>@60 --capture capB.bin.gz@<start unix>@60 \
  --receivers registry.json --out summary.json --dump-sbs modes-only.sbs
```

Runs the real feed adapter and engine on captures of the 4906 stream
(`exec 3<>/dev/tcp/127.0.0.1/4906; timeout 60 cat <&3 | gzip -1` on each
ingest pod) on a simulated clock, and prints clock-sync figures, the bench
(error of ADS-B aircraft multilaterated from their non-ADS-B replies) and the
Mode-S-only output. Captures and registries hold feeder identities and
positions: keep them out of the repository.

## Layout

| File | |
|---|---|
| `src/proto/` | wire protocol: PROXY, handshake, none/zlib/zlib2 framing, message parsing (network threads) |
| `src/modes/` | Mode S / ADS-B / Mode A/C decoding |
| `src/clock.rs` | per-pair clock Kalman filter |
| `src/norm.rs` | joint least-squares timestamp normalisation, transmission clustering |
| `src/solver.rs` | position solve with full time covariance |
| `src/track.rs` | EKF tracks with fix and direct TDOA updates |
| `src/engine.rs` | the single owner of all state: sync, holdout, groups, tracks, Mode A/C |
| `src/output.rs` | SBS server, HTTP server |
| `src/ready.rs` | `/readyz`, the gated serve listener, the drain on SIGTERM |
| `src/relay/` | feed input: Beast parser, receiver registry, frames-to-events adapter, cell readers |
| `src/replay.rs` | offline replay of feed captures |
| `src/sim.rs` | end-to-end simulation test |

## Deploy

Image: `ghcr.io/nesso-labs/mlat-ng:sha-<commit>` (built by `.github/workflows/build.yml`
on every push to main). Kubernetes manifests live in nesso-infra:
`k8s/flyitalyadsb/base/mlat-contenders.yaml`; the production tag is pinned in
`k8s/flyitalyadsb/production/kustomization.yaml`.

## License

AGPL-3.0-or-later.
