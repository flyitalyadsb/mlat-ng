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
