# mlat-ng

**The multilateration (MLAT) server of [FlyItalyADSB](https://flyitalyadsb.com/en/?utm_source=github&utm_campaign=mlat-ng).**
It is written in Rust and runs as one process for the whole network. It is
wire-compatible with every mlat-client from 0.2.13 to 0.4.2, so feeders change
nothing.

FlyItalyADSB is a global community network, born in Italy in 2020, with about
3,200 receivers: they send us their data directly, ADS-B and our own
multilateration (MLAT), open and unfiltered. A free live map, a REST API and an
MCP server for AI assistants.

## Status

One process serves every MLAT feeder of the network, on about 2.4 CPU cores.

A second instance runs on another server and receives the same traffic. When
the first one fails or restarts, the second takes over.

## What feeders and users get

Measured from 26 to 31 October 2026, when mlat-ng ran beside mlat-server on a
copy of the same feeder traffic and both were scored against the aircraft's
own ADS-B positions ([docs/PLAN.md](docs/PLAN.md)). The figures are for the
build that now serves the network. Since mlat-ng chooses the traffic itself,
the two can no longer be compared side by side.

- **More precise positions.** On the same aircraft at the same moment,
  mlat-ng's position was closer to the ADS-B truth in 66% of cases. Its median
  error was 177 m, against 273 m for mlat-server.
- **More aircraft without ADS-B.** mlat-ng located 9.1% more aircraft-minutes
  of aircraft that send no ADS-B position, such as military and older
  aircraft.
- **Fewer impossible jumps.** 0.53% of its positions were implausible, against
  0.85%.
- **Aircraft heard by only two receivers.** mlat-server needs three receivers
  and an altitude, or four. mlat-ng also places aircraft heard by two, from
  their motion and the receivers' coverage. Their median error was about
  110 m (measured on 2 November 2026).
- **No real aircraft hidden.** mlat-server dropped every address up to
  0xF423F to hide fake Mode A/C tracks. With them went real aircraft whose
  addresses are low, such as those from Africa, Qatar and Mexico. mlat-ng drops
  only the exact fake pattern.
- **One network, no borders.** One process sees every pair of receivers. With
  five regional shards, a receiver near the edge of its shard could not pair
  with its neighbours across the edge.
- **Restarts and failures without a gap.** A warm standby takes over. In a
  failover test on 3 November 2026, the 478 connected feeders moved over in a median of
  5.9 s (95th percentile 7.0 s). With mlat-server, a restart made every feeder
  of the region reconnect, which took up to about a minute.
- **Antenna positions corrected.** The engine measures each receiver's
  position from the clock-sync residuals. It corrects the ones that are
  reliably off, so a wrong antenna position no longer spoils its neighbours'
  results.
- **Lightcone for every feeder.** The same engine learns each receiver's
  coverage. Lightcone uses it to show the aircraft that flew past a receiver
  that the receiver did not receive.

## How it works, compared

| | mlat-server (as we ran it) | mlat-ng |
|---|---|---|
| Process | one thread per process; the network split into five regional shards | one process for the whole network: network I/O on every core, one engine thread owning the state, aircraft workers on their own threads ([docs/MULTICORE.md](docs/MULTICORE.md)) |
| Clock pairs | windowed fit and PI drift loop | a Kalman filter per pair (offset and drift), whose variance grows with age |
| Timestamp normalisation | minimum spanning tree | joint weighted least squares over every usable pair |
| Solver | diagonal errors, numeric Jacobian | full time covariance, analytic Jacobian, multi-start with a mirror-solution check |
| Tracking | filter fed with fixes only | EKF fed with fixes and directly with time differences, so groups of 2–3 receivers keep a track alive |
| Inputs | mlat-client | mlat-client, plus the same receiver's Beast feed, merged into one receiver |
| Self-check | none | 5% of ADS-B sync points are held out and multilaterated against their own ADS-B position: a continuous accuracy figure |
| Failover | none | a warm standby: `/readyz`, a drain on SIGTERM, and receivers that keep their clock pairs across a takeover |

## Feed the network

With a Raspberry Pi, an RTL-SDR dongle and an antenna (about €55–110) you send
your receiver's data to the network. In return you get the API with Pro limits
for free, and you see what your receiver can't see on its own.

Your mlat-client reaches this server in either of two ways:
- the [install script](https://github.com/flyitalyadsb/fly-italy-adsb);
- ultrafeeder or adsb.im, which list FlyItalyADSB.

See [how to feed](https://flyitalyadsb.com/en/community/feeder/?utm_source=github&utm_campaign=mlat-ng).
[Lightcone](https://flyitalyadsb.com/en/community/lightcone/?utm_source=github&utm_campaign=mlat-ng) shows each
receiver the aircraft that flew past it and it did not receive. Lightcone needs MLAT.

## Using the positions

MLAT positions reach the [live map](https://map.flyitalyadsb.com/?utm_source=github&utm_campaign=mlat-ng) and the
[API](https://github.com/flyitalyadsb/adsb-api) next to ADS-B, each with its
source marked. Commercial use is allowed on every plan, Free included.

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
| 9100 | `/metrics` (Prometheus), `/state.json`, `/healthz`, `/receivers.json` (receiver positions by UUID; keep this port private), `/clients.json` (served feeders in mlat-server's clients.json layout, source addresses included; keep this port private) |
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

## Container image

`ghcr.io/flyitalyadsb/mlat-ng:sha-<commit>` and `:main`, built by
[.github/workflows/build.yml](.github/workflows/build.yml) on every push to main.

## Credits

mlat-ng talks to [mlat-client](https://github.com/wiedehopf/mlat-client) and
keeps the behaviour feeders know from
[mlat-server](https://github.com/wiedehopf/mlat-server). Both were written by
Oliver Jowett ([mutability](https://github.com/mutability)) and are maintained
by wiedehopf. From mlat-server, mlat-ng ports the traffic selection and the
duplicate-receiver rules, and it keeps the `clients.json` layout. Thank you
to both.

## License

[AGPL-3.0-or-later](LICENSE). If you run a modified version for others, offer
them its source. mlat-ng does this in the message of the day it sends to every
mlat-client (`--source-url`, `MLAT_NG_SOURCE_URL`).

<details>
<summary>In italiano</summary>

mlat-ng è il server di multilaterazione (MLAT) di FlyItalyADSB. È scritto in
Rust e gira come un solo processo per tutta la rete. È compatibile con ogni
mlat-client dalla 0.2.13 alla 0.4.2, quindi i feeder non cambiano niente.

FlyItalyADSB è una rete community globale, nata in Italia nel 2020, con circa
3.200 ricevitori: ci mandano i dati direttamente, ADS-B e multilaterazione
nostra, aperti e non filtrati.

Un solo processo serve tutti i feeder MLAT della rete con circa 2,4 core. Una seconda istanza gira su un
altro server e riceve lo stesso traffico: se la prima si ferma o si riavvia,
prende il suo posto.

Rispetto a mlat-server, misurato dal 26 al 31 ottobre 2026 sullo stesso
traffico e confrontato con le posizioni ADS-B degli aerei:
- **posizioni più precise**: sullo stesso aereo nello stesso istante, quella di
  mlat-ng era la più vicina al vero nel 66% dei casi; l'errore mediano era di
  177 m, contro i 273 m di mlat-server;
- **più aerei senza ADS-B**: il 9,1% di aerei-minuto in più, come i militari e
  gli aerei più vecchi;
- **meno salti impossibili**: 0,53% delle posizioni contro 0,85%;
- **aerei ricevuti da due soli ricevitori**: mlat-ng li posiziona, mlat-server
  no;
- **nessun aereo vero nascosto**: niente filtro sugli indirizzi bassi, per
  esempio di Africa, Qatar e Messico;
- **riavvii e guasti senza buchi**: una riserva calda prende il posto
  dell'istanza attiva, e i feeder si spostano in circa 6 secondi;
- **posizioni delle antenne corrette** dai dati della rete, e la copertura
  per Lightcone.

Con un Raspberry Pi, un dongle RTL-SDR e un'antenna (55–110 € circa) mandi alla
rete i dati del tuo ricevitore. In cambio hai l'API con i limiti Pro gratis e
vedi quello che il tuo ricevitore da solo non può vedere.
[Diventa feeder](https://flyitalyadsb.com/community/feeder/?utm_source=github&utm_campaign=mlat-ng).

</details>
