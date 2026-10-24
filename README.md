# Repo Template

Template di partenza per i nuovi repo nesso-labs:

- `.github/workflows/common-workflows.yml` — workflow riusabili org-wide (auto-README, build & deploy staging/prod)
- `.github/workflows/check-k8s-bases-sync.yml` — CI guard: fallisce se `nesso-k8s-bases/` torna a essere una copia vendorizzata invece di puntare al repo canonico
- `nesso-k8s-bases/` — **forwarding shim** verso [`nesso-k8s-template`](https://github.com/nesso-labs/nesso-k8s-template) (fonte canonica delle basi kustomize, es. `http-app`). Vedi [`nesso-k8s-bases/README.md`](nesso-k8s-bases/README.md) — non aggiungere qui copie locali degli archetipi.
- `renovate.json` + [`docs/PINNING.md`](docs/PINNING.md) — policy org di version-pinning per le librerie di piattaforma (git-tag `uv`/`pip` installs pinnati `@vX.Y.Z`, basi kustomize pinnate `?ref=vX.Y.Z`) e bump automatico via Renovate.
- `starters/python/observability/` — **alerting errori → Slack** drop-in (solo stdlib, integrazioni opzionali loguru/Celery): ogni record ERROR+ posta un alert env-tagged e throttlato. Vedi il [README dello starter](starters/python/observability/README.md).
