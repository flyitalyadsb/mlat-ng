# Version-pinning policy (WS1 — platform factory)

Org rule for consuming the shared Nesso "bricks" (`nesso-notify`, `nesso-fetch`,
`mcp-common`, `nesso-scraping`, `nesso-ocr`, `nesso-k8s-template`, ...). See
`../docs/WS1-PACKAGE-AUDIT.md` and `../docs/PLATFORM-FACTORY.md` in
`nesso-infra` for the full rationale; this file states the rule new repos
must actually follow, plus the `renovate.json` that enforces it.

## Packaging mechanism: git-tag `uv`/`pip` installs

No private PyPI index yet (deliberately - see the audit's §7). Every brick is
consumed straight from its GitHub repo at an exact tag:

```toml
# pyproject.toml
dependencies = [
  "nesso-notify @ git+https://github.com/nesso-labs/nesso-notify@v0.1.0",
]
```

```
# requirements.txt
nesso-fetch @ git+https://github.com/nesso-labs/nesso-fetch@v0.2.1
```

`uv`/`pip` git dependencies don't support version *ranges* the way PyPI
packages do — you pin one exact ref. That ref **must** be a tag
(`vX.Y.Z`), never a branch name (`main`) or a raw commit SHA:

- a branch floats — the exact thing WS1 is trying to kill;
- a raw SHA works but throws away the semver signal Renovate needs to know
  whether a bump is patch/minor/major.

## The "compatible-release range" is enforced by Renovate config, not by the pin itself

Because a git-tag dependency is a single pinned ref, not a `>=x.y,<x.(y+1)`
range, the org's compatible-release policy is implemented one layer up, in
`renovate.json`:

- Renovate opens a PR for **every** new tag on a tracked brick.
- **Patch/minor** bumps auto-merge once CI passes (this is the
  `>=x.y,<x.(y+1)` ceiling — Renovate just keeps re-pinning inside it
  automatically instead of a range in the file doing it).
- **Major** bumps never auto-merge — they need a human, because a major tag
  is the brick's own signal that something breaking changed.

If a real private PyPI index shows up later (audit §7 — only once there are
>6-8 interdependent packages), classic dependency ranges
(`nesso-notify>=0.1,<0.2`) become possible and should replace the git-tag
pin; the Renovate `packageRules` in `renovate.json` don't change, only the
`customManagers` matching git-tag URLs stop being needed for that package.

## Kustomize remote bases follow the same rule

`k8s/<env>/kustomization.yaml` files that reference a shared base
(`nesso-k8s-template`, or historically `new-repo-template`) must pin
`?ref=vX.Y.Z`, never `?ref=main`:

```yaml
resources:
  - github.com/nesso-labs/nesso-k8s-template//nesso-k8s-bases/archetypes/http-app/base?ref=v1.1.0
```

`scripts/check_k8s_bases_sync.py` (wired into
`.github/workflows/check-k8s-bases-sync.yml`) enforces this **inside this
repo's own forwarding shim** at `nesso-k8s-bases/`. It does not (and cannot)
reach into other repos' `k8s/*/kustomization.yaml` files — each consuming
repo should copy the same check (or a version of it) if it wants CI to
enforce the same rule locally. Today, ~30 production repos still pin
`new-repo-template//nesso-k8s-bases/...?ref=main` (floating, and going
through the retired duplicate) - migrating those, at each repo's own pace,
to `nesso-k8s-template//nesso-k8s-bases/...?ref=vX.Y.Z` is the intended
end-state, tracked by the `renovate.json` `customManagers` entry for
`kustomization.yaml` in this file's own repo, and reusable by any repo that
adopts this template's `renovate.json`.

## `renovate.json` in this template

Ships two `customManagers` (since Renovate has no built-in datasource for
"git-tag URL embedded in a TOML/YAML string"):

1. Matches `git+https://github.com/nesso-labs/<repo>@vX.Y.Z` inside
   `pyproject.toml` / `requirements*.txt`.
2. Matches `github.com/nesso-labs/<repo>//...?ref=vX.Y.Z` inside
   `kustomization.yaml`.

Both resolve new versions against the `github-tags` datasource, so Renovate
opens a PR the moment the upstream brick cuts a new tag. `packageRules`
group all platform-library PRs together and auto-merge patch/minor.

Copy `renovate.json` as-is into new repos scaffolded from this template;
only the `matchPackageNames` allow-list needs extending if a new brick is
added to the platform.
