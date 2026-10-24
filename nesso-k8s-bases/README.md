# nesso-k8s-bases (this copy is a forwarding shim)

**Canonical source: [`nesso-k8s-template`](https://github.com/nesso-labs/nesso-k8s-template).**
This directory used to carry its own copy of the Kustomize archetypes
(`deployment.yaml`, `service.yaml`, `ingress.yaml`) and drifted out of sync
with the standalone repo with no automated check — three real behavioral
differences (ingress class, a node selector, a Reloader annotation) existed
here and not there. That's the drift WS1 ("kill copy-from drift") closes.

## What lives here now

Only `archetypes/http-app/base/kustomization.yaml`, and it contains nothing
but a forwarding reference:

```yaml
resources:
  - github.com/nesso-labs/nesso-k8s-template//nesso-k8s-bases/archetypes/http-app/base?ref=v1.1.0
```

This exists purely for backward compatibility: roughly 30 production repos
already build against
`github.com/nesso-labs/new-repo-template//nesso-k8s-bases/archetypes/http-app/base?ref=main`
in their `k8s/*/kustomization.yaml`. Kustomize resolves remote bases
recursively, so those repos keep working unmodified through this shim - see
the divergence check below for what stops it from silently rotting again.

## If you're scaffolding a NEW repo

Don't copy this directory and don't reference this shim. Point your
`k8s/<env>/kustomization.yaml` straight at the canonical repo with an
explicit pinned tag, e.g.:

```yaml
resources:
  - github.com/nesso-labs/nesso-k8s-template//nesso-k8s-bases/archetypes/http-app/base?ref=v1.1.0
```

See [`docs/PINNING.md`](../docs/PINNING.md) for the org's pinning/Renovate
policy - never pin `?ref=main` or any branch name in a real service.

## Divergence / drift guard

`scripts/check_k8s_bases_sync.py` (wired into
`.github/workflows/check-k8s-bases-sync.yml`) fails CI if:

- any file other than `kustomization.yaml` reappears under
  `nesso-k8s-bases/archetypes/**` (i.e. someone re-vendors a local copy), or
- the shim's `resources:` entry stops pointing at
  `github.com/nesso-labs/nesso-k8s-template`, or
- the shim's `?ref=` is a floating ref (`main`, `master`, or no ref at all)
  instead of a pinned `vX.Y.Z` tag.

## Migrating an existing consumer off the shim

Whenever convenient (no urgency - the shim works), change your
`k8s/*/kustomization.yaml` `resources:` entry from
`new-repo-template//nesso-k8s-bases/...?ref=main` to
`nesso-k8s-template//nesso-k8s-bases/...?ref=vX.Y.Z` directly, and let
Renovate keep the tag current from then on.
