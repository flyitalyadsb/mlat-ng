#!/usr/bin/env python3
"""Fail CI if nesso-k8s-bases/ drifts back into a vendored copy.

WS1 ("kill copy-from drift") context: this repo used to ship its own copy of
the Kustomize archetypes under nesso-k8s-bases/, which silently diverged from
the canonical `nesso-k8s-template` repo (different ingress class, a stray
nodeSelector, a missing Reloader annotation - three real behavioral
differences with no CI to catch them). The fix: nesso-k8s-bases/ here is now
a *forwarding shim* - a kustomization.yaml whose only job is to point at
nesso-k8s-template - so there's nothing left to vendor, and therefore nothing
left to drift.

This script is the guard that keeps it that way. It fails if:

1. Any file other than `kustomization.yaml` exists under
   `nesso-k8s-bases/archetypes/**` (i.e. someone re-added deployment.yaml /
   service.yaml / ingress.yaml or any other manifest instead of forwarding).
2. A `kustomization.yaml` under `nesso-k8s-bases/archetypes/**` has no
   `resources:` entries, or an entry that doesn't reference the canonical
   `github.com/nesso-labs/nesso-k8s-template` repo.
3. Any such resource entry is pinned to a floating ref (`main`, `master`,
   `HEAD`, or missing `?ref=` entirely) instead of an explicit `vX.Y.Z` tag -
   enforcing the org's pinning policy (docs/PINNING.md) at the source.

Stdlib only, no PyYAML dependency - this only ever needs to parse the narrow,
known shape of a forwarding-shim kustomization.yaml, not arbitrary YAML.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

CANONICAL_REPO = "github.com/nesso-labs/nesso-k8s-template"
ALLOWED_FILENAME = "kustomization.yaml"
SEMVER_TAG_RE = re.compile(r"^v\d+\.\d+\.\d+$")
FLOATING_REFS = {"main", "master", "head", "trunk"}

# Matches a kustomize remote-resource "resources:" list item, e.g.
#   - github.com/org/repo//path/to/base?ref=v1.2.3
RESOURCE_LINE_RE = re.compile(r"^-\s+(\S+)\s*$")


def find_archetype_root(repo_root: Path) -> Path | None:
    root = repo_root / "nesso-k8s-bases" / "archetypes"
    return root if root.is_dir() else None


def collect_kustomization_files(archetypes_root: Path) -> list[Path]:
    return sorted(archetypes_root.rglob(ALLOWED_FILENAME))


def collect_stray_files(archetypes_root: Path) -> list[Path]:
    return sorted(
        p
        for p in archetypes_root.rglob("*")
        if p.is_file() and p.name != ALLOWED_FILENAME
    )


def parse_resource_lines(kustomization_path: Path) -> list[str]:
    """Extract the string items of the `resources:` YAML list, line-scanned.

    Deliberately not a general YAML parser: a forwarding shim's
    kustomization.yaml is expected to have exactly one top-level
    `resources:` key followed by a flat list of scalar strings. Anything
    else falls outside what this guard is meant to validate.
    """
    lines = kustomization_path.read_text().splitlines()
    resources: list[str] = []
    in_resources = False
    for raw_line in lines:
        stripped = raw_line.rstrip()
        if not stripped or stripped.lstrip().startswith("#"):
            continue
        if re.match(r"^resources:\s*$", stripped):
            in_resources = True
            continue
        if in_resources:
            if re.match(r"^\S", stripped):
                # dedented back to a new top-level key -> resources block ended
                break
            m = RESOURCE_LINE_RE.match(stripped.strip())
            if m:
                resources.append(m.group(1))
            else:
                # a "- foo:" mapping-style item or something we don't expect
                in_resources = False
    return resources


def check_resource_ref(resource: str) -> list[str]:
    errors = []
    if CANONICAL_REPO not in resource:
        errors.append(
            f"resource {resource!r} does not reference canonical repo "
            f"({CANONICAL_REPO}) - nesso-k8s-bases/ must forward to it, not "
            "vendor a copy or point elsewhere"
        )
        return errors  # ref check below is meaningless if repo is wrong

    if "?ref=" not in resource:
        errors.append(
            f"resource {resource!r} has no '?ref=' - pin an explicit vX.Y.Z "
            "tag (docs/PINNING.md), don't float on the default branch"
        )
        return errors

    ref = resource.rsplit("?ref=", 1)[1]
    if ref.lower() in FLOATING_REFS:
        errors.append(
            f"resource {resource!r} is pinned to a floating ref ({ref!r}) - "
            "pin an explicit vX.Y.Z tag instead (docs/PINNING.md)"
        )
    elif not SEMVER_TAG_RE.match(ref):
        errors.append(
            f"resource {resource!r} ref ({ref!r}) doesn't look like a semver "
            "tag (vX.Y.Z) - pin an explicit released tag (docs/PINNING.md)"
        )
    return errors


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    archetypes_root = find_archetype_root(repo_root)

    if archetypes_root is None:
        print("No nesso-k8s-bases/archetypes/ directory found - nothing to check.")
        return 0

    errors: list[str] = []

    stray_files = collect_stray_files(archetypes_root)
    for f in stray_files:
        errors.append(
            f"unexpected vendored file: {f.relative_to(repo_root)} - "
            "nesso-k8s-bases/ must only contain forwarding kustomization.yaml "
            "files, see nesso-k8s-bases/README.md"
        )

    kustomization_files = collect_kustomization_files(archetypes_root)
    if not kustomization_files:
        errors.append(
            f"no {ALLOWED_FILENAME} found anywhere under "
            f"{archetypes_root.relative_to(repo_root)}"
        )

    for kfile in kustomization_files:
        resources = parse_resource_lines(kfile)
        rel = kfile.relative_to(repo_root)
        if not resources:
            errors.append(f"{rel}: no 'resources:' entries found")
            continue
        for resource in resources:
            for err in check_resource_ref(resource):
                errors.append(f"{rel}: {err}")

    if errors:
        print("nesso-k8s-bases sync check FAILED:\n")
        for err in errors:
            print(f"  - {err}")
        print(
            "\nSee nesso-k8s-bases/README.md and docs/PINNING.md for the "
            "expected shape."
        )
        return 1

    print(f"nesso-k8s-bases sync check OK ({len(kustomization_files)} shim(s) verified).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
