"""Compare or refresh the explicit local source allowlist. Never pushes or uses hardware."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from verify_baseline import ROOT, BASELINE, digest, verify

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--write", action="store_true", help="replace the selected research snapshot")
    args = parser.parse_args()
    source = args.source.resolve(strict=True)
    if source == BASELINE.resolve() or BASELINE.resolve() in source.parents or source in BASELINE.resolve().parents:
        parser.error("source must be separate from the research snapshot")
    selection = json.loads((ROOT / "source_selection.json").read_text(encoding="utf-8"))
    paths = selection["paths"]
    if paths != sorted(set(paths)):
        parser.error("source selection must be unique and sorted")
    buffers = {}
    rows = []
    for name in paths:
        rel = Path(name)
        if rel.is_absolute() or ".." in rel.parts:
            parser.error("selection contains an invalid path")
        original = (source / rel).resolve(strict=True)
        if source not in original.parents:
            parser.error("selection escapes the source directory")
        data = original.read_bytes()
        buffers[name] = data
        rows.append({"path": name, "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()})
    # Detect concurrent source edits before writing anything to the research copy.
    for name, data in buffers.items():
        if (source / name).read_bytes() != data:
            raise SystemExit("Source changed while reading; retry from a stable version")
    changed = [name for name, data in buffers.items()
               if not (BASELINE / name).is_file() or (BASELINE / name).read_bytes() != data]
    print(f"{len(changed)} changed files; snapshot {digest(rows)}")
    for name in changed:
        print(name)
    if not args.write:
        print("Comparison only. Use --write to update the local research snapshot.")
        return
    existing = {p.relative_to(BASELINE).as_posix() for p in BASELINE.rglob("*")
                if p.is_file() and not set(p.relative_to(BASELINE).parts) & {"output", "__pycache__"}}
    if existing - set(paths):
        raise SystemExit("Unlisted files exist; review them before refreshing the snapshot")
    old_manifest = ROOT / "baseline_manifest.json"
    if not changed and old_manifest.is_file():
        verify()
        return
    for name, data in buffers.items():
        target = BASELINE / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    manifest = {
        "schema_version": 1,
        "source_label": "AMR_PCB _RGB",
        "source_revision_kind": "local-directory-content-digest",
        "source_git_commit": None,
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "snapshot_sha256": digest(rows),
        "files": rows,
        "deployment_verified": False,
        "note": "Selected source bytes, not a flashed-image or hardware-validation identity."
    }
    old_manifest.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    # Previous validation cannot describe refreshed bytes.
    validation = ROOT / "VALIDATION.json"
    if validation.exists():
        validation.write_text(json.dumps({"status": "not_run", "snapshot_sha256": digest(rows)}, indent=2) + "\n", encoding="utf-8")
    verify()

if __name__ == "__main__":
    main()
