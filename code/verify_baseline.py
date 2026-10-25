"""Verify the byte-exact AMR PCB baseline; uses only the Python standard library."""
from pathlib import Path
import hashlib
import json
import sys

ROOT = Path(__file__).resolve().parent
BASELINE = ROOT / "baseline" / "amr_pcb"

def digest(files):
    payload = "".join(f"{row['sha256']}  {row['path']}\n" for row in files)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()

def verify():
    manifest = json.loads((ROOT / "baseline_manifest.json").read_text(encoding="utf-8"))
    rows = manifest["files"]
    errors = []
    if rows != sorted(rows, key=lambda row: row["path"]):
        errors.append("manifest paths are not sorted")
    expected = {row["path"] for row in rows}
    if len(expected) != len(rows):
        errors.append("manifest contains duplicate paths")
    for row in rows:
        relative = Path(row["path"])
        if relative.is_absolute() or ".." in relative.parts:
            errors.append("invalid manifest path")
            continue
        path = BASELINE / relative
        if not path.is_file():
            errors.append(f"missing: {row['path']}")
            continue
        data = path.read_bytes()
        if len(data) != row["bytes"] or hashlib.sha256(data).hexdigest() != row["sha256"]:
            errors.append(f"changed: {row['path']}")
    actual = {p.relative_to(BASELINE).as_posix() for p in BASELINE.rglob("*")
              if p.is_file() and not set(p.relative_to(BASELINE).parts) & {"output", "__pycache__"}}
    errors.extend(f"unlisted: {path}" for path in sorted(actual - expected))
    selection = json.loads((ROOT / "source_selection.json").read_text(encoding="utf-8"))
    if set(selection["paths"]) != expected:
        errors.append("selection and manifest paths differ")
    if digest(rows) != manifest["snapshot_sha256"]:
        errors.append("snapshot digest mismatch")
    if errors:
        raise SystemExit("BASELINE CHECK FAILED\n" + "\n".join(errors))
    print(f"PASS: {len(rows)} source files; snapshot {manifest['snapshot_sha256']}")
    return manifest

if __name__ == "__main__":
    verify()
