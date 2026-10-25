#!/usr/bin/env python3
"""Read-only, firmware-build-bound RPDO diagnostics; wheel identity is optional."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import time
from amr_protocol import SerialSession, collect_numbered_snapshot


VARYING_TARGET_SECONDS = 8.0


def varying_low_speed_target(elapsed_s: float) -> str:
    """Fixed lifted-wheel profile: 0..80 mm/s, no yaw, holds and normal zero.

    Pure command generation; caller owns identity, qualification and cleanup.
    Duration/speed are deliberately not user-expandable in this field test.
    """
    knots = ((0, 0), (1, 40), (2, 40), (3, 80), (4, 80),
             (5, 40), (6, 40), (7, 0), (8, 0))
    if not 0 <= elapsed_s < VARYING_TARGET_SECONDS:
        return "CMD_VEL 0 0"
    for (start, a), (end, b) in zip(knots, knots[1:]):
        if start <= elapsed_s < end:
            velocity = a + (b - a) * (elapsed_s - start) / (end - start)
            return f"CMD_VEL {round(velocity)} 0"
    return "CMD_VEL 0 0"


def parse_rpdo(lines: list[str], nonce: str) -> list[dict[str, str]] | None:
    complete = collect_numbered_snapshot(lines, "DRIVE_RPDO", 9, nonce)
    if complete is None:
        return None
    rows = [dict(token.split("=", 1) for token in line.split()[2:] if "=" in token)
            for line in complete]
    needed = ["control period gap max_gap queued sent state ready t_ms",
              "interval_ms latency_ms max_latency skew_ticks max_skew pending",
              "busy failed cancel bus_off cancel_timeout reason",
              "node cfg op epoch identity phase expected actual",
              "node cfg op epoch identity phase expected actual",
              "node cob type count map step idx sub abort result",
              "node cob type count map step idx sub abort result",
              "runtime_ms max_runtime_ms runtime_gap_ms over20 max_interval_ms",
              "startup blocked tracking inhibit drive hw_dis basic"]
    if any(not set(fields.split()).issubset(row) for fields, row in zip(needed, rows)):
        return None
    # Firmware does not query 0x1018 during startup/recovery. Keep unknown
    # device numbers explicit in saved JSON; never manufacture numeric IDs.
    for row in rows[3:5]:
        for field in ("vendor", "product", "revision", "serial"):
            row.setdefault(field, "UNKNOWN")
    return rows


def query_rpdo(session: SerialSession) -> list[dict[str, str]]:
    result = session.query_bound("DRIVE_RPDO_DEBUG", parse_rpdo, timeout=3)
    if result is None:
        raise RuntimeError("Incomplete/non-matching RPDO diagnostic snapshot")
    return result


def require_rpdo_ready(rows: list[dict[str, str]]) -> None:
    if int(rows[0]["period"]) != 25:
        raise RuntimeError("Firmware control period is not 25 ms")
    context = "; ".join(f"Node {node}: {rows[node + 2]} {rows[node + 4]}"
                        for node in (1, 2))
    context += f"; Startup: {rows[8]}"
    for node in (1, 2):
        qualification, profile = rows[node + 2], rows[node + 4]
        if (int(qualification["node"]) != node or int(profile["node"]) != node
                or qualification["cfg"] != "1" or qualification["op"] != "1"
                or int(profile["cob"], 16) != 0x300 + node
                or int(profile["type"]) != 254 or int(profile["count"]) != 1
                or int(profile["map"], 16) != 0x60FF0020
                or int(profile["abort"], 16) != 0):
            # Both cfg flags are invalidated if either wheel fails. Include
            # both records so node 1's invalidation cannot hide node 2's cause.
            raise RuntimeError(f"RPDO gate failed: {context}")
    if rows[0]["ready"] != "1":
        raise RuntimeError("RPDO heartbeat/configuration qualification is not current: "
                           f"{rows[0]} {context}")


def rate_summary(before: list[dict[str, str]], after: list[dict[str, str]]) -> dict:
    elapsed = (int(after[0]["t_ms"]) - int(before[0]["t_ms"])) & 0xFFFFFFFF
    if elapsed == 0:
        raise RuntimeError("No elapsed firmware control time")
    delta = lambda key: (int(after[0][key]) - int(before[0][key])) & 0xFFFFFFFF
    return {"elapsed_ms": elapsed, "control_hz": delta("control") * 1000 / elapsed,
            "sent_pair_hz_observed": delta("sent") * 1000 / elapsed,
            "sent_frame_hz_observed": delta("sent") * 2000 / elapsed,
            "queued_pairs": delta("queued"), "max_gap_ms": int(after[0]["max_gap"]),
            "last_skew_nominal_bit_ticks": int(after[1]["skew_ticks"]),
            "dedup_note": "constant targets may send less often than control ticks"}


def main() -> int:
    # Reuse the established identity gate. This tool sends no ENABLE, target,
    # NMT or configuration command; firmware startup owns automatic setup.
    from amr_test_runner import require_identity, require_tpdo_motion_preflight
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyTHS1")
    parser.add_argument("--seconds", type=float, default=0)
    parser.add_argument("--log", type=Path, default=Path("rpdo_first_boot.log"))
    parser.add_argument("--first-boot", action="store_true", help="also require confirmed Drive DISABLED")
    args = parser.parse_args()
    args.log.parent.mkdir(parents=True, exist_ok=True)
    session = SerialSession(args.port, args.log)
    records = []
    try:
        identity = require_identity(session)
        before = query_rpdo(session)
        tpdo = session.wheel_tpdo_active_status()
        records.append({"build_id": identity.build, "rpdo": before,
                        "tpdo": vars(tpdo)})
        print(json.dumps({"pdo_startup": before[8]}))
        require_rpdo_ready(before)
        if (tpdo.mode != "TPDO_ACTIVE" or tpdo.authority != "TPDO" or
                tpdo.pair_valid != 1 or tpdo.pair_age_ms > 150 or tpdo.pair_skew_ms > 30):
            raise RuntimeError("TPDO qualification is incomplete")
        if args.first_boot:
            require_tpdo_motion_preflight(tpdo)
            if tpdo.drive_state != "DISABLED":
                raise RuntimeError("First-boot diagnostics require Drive DISABLED")
        deadline = time.monotonic() + max(0, args.seconds)
        while time.monotonic() < deadline:
            session.read_for(min(1, max(0, deadline - time.monotonic())))
            after = query_rpdo(session)
            require_rpdo_ready(after)
            summary = rate_summary(before, after)
            records.append({"rpdo": after, "rates": summary})
            print(json.dumps(summary))
            before = after
        print("DIAGNOSTIC GATES OK; iWMC series=MANUAL CONFIRMED; "
              "wheel identity=NOT_QUERIED (numbers UNKNOWN); "
              "motion acceptance still requires the lifted-wheel procedure")
        return 0
    finally:
        args.log.with_suffix(".json").write_text(json.dumps(records, indent=2), encoding="utf-8")
        session.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
