#!/usr/bin/env python3
"""
tools/view_logs.py — Parse logs/telemetry.jsonl and print a mission summary.

Usage:
  python tools/view_logs.py                        # latest telemetry.jsonl
  python tools/view_logs.py --file logs/my.jsonl   # specific file
  python tools/view_logs.py --errors-only          # only WARNING/ERROR/CRITICAL
  python tools/view_logs.py --states               # only state transitions
"""

import argparse, json, sys, os
from collections import Counter


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", default="logs/telemetry.jsonl")
    ap.add_argument("--errors-only", action="store_true")
    ap.add_argument("--states",      action="store_true")
    args = ap.parse_args()

    if not os.path.exists(args.file):
        print(f"File not found: {args.file}")
        sys.exit(1)

    entries = []
    with open(args.file) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                entries.append(json.loads(line))
            except json.JSONDecodeError:
                pass

    if not entries:
        print("No log entries found.")
        return

    print(f"\n{'='*60}")
    print(f"  Mission Log Summary — {args.file}")
    print(f"  {len(entries)} entries  |  {entries[0]['ts']} → {entries[-1]['ts']}")
    print(f"{'='*60}\n")

    level_counts = Counter(e["level"] for e in entries)
    print("Level counts:")
    for lvl in ("DEBUG","INFO","WARNING","ERROR","CRITICAL"):
        n = level_counts.get(lvl, 0)
        marker = " ◄◄◄" if lvl in ("ERROR","CRITICAL") and n > 0 else ""
        print(f"  {lvl:<10} {n:>5}{marker}")
    print()

    # State transitions
    state_entries = [e for e in entries if e.get("module") == "rov.autonomy_core"
                     and ("━━ STATE:" in e.get("msg","") or "Transition:" in e.get("msg",""))]
    if state_entries:
        print("State transitions:")
        for e in state_entries:
            print(f"  {e['ts']}  {e['msg']}")
        print()

    # Errors
    errors = [e for e in entries if e["level"] in ("WARNING","ERROR","CRITICAL")]
    if errors:
        print(f"Warnings / Errors / Criticals ({len(errors)}):")
        for e in errors:
            lvl = e["level"]
            col = {"WARNING":"\033[33m","ERROR":"\033[31m","CRITICAL":"\033[35m"}.get(lvl,"")
            rst = "\033[0m"
            print(f"  {col}{lvl:<10}{rst} [{e['module']}] {e['ts']}  {e['msg'][:100]}")
    else:
        print("\033[32mNo warnings or errors — clean run!\033[0m")

    print(f"\n{'='*60}\n")


if __name__ == "__main__":
    main()
