"""Summarise results/landing_runs.csv as a Markdown table for the README.

usage: python tools/summarize_results.py [results/landing_runs.csv]
"""
import csv
import statistics as st
import sys
from pathlib import Path


def summarize(rows: list[dict]) -> str:
    groups = {"Nominal (no occlusion)": [r for r in rows if not r["occlusion"]],
              "With 3 s simulated occlusion": [r for r in rows if r["occlusion"]]}
    out = ["| Condition | Runs | Precision landings | Mean error | Median | Std dev | Max |",
           "|---|---|---|---|---|---|---|"]
    for name, rs in groups.items():
        if not rs:
            continue
        ok = [r for r in rs if r["outcome"] == "FINAL"]
        e = [float(r["error_m"]) * 100 for r in ok]
        sd = st.stdev(e) if len(e) > 1 else 0.0
        out.append(f"| {name} | {len(rs)} | {len(ok)}/{len(rs)} | {st.mean(e):.1f} cm | "
                   f"{st.median(e):.1f} cm | {sd:.1f} cm | {max(e):.1f} cm |")
    out += ["", "| Run | Approach error N, E (m) | Occlusion | Outcome | Marker losses | Vision phase (sim s) | Landing error |",
            "|---|---|---|---|---|---|---|"]
    for r in rows:
        err = f"{float(r['error_m']) * 100:.1f} cm" if r.get("error_m") else "-"
        out.append(f"| {r['run_id']} | {float(r['offset_n']):+.2f}, {float(r['offset_e']):+.2f} | "
                   f"{r['occlusion'] or '-'} | {r['outcome']} | {r.get('loss_events') or '-'} | "
                   f"{r.get('vision_time_s') or '-'} | {err} |")
    return "\n".join(out)


if __name__ == "__main__":
    path = Path(sys.argv[1] if len(sys.argv) > 1 else "results/landing_runs.csv")
    with open(path) as fh:
        print(summarize(list(csv.DictReader(fh))))
