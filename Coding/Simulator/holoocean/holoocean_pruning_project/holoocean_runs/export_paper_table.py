#!/usr/bin/env python
"""
Export a paper-ready 'Simulation 2 results' table from rate-cap sweep CSVs.

Outputs:
  paper_table.csv   — machine-readable summary
  paper_table.tex   — LaTeX tabular snippet (paste into paper)

Usage (from holoocean_pruning_project/):
    python -m holoocean_runs.export_paper_table \\
        "sweeps/rate_cap_N10/*.csv" \\
        "sweeps/rate_cap_N20/sweep_N20_rebuilt.csv" \\
        "sweeps/breakpoint_N20/sweep_N20_breakpoint_rebuilt.csv" \\
        --out_dir sweeps/paper
"""
import argparse
import glob
import math
from pathlib import Path

import numpy as np
import pandas as pd

# Selected cap rows per N: (cap_bps, description)
_SELECTED_CAPS = {
    10: [
        (float("inf"), "No limit"),
        (250,          "High cap"),
        (48,           "2× above breakpoint"),
        (30,           "Above breakpoint"),
        (18,           "Near breakpoint (partial)"),
        (12,           "Below breakpoint (failure)"),
    ],
    20: [
        (float("inf"), "No limit"),
        (250,          "High cap"),
        (48,           "2× above breakpoint"),
        (30,           "Above breakpoint"),
        (24,           "Near breakpoint (first success)"),
        (18,           "Below breakpoint (failure)"),
    ],
}


def expand_globs(patterns):
    expanded = []
    for p in patterns:
        matches = glob.glob(p)
        expanded.extend(matches if matches else [p])
    return expanded


def load_and_merge(csv_paths):
    expanded = expand_globs(csv_paths)
    dfs = []
    seen = set()
    for p in expanded:
        if not Path(p).exists():
            print(f"Warning: not found: {p}")
            continue
        p_res = str(Path(p).resolve())
        if p_res in seen:
            continue
        seen.add(p_res)
        dfs.append(pd.read_csv(p))
    if not dfs:
        raise ValueError("No CSV files found")
    merged = pd.concat(dfs, ignore_index=True)
    key_cols = [c for c in ["rate_cap_bps", "seed", "N"] if c in merged.columns]
    merged = merged.drop_duplicates(subset=key_cols)
    return merged


def cap_to_numeric(cap):
    if cap == "inf":
        return float("inf")
    return float(cap)


def aggregate_by_cap(df):
    df = df.copy()
    df["cap_numeric"] = df["rate_cap_bps"].apply(cap_to_numeric)
    sem = lambda x: x.std() / np.sqrt(x.count()) if x.count() > 1 else np.nan

    rows = []
    for cap_val, grp in df.groupby("cap_numeric"):
        rows.append({
            "cap_numeric": cap_val,
            "n_runs": len(grp),
            "success_rate": grp["success"].mean(),
            "t_tree_first_mean": grp["t_tree_first"].mean(),
            "t_tree_first_sem": grp["t_tree_first"].sem(),
            "t_stable_mean": grp["t_tree_stable_first"].mean(),
            "rx_Bps_mean": grp["rx_Bps"].mean(),
            "delay_p95_mean_ms": grp["queue_delay_p95_s"].mean() * 1000,
            "delay_p95_sem_ms": grp["queue_delay_p95_s"].sem() * 1000,
        })
    return pd.DataFrame(rows)


def fmt_mean_sem(mean, sem, decimals=1, unit=""):
    if pd.isna(mean):
        return "—"
    if pd.isna(sem) or sem == 0:
        return f"{mean:.{decimals}f}{unit}"
    return f"{mean:.{decimals}f}±{sem:.{decimals}f}{unit}"


def cap_label(cap):
    if math.isinf(cap):
        return "∞"
    if cap >= 1000:
        return f"{int(cap/1000)}k"
    return str(int(cap))


def build_output_rows(df_all):
    n_values = sorted(df_all["N"].dropna().unique())
    output_rows = []

    for n_val in n_values:
        n_val = int(n_val)
        df_n = df_all[df_all["N"] == n_val]
        agg = aggregate_by_cap(df_n)

        selected = _SELECTED_CAPS.get(n_val, [])
        for cap_bps, desc in selected:
            # Find matching row
            if math.isinf(cap_bps):
                row = agg[agg["cap_numeric"].apply(math.isinf)]
            else:
                row = agg[agg["cap_numeric"].apply(lambda c: not math.isinf(c) and int(round(c)) == int(cap_bps))]

            if row.empty:
                print(f"  [warning] N={n_val}, cap={cap_bps} not found in data")
                output_rows.append({
                    "N": n_val,
                    "cap_bps": cap_bps,
                    "cap_label": cap_label(cap_bps),
                    "description": desc,
                    "n_runs": 0,
                    "success_rate": np.nan,
                    "t_tree_first_mean_s": np.nan,
                    "t_tree_first_sem_s": np.nan,
                    "delay_p95_mean_ms": np.nan,
                    "delay_p95_sem_ms": np.nan,
                    "rx_Bps_mean": np.nan,
                })
                continue

            r = row.iloc[0]
            output_rows.append({
                "N": n_val,
                "cap_bps": cap_bps,
                "cap_label": cap_label(cap_bps),
                "description": desc,
                "n_runs": int(r["n_runs"]),
                "success_rate": round(r["success_rate"], 3),
                "t_tree_first_mean_s": round(r["t_tree_first_mean"], 2) if not pd.isna(r["t_tree_first_mean"]) else np.nan,
                "t_tree_first_sem_s": round(r["t_tree_first_sem"], 2) if not pd.isna(r["t_tree_first_sem"]) else np.nan,
                "delay_p95_mean_ms": round(r["delay_p95_mean_ms"], 1),
                "delay_p95_sem_ms": round(r["delay_p95_sem_ms"], 1) if not pd.isna(r["delay_p95_sem_ms"]) else np.nan,
                "rx_Bps_mean": round(r["rx_Bps_mean"], 1),
            })

    return pd.DataFrame(output_rows)


def to_latex(out_df: pd.DataFrame) -> str:
    lines = []
    lines.append(r"\begin{table}[ht]")
    lines.append(r"\centering")
    lines.append(r"\caption{δ-BFS Protocol: Rate Cap Sweep Summary (N=10, N=20, $T_\text{prune}$=30\,s, 3 seeds)}")
    lines.append(r"\label{tab:rate_cap_sweep}")
    lines.append(r"\begin{tabular}{llccrrrr}")
    lines.append(r"\toprule")
    lines.append(r"$N$ & Cap (bps) & Description & Runs & Success & "
                 r"$t_\text{tree}$ (s) & Delay p95 (ms) & rx (B/s) \\")
    lines.append(r"\midrule")

    prev_n = None
    for _, row in out_df.iterrows():
        n_val = int(row["N"])
        if prev_n is not None and n_val != prev_n:
            lines.append(r"\midrule")
        prev_n = n_val

        n_str = str(n_val) if prev_n != n_val else ""  # no repeated N
        # Always print N for first row of group
        cap_str = row["cap_label"].replace("∞", r"$\infty$")
        desc = row["description"]
        n_runs = int(row["n_runs"]) if not pd.isna(row["n_runs"]) else 0
        succ = f"{row['success_rate']:.2f}" if not pd.isna(row["success_rate"]) else "—"
        t_tree = fmt_mean_sem(row["t_tree_first_mean_s"], row["t_tree_first_sem_s"], decimals=1)
        delay = fmt_mean_sem(row["delay_p95_mean_ms"], row["delay_p95_sem_ms"], decimals=0)
        rx = f"{row['rx_Bps_mean']:.0f}" if not pd.isna(row["rx_Bps_mean"]) else "—"

        lines.append(
            f"{n_val} & {cap_str} & {desc} & {n_runs} & {succ} & {t_tree} & {delay} & {rx} \\\\"
        )

    lines.append(r"\bottomrule")
    lines.append(r"\end{tabular}")
    lines.append(r"\end{table}")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Export paper results table from rate-cap CSVs")
    parser.add_argument("csv_files", nargs="+", help="Input CSV files (globs allowed)")
    parser.add_argument("--out_dir", type=str, default="sweeps/paper",
                        help="Output directory for paper_table.csv and paper_table.tex")
    args = parser.parse_args()

    project_dir = Path(__file__).resolve().parent.parent
    out_dir = project_dir / args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    df_all = load_and_merge(args.csv_files)
    print(f"Loaded {len(df_all)} rows total")
    n_values = sorted(df_all["N"].dropna().unique())
    print(f"N values: {n_values}")

    out_df = build_output_rows(df_all)

    # Save CSV
    csv_path = out_dir / "paper_table.csv"
    out_df.to_csv(csv_path, index=False)
    print(f"CSV: {csv_path}")

    # Save LaTeX
    tex_path = out_dir / "paper_table.tex"
    tex_str = to_latex(out_df)
    tex_path.write_text(tex_str, encoding="utf-8")
    print(f"LaTeX: {tex_path}")

    # Print preview
    print("\n--- Table preview ---")
    pd.set_option("display.max_columns", None)
    pd.set_option("display.width", 120)
    print(out_df.to_string(index=False))
    print("\n--- LaTeX ---")
    print(tex_str)


if __name__ == "__main__":
    main()
