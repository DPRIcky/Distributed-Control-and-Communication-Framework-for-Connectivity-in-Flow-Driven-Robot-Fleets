#!/usr/bin/env python3
"""
Plotting script for Monte Carlo stress grid results.

Reads results.csv produced by distributed_pruning_algorithm.py --stress
and generates publication-ready figures.

Usage:
    python make_plots.py --csv stress_results/<tag>/results.csv
    python make_plots.py  # auto-finds newest results.csv under stress_results/

Outputs saved to Figures/ folder:
    - fig_convergence_time_vs_drop.png / .pdf
    - fig_rx_bandwidth_vs_drop.png / .pdf
"""

import argparse
import os
import sys
from pathlib import Path
from glob import glob

try:
    import pandas as pd
    HAS_PANDAS = True
except ImportError:
    import csv
    HAS_PANDAS = False

import matplotlib.pyplot as plt
import matplotlib

# Use non-interactive backend for file saving
matplotlib.use('Agg')


# =============================================================================
# Column name mappings (handle minor naming variations)
# =============================================================================
COLUMN_ALIASES = {
    'p_drop': ['p_drop', 'pdrop', 'drop_prob', 'packet_drop'],
    'delay_max': ['delay_max', 'delay', 'max_delay', 'delaymax'],
    'jitter': ['jitter', 'jitter_max'],
    'avg_duration': ['avg_duration', 'duration', 'convergence_time', 'avg_time'],
    'avg_tx_Bps': ['avg_tx_Bps', 'avg_tx_bps', 'tx_bps', 'tx_bytes_per_second'],
    'avg_rx_Bps': ['avg_rx_Bps', 'avg_rx_bps', 'rx_bps', 'rx_bytes_per_second'],
    'success_rate': ['success_rate', 'success'],
}

REQUIRED_COLUMNS = ['p_drop', 'delay_max', 'jitter', 'avg_duration', 'avg_rx_Bps']


def find_column(df_columns, target):
    """Find matching column name from aliases."""
    aliases = COLUMN_ALIASES.get(target, [target])
    for alias in aliases:
        for col in df_columns:
            if col.lower() == alias.lower():
                return col
    return None


def load_csv(csv_path):
    """Load CSV file and normalize column names."""
    if HAS_PANDAS:
        df = pd.read_csv(csv_path)
    else:
        # Fallback to csv module
        with open(csv_path, 'r', newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            rows = list(reader)
        # Convert to dict of lists
        if rows:
            df_dict = {key: [] for key in rows[0].keys()}
            for row in rows:
                for key, val in row.items():
                    try:
                        df_dict[key].append(float(val))
                    except ValueError:
                        df_dict[key].append(val)
            # Create a simple namespace to mimic pandas DataFrame
            class SimpleDF:
                def __init__(self, data):
                    self._data = data
                    self.columns = list(data.keys())
                def __getitem__(self, key):
                    return self._data[key]
                def __setitem__(self, key, val):
                    self._data[key] = val
                    if key not in self.columns:
                        self.columns.append(key)
            df = SimpleDF(df_dict)
        else:
            raise ValueError("CSV file is empty")
    
    return df


def validate_and_map_columns(df):
    """Validate required columns exist and create mapping."""
    df_columns = list(df.columns)
    column_map = {}
    missing = []
    
    for target in REQUIRED_COLUMNS:
        found = find_column(df_columns, target)
        if found:
            column_map[target] = found
        else:
            missing.append(target)
    
    # Also try to find optional columns
    for target in ['avg_tx_Bps', 'success_rate']:
        if target not in column_map:
            found = find_column(df_columns, target)
            if found:
                column_map[target] = found
    
    if missing:
        print(f"ERROR: Missing required columns: {missing}")
        print(f"Detected columns in CSV: {df_columns}")
        print(f"Column aliases checked: {COLUMN_ALIASES}")
        sys.exit(1)
    
    return column_map


def get_groups(df, col_map):
    """Extract unique (delay_max, jitter) groups."""
    if HAS_PANDAS:
        groups = df[[col_map['delay_max'], col_map['jitter']]].drop_duplicates()
        return [(row[col_map['delay_max']], row[col_map['jitter']]) 
                for _, row in groups.iterrows()]
    else:
        seen = set()
        groups = []
        delay_col = col_map['delay_max']
        jitter_col = col_map['jitter']
        for i in range(len(df[delay_col])):
            pair = (df[delay_col][i], df[jitter_col][i])
            if pair not in seen:
                seen.add(pair)
                groups.append(pair)
        return groups


def filter_data(df, col_map, delay_val, jitter_val):
    """Filter data for a specific (delay, jitter) combination."""
    delay_col = col_map['delay_max']
    jitter_col = col_map['jitter']
    
    if HAS_PANDAS:
        mask = (df[delay_col] == delay_val) & (df[jitter_col] == jitter_val)
        subset = df[mask].sort_values(col_map['p_drop'])
        return subset
    else:
        indices = []
        for i in range(len(df[delay_col])):
            if df[delay_col][i] == delay_val and df[jitter_col][i] == jitter_val:
                indices.append(i)
        # Sort by p_drop
        p_drop_col = col_map['p_drop']
        indices.sort(key=lambda i: df[p_drop_col][i])
        
        class FilteredDF:
            def __init__(self, df, indices):
                self._df = df
                self._indices = indices
            def __getitem__(self, key):
                return [self._df[key][i] for i in self._indices]
        
        return FilteredDF(df, indices)


def find_newest_csv():
    """Find the newest results.csv under stress_results/ or scripts/stress_results/."""
    # Search in multiple likely locations
    search_patterns = [
        os.path.join("stress_results", "**", "results.csv"),
        os.path.join("scripts", "stress_results", "**", "results.csv"),
        os.path.join(os.path.dirname(__file__), "stress_results", "**", "results.csv"),
    ]
    
    all_files = []
    for pattern in search_patterns:
        files = glob(pattern, recursive=True)
        all_files.extend(files)
    
    if not all_files:
        return None
    
    # Deduplicate by absolute path
    unique = list(set(os.path.abspath(f) for f in all_files))
    
    # Sort by modification time
    unique.sort(key=lambda f: os.path.getmtime(f), reverse=True)
    return unique[0]


def plot_convergence_time(df, col_map, output_dir):
    """
    Plot 1: Convergence time vs packet drop probability.
    """
    fig, ax = plt.subplots(figsize=(8, 6))
    
    groups = get_groups(df, col_map)
    markers = ['o', 's', '^', 'D', 'v', '<', '>', 'p', 'h']
    colors = plt.cm.tab10.colors
    
    for idx, (delay_val, jitter_val) in enumerate(sorted(groups)):
        subset = filter_data(df, col_map, delay_val, jitter_val)
        
        p_drop = subset[col_map['p_drop']]
        duration = subset[col_map['avg_duration']]
        
        label = f"delay={delay_val}, jitter={jitter_val}"
        marker = markers[idx % len(markers)]
        color = colors[idx % len(colors)]
        
        ax.plot(p_drop, duration, marker=marker, color=color, 
                label=label, linewidth=2, markersize=8)
    
    ax.set_xlabel("Packet drop probability $p_{drop}$", fontsize=12)
    ax.set_ylabel("Convergence time (s)", fontsize=12)
    ax.set_title(r"$\delta$-BFS convergence time vs packet drops", fontsize=14)
    ax.legend(loc='lower left', fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(left=-0.02)
    ax.set_ylim(bottom=0)
    
    plt.tight_layout()
    
    # Save PNG and PDF
    png_path = os.path.join(output_dir, "fig_convergence_time_vs_drop.png")
    pdf_path = os.path.join(output_dir, "fig_convergence_time_vs_drop.pdf")
    
    fig.savefig(png_path, dpi=300, bbox_inches='tight')
    fig.savefig(pdf_path, bbox_inches='tight')
    plt.close(fig)
    
    print(f"  Saved: {png_path}")
    print(f"  Saved: {pdf_path}")


def plot_bandwidth(df, col_map, output_dir):
    """
    Plot 2: Delivered bandwidth (RX B/s) vs packet drop probability.
    Optionally also show TX bandwidth as dashed lines.
    """
    fig, ax = plt.subplots(figsize=(8, 6))
    
    groups = get_groups(df, col_map)
    markers = ['o', 's', '^', 'D', 'v', '<', '>', 'p', 'h']
    colors = plt.cm.tab10.colors
    
    has_tx = 'avg_tx_Bps' in col_map
    
    for idx, (delay_val, jitter_val) in enumerate(sorted(groups)):
        subset = filter_data(df, col_map, delay_val, jitter_val)
        
        p_drop = subset[col_map['p_drop']]
        rx_bps = subset[col_map['avg_rx_Bps']]
        
        marker = markers[idx % len(markers)]
        color = colors[idx % len(colors)]
        
        # RX bandwidth (solid line, primary)
        label_rx = f"RX: delay={delay_val}, jitter={jitter_val}"
        ax.plot(p_drop, rx_bps, marker=marker, color=color,
                label=label_rx, linewidth=2, markersize=8, linestyle='-')
        
        # TX bandwidth (dashed line, secondary) if available
        if has_tx:
            tx_bps = subset[col_map['avg_tx_Bps']]
            label_tx = f"TX: delay={delay_val}, jitter={jitter_val}"
            ax.plot(p_drop, tx_bps, marker=marker, color=color,
                    label=label_tx, linewidth=1.5, markersize=6, 
                    linestyle='--', alpha=0.6)
    
    ax.set_xlabel("Packet drop probability $p_{drop}$", fontsize=12)
    ax.set_ylabel("Bandwidth (B/s)", fontsize=12)
    ax.set_title("Delivered bandwidth vs packet drops (payload=6 bytes/msg)", fontsize=14)
    
    ax.legend(loc='lower left', fontsize=9, framealpha=0.9)

    ax.grid(True, alpha=0.3)
    ax.set_xlim(left=-0.02)
    ax.set_ylim(bottom=0)
    
    plt.tight_layout()
    
    # Save PNG and PDF
    png_path = os.path.join(output_dir, "fig_rx_bandwidth_vs_drop.png")
    pdf_path = os.path.join(output_dir, "fig_rx_bandwidth_vs_drop.pdf")
    
    fig.savefig(png_path, dpi=300, bbox_inches='tight')
    fig.savefig(pdf_path, bbox_inches='tight')
    plt.close(fig)
    
    print(f"  Saved: {png_path}")
    print(f"  Saved: {pdf_path}")


def get_default_figures_dir():
    """Return path to Figures/ folder at project root."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # If script is in scripts/, go up one level
    parent = os.path.dirname(script_dir)
    candidates = [
        os.path.join(parent, "Figures"),  # ../Figures from scripts/
        os.path.join(script_dir, "Figures"),  # ./Figures
        "Figures",  # current working directory
    ]
    for c in candidates:
        if os.path.isdir(c):
            return c
    # Default to project root's Figures
    return os.path.join(parent, "Figures")


def main():
    parser = argparse.ArgumentParser(
        description="Generate plots from Monte Carlo stress grid results."
    )
    parser.add_argument(
        '--csv', type=str, default=None,
        help="Path to results.csv. If not provided, auto-finds newest under stress_results/."
    )
    parser.add_argument(
        '--outdir', type=str, default=None,
        help="Output directory for figures (default: Figures/ at project root)"
    )
    
    args = parser.parse_args()
    
    # Find CSV file
    if args.csv:
        csv_path = args.csv
    else:
        csv_path = find_newest_csv()
        if csv_path is None:
            print("ERROR: No results.csv found under stress_results/")
            print("Run: python distributed_pruning_algorithm.py --stress --outdir stress_results")
            sys.exit(1)
    
    if not os.path.exists(csv_path):
        print(f"ERROR: CSV file not found: {csv_path}")
        sys.exit(1)
    
    print(f"Loading: {csv_path}")
    
    # Load data
    df = load_csv(csv_path)
    
    # Validate columns
    col_map = validate_and_map_columns(df)
    print(f"Column mapping: {col_map}")
    
    # Create output directory
    output_dir = args.outdir if args.outdir else get_default_figures_dir()
    os.makedirs(output_dir, exist_ok=True)
    print(f"Output directory: {output_dir}")
    
    # Generate plots
    print("\nGenerating plots...")
    
    print("\nPlot 1: Convergence time vs packet drop")
    plot_convergence_time(df, col_map, output_dir)
    
    print("\nPlot 2: Delivered bandwidth vs packet drop")
    plot_bandwidth(df, col_map, output_dir)
    
    print("\nDone!")


if __name__ == "__main__":
    main()
