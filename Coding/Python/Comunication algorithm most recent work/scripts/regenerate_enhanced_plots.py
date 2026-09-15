"""Regenerate enhanced plots from existing experiment results.

This script finds experiment result JSON files and regenerates
all plots using the enhanced visualization module.
"""

import sys
from pathlib import Path

# Add repo root to path
REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from visualization.enhanced_comparison_plots import generate_all_enhanced_plots
import argparse


def find_result_files(search_dir: Path, pattern: str = "*.json") -> list:
    """Find all result JSON files in a directory."""
    return sorted(search_dir.rglob(pattern))


def main():
    parser = argparse.ArgumentParser(
        description='Regenerate enhanced plots from experiment results'
    )
    parser.add_argument(
        'input_path',
        nargs='?',
        default='experiments/results_section_20260219',
        help='Path to results directory or JSON file (default: experiments/results_section_20260219)'
    )
    parser.add_argument(
        '--pattern',
        default='*concurrent*.json',
        help='File pattern to search for (default: *concurrent*.json)'
    )
    parser.add_argument(
        '--output-dir',
        help='Specific output directory (default: enhanced_plots/ in same dir as input)'
    )
    parser.add_argument(
        '--methods',
        nargs='+',
        help='Specific methods to include in plots'
    )
    
    args = parser.parse_args()
    
    input_path = Path(args.input_path)
    
    if not input_path.exists():
        print(f"❌ Error: Path does not exist: {input_path}")
        return 1
    
    # Determine result files
    if input_path.is_file():
        result_files = [input_path]
    else:
        result_files = find_result_files(input_path, args.pattern)
    
    if not result_files:
        print(f"❌ No result files found matching pattern: {args.pattern}")
        return 1
    
    print(f"Found {len(result_files)} result file(s):")
    for f in result_files:
        print(f"  - {f}")
    print()
    
    # Process each file
    for result_file in result_files:
        try:
            print(f"\n{'=' * 80}")
            print(f"Processing: {result_file.name}")
            print(f"{'=' * 80}\n")
            
            # Determine output directory
            if args.output_dir:
                output_dir = Path(args.output_dir)
            else:
                output_dir = result_file.parent / 'enhanced_plots'
            
            generate_all_enhanced_plots(
                str(result_file),
                str(output_dir),
                args.methods
            )
            
        except Exception as e:
            print(f"❌ Error processing {result_file.name}: {e}")
            import traceback
            traceback.print_exc()
    
    print(f"\n{'=' * 80}")
    print("🎉 COMPLETE - Enhanced plots generated!")
    print(f"{'=' * 80}\n")
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
