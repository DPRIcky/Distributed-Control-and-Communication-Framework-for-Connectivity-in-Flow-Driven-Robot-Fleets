"""Check if enhanced visualization dependencies are installed and working.

Run this script to verify your setup before generating plots.
"""

import sys
from pathlib import Path

print("=" * 80)
print("ENHANCED VISUALIZATION - DEPENDENCY CHECK")
print("=" * 80)
print()

# Check Python version
print(f"✓ Python version: {sys.version.split()[0]}")

# Check required packages
packages = {
    'numpy': 'NumPy',
    'matplotlib': 'Matplotlib',
    'pandas': 'Pandas',
    'seaborn': 'Seaborn',
    'scipy': 'SciPy',
}

missing = []
installed = []

for package, name in packages.items():
    try:
        mod = __import__(package)
        version = getattr(mod, '__version__', 'unknown')
        installed.append(f"✓ {name:<15} {version}")
    except ImportError:
        missing.append(f"✗ {name:<15} NOT INSTALLED")

print("\nInstalled packages:")
for pkg in installed:
    print(f"  {pkg}")

if missing:
    print("\n⚠️  Missing packages:")
    for pkg in missing:
        print(f"  {pkg}")
    print("\nTo install missing packages:")
    print("  pip install -r requirements.txt")
    print("\nOr individually:")
    if any('Pandas' in pkg for pkg in missing):
        print("  pip install pandas>=1.3.0")
    if any('Seaborn' in pkg for pkg in missing):
        print("  pip install seaborn>=0.11.0")
    if any('SciPy' in pkg for pkg in missing):
        print("  pip install scipy>=1.7.0")
else:
    print("\n✅ All dependencies installed!")

# Check if visualization module exists
print("\nChecking module files...")
root = Path(__file__).parent.parent
viz_module = root / 'visualization' / 'enhanced_comparison_plots.py'
regen_script = root / 'scripts' / 'regenerate_enhanced_plots.py'

if viz_module.exists():
    print(f"  ✓ Enhanced plots module found")
else:
    print(f"  ✗ Enhanced plots module NOT found at {viz_module}")

if regen_script.exists():
    print(f"  ✓ Regeneration script found")
else:
    print(f"  ✗ Regeneration script NOT found at {regen_script}")

# Try importing the module
print("\nTrying to import visualization module...")
try:
    sys.path.insert(0, str(root))
    from visualization.enhanced_comparison_plots import generate_all_enhanced_plots
    print("  ✓ Import successful!")
    
    # List available functions
    import visualization.enhanced_comparison_plots as ecp
    funcs = [f for f in dir(ecp) if f.startswith('plot_')]
    print(f"\n  Available plot functions:")
    for func in funcs:
        print(f"    - {func}")
    
except ImportError as e:
    print(f"  ✗ Import failed: {e}")
    print(f"\n  Make sure all dependencies are installed:")
    print(f"    pip install pandas seaborn scipy")

# Check for result files
print("\nChecking for existing result files...")
results_dir = root / 'experiments' / 'results_section_20260219'
if results_dir.exists():
    json_files = list(results_dir.rglob('*.json'))
    if json_files:
        print(f"  Found {len(json_files)} JSON file(s):")
        for f in json_files[:3]:  # Show first 3
            print(f"    - {f.relative_to(root)}")
        if len(json_files) > 3:
            print(f"    ... and {len(json_files) - 3} more")
        print("\n  You can regenerate plots with:")
        print(f"    python scripts/regenerate_enhanced_plots.py {results_dir.relative_to(root)}")
    else:
        print("  No result JSON files found yet.")
        print("\n  Generate results first with:")
        print("    python experiments/results_section_20260219/run_three_method_study.py --seeds 30")
else:
    print(f"  Results directory not found: {results_dir}")

print("\n" + "=" * 80)

if not missing:
    print("✅ SETUP COMPLETE - Ready to generate enhanced plots!")
    print("\nQuick Start:")
    print("  1. Run experiments (if you haven't):")
    print("     python experiments/results_section_20260219/run_three_method_study.py --seeds 30")
    print("\n  2. Enhanced plots will be in: [run_dir]/plots/enhanced/")
    print("\n  Or regenerate from existing results:")
    print("     python scripts/regenerate_enhanced_plots.py experiments/results_section_20260219")
else:
    print("⚠️  SETUP INCOMPLETE - Install missing dependencies first")
    print("   pip install -r requirements.txt")

print("=" * 80)
