"""Quick 2-trial end-to-end test of compare_methods.py"""
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

# Monkey-patch to run fast
import compare_methods as cm
cm.TRIALS = 2
cm.N_VALUES = [10]

cm.main()
