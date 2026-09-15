"""Legacy interface wrapper for backward compatibility.

DEPRECATED: This file maintains backward compatibility with the old monolithic script.
Please use 'python main.py' instead.

Usage:
    python underwater_consensus_chain_hybrid.py         # text mode
    python underwater_consensus_chain_hybrid.py gui     # GUI mode
"""

import warnings
import sys

warnings.warn(
    "underwater_consensus_chain_hybrid.py is deprecated. "
    "Please use 'python main.py' instead.",
    DeprecationWarning,
    stacklevel=2
)

# Import and run the new modular main
from main import main

if __name__ == "__main__":
    main()
