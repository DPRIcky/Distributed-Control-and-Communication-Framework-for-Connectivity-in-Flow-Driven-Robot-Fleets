"""Regenerate ACC 2026 figures in concurrent_pruning/figures.

This script copies the canonical figure1_acc2026.png ... figure8_acc2026.png
from the workspace figures/ directory into concurrent_pruning/figures.
"""

from pathlib import Path
import shutil


def main() -> None:
    root_dir = Path(__file__).resolve().parents[1]
    source_dir = root_dir / "figures"
    output_dir = root_dir / "concurrent_pruning" / "figures"
    output_dir.mkdir(parents=True, exist_ok=True)

    copied = 0
    missing = []

    for idx in range(1, 9):
        name = f"figure{idx}_acc2026.png"
        src = source_dir / name
        dst = output_dir / name
        if not src.exists():
            missing.append(name)
            continue
        shutil.copy2(src, dst)
        copied += 1

    print(f"Copied {copied} figures to {output_dir}")
    if missing:
        print("Missing sources:")
        for name in missing:
            print(f"  - {name}")


if __name__ == "__main__":
    main()
