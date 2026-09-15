# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a LaTeX semester report documenting HoloOcean pruning and sequential-pruning research. The report covers communication-feasible topology management for underwater multi-agent systems, including:
- δ-BFS distributed pruning baseline
- Gilbert-Elliott bursty channel modeling
- Rate-cap and bandwidth experiments
- Distance-dependent impairment studies
- Sequential pruning with committed parents and grace periods

## Build Commands

Compile the LaTeX document:
```bash
pdflatex -interaction=nonstopmode main.tex
```

For a complete build with references, run twice:
```bash
pdflatex -interaction=nonstopmode main.tex && pdflatex -interaction=nonstopmode main.tex
```

Output PDF is generated at `main.pdf` (and `out/main.pdf`).

## File Structure

- `main.tex` - Main LaTeX source, the only file that needs editing
- `SEMESTER_REPORT_MOTHERSHIP.md` - Markdown reference version of the report content
- `main.pdf` / `out/main.pdf` - Compiled output
- `out/` - Auxiliary build files (`.aux`, `.log`, `.fls`, `.fdb_latexmk`, `.synctex.gz`)

## LaTeX Conventions

- Uses `mathptmx` for Times font
- A4 paper with custom margins (0.1666×paperwidth left/right, 0.1111×paperheight top/bottom)
- Uses `microtype` for typography improvements
- Figures are currently placeholders - actual figures reference paths like `holoocean_pruning_project/figures/` which exist in separate research directories

## Note on Referenced Code

The report references code and results from external directories:
- `holoocean_pruning_project/` - HoloOcean pruning project with installable `pruning` package
- `holoocean sequential pruning/` - Sequential pruning extension

These directories contain the actual research implementation but are not part of this report directory. When updating the report with results, figures should be copied into this directory or the `\includegraphics` paths adjusted.