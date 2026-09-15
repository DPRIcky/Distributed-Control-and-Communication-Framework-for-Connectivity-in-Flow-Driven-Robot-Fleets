# HoloOcean Pruning Project – Import Setup

## Project layout

```
holoocean_pruning_project/
├── pyproject.toml          ← makes 'pruning' pip-installable
├── pruning/
│   ├── __init__.py
│   └── distributed_pruning_algorithm.py
├── holoocean_runs/
│   ├── keyboard_control.py
│   ├── autonomous_control.py
│   └── run_import_smoke_test.py
└── README_IMPORTS.md
```

## How to run

### 1. Open a terminal and `cd` to the project root

```powershell
cd "C:\Users\prajj\OneDrive - Arizona State University\ASU\PhD\Research\Coding\Simulator\holoocean\holoocean_pruning_project"
```

### 2. Install the project in editable mode (one-time setup)

```powershell
pip install -e .
```

This registers the `pruning` package with Python so it is importable from any
directory — no `sys.path` hacks required.

### 3. Run the smoke test

```powershell
python holoocean_runs\run_import_smoke_test.py
```

### 4. (Optional) Run the HoloOcean scripts

```powershell
python holoocean_runs\keyboard_control.py
python holoocean_runs\autonomous_control.py
```

## Why this works

`pip install -e .` reads `pyproject.toml` and creates an editable link so that
`import pruning` resolves to the local `pruning/` directory.  Because the package
is properly installed, scripts can import it regardless of which directory they
live in — Python's standard package machinery handles everything.

## Note

- You only need to run `pip install -e .` once (or again after changing
  `pyproject.toml`).
- You must run from the **project root** so pip finds `pyproject.toml`.
