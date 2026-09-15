"""
Smoke test: confirms that the pruning package can be imported alongside
holoocean and numpy, then runs a minimal 3-agent HoloOcean simulation for
~120 ticks, printing each agent's LocationSensor once per second.
"""

import os
import sys

print("cwd            :", os.getcwd())
print("sys.path[0]    :", sys.path[0])

# ── Package imports ──────────────────────────────────────────────────────────
import holoocean
import numpy as np
from pruning.distributed_pruning_algorithm import DistributedPruningAlgorithm

print("Import OK")
print("  holoocean version :", getattr(holoocean, "__version__", "unknown"))
print("  numpy version     :", np.__version__)
print("  DistributedPruningAlgorithm :", DistributedPruningAlgorithm)

# ── Minimal 3-agent config (same structure as keyboard_control.py) ───────────
config = {
    "name": "smoke_test_3agents",
    "world": "SimpleUnderwater",
    "package_name": "Ocean",
    "main_agent": "auv0",
    "ticks_per_sec": 60,
    "agents": [
        {
            "agent_name": "auv0",
            "agent_type": "BlueROV2",
            "sensors": [
                {"sensor_type": "LocationSensor"}
            ],
            "control_scheme": 0,
            "location": [0, 0, -5],
            "rotation": [0, 0, 0]
        },
        {
            "agent_name": "auv1",
            "agent_type": "BlueROV2",
            "sensors": [
                {"sensor_type": "LocationSensor"}
            ],
            "control_scheme": 0,
            "location": [5, 0, -5],
            "rotation": [0, 0, 0]
        },
        {
            "agent_name": "auv2",
            "agent_type": "BlueROV2",
            "sensors": [
                {"sensor_type": "LocationSensor"}
            ],
            "control_scheme": 0,
            "location": [10, 0, -5],
            "rotation": [0, 0, 0]
        }
    ]
}

agent_names = ["auv0", "auv1", "auv2"]
TICKS_PER_SEC = 60
TOTAL_TICKS = 120

print(f"\nStarting HoloOcean smoke test ({TOTAL_TICKS} ticks) …")

with holoocean.make(scenario_cfg=config) as env:
    for tick in range(TOTAL_TICKS):
        # Send zero commands so agents drift naturally
        for name in agent_names:
            env.act(name, np.zeros(8))

        states = env.tick()

        # Print once per simulated second
        if tick % TICKS_PER_SEC == 0:
            sec = tick // TICKS_PER_SEC
            print(f"\n[t={sec}s]")
            for name in agent_names:
                loc = states[name]["LocationSensor"]
                print(f"  {name} location: [{loc[0]:.2f}, {loc[1]:.2f}, {loc[2]:.2f}]")

print("\nSmoke test complete.")
