"""Stability test for GHS with edge pruning."""
import subprocess
import re

RUNS = 10
results = {"4": 0, "5": 0, "6": 0, "8": 0}
total_runs = 0

print(f"Running {RUNS} iterations of GHS convergence tests...\n")

for i in range(RUNS):
    print(f"Run {i+1}/{RUNS}...", end=" ", flush=True)
    result = subprocess.run(
        ["python", "quick_test.py"],
        capture_output=True,
        text=True,
        timeout=60
    )
    
    # Parse results
    for line in result.stdout.split('\n'):
        if "✓" in line and "robots:" in line:
            match = re.search(r'(\d+) robots:', line)
            if match:
                num_robots = match.group(1)
                results[num_robots] += 1
        total_runs += 1
    
    if "ALL TESTS PASSED" in result.stdout:
        print("✓ ALL PASSED")
    else:
        # Show which failed
        failed = []
        for line in result.stdout.split('\n'):
            if "✗" in line and "robots:" in line:
                match = re.search(r'(\d+) robots:', line)
                if match:
                    failed.append(match.group(1))
        print(f"✗ Failed: {', '.join(failed)}")

print(f"\n{'='*60}")
print(f"STABILITY RESULTS OVER {RUNS} RUNS:")
print(f"{'='*60}")
for num_robots in ["4", "5", "6", "8"]:
    success_rate = (results[num_robots] / RUNS) * 100
    print(f"{num_robots} robots: {results[num_robots]}/{RUNS} ({success_rate:.0f}%)")

total_success = sum(results.values())
total_tests = RUNS * 4
overall_rate = (total_success / total_tests) * 100
print(f"\nOverall: {total_success}/{total_tests} ({overall_rate:.0f}%)")
