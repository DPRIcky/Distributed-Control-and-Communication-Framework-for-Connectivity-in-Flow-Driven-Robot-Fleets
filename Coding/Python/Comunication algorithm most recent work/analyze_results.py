import json
import numpy as np

# Load batch results
with open('results/batch_results_20260202_102206.json', 'r') as f:
    data = json.load(f)

results = data['results']
print(f"Total runs: {len(results)}")
print(f"Success rate: {sum(1 for r in results if r['success'])/len(results)*100:.1f}%")

# Group by method
methods = {}
for r in results:
    methods.setdefault(r['method'], []).append(r)

print("\nBy Method:")
for method, runs in methods.items():
    successful = [r for r in runs if r['success']]
    if successful:
        print(f"\n{method.upper()}:")
        print(f"  Success: {len(successful)}/{len(runs)} ({len(successful)/len(runs)*100:.1f}%)")
        print(f"  Avg λ2_min: {np.mean([r['min_lambda2'] for r in successful]):.3f} ± {np.std([r['min_lambda2'] for r in successful]):.3f}")
        print(f"  Avg edges pruned: {np.mean([r['pruning_events'] for r in successful]):.1f} ± {np.std([r['pruning_events'] for r in successful]):.1f}")
        print(f"  Avg edge reduction: {np.mean([r['edge_reduction_pct'] for r in successful]):.1f}%")
        print(f"  Graph disconnections: {sum(r['graph_disconnections'] for r in successful)}")
        print(f"  Avg goal distance: {np.mean([r['goal_distance'] for r in successful]):.2f}m")

# Group by scenario
print("\n\nBy Scenario:")
scenarios = {}
for r in results:
    scenarios.setdefault(r['scenario'], []).append(r)

for scenario, runs in sorted(scenarios.items()):
    successful = [r for r in runs if r['success']]
    print(f"\nScenario {scenario}:")
    print(f"  Success: {len(successful)}/{len(runs)} ({len(successful)/len(runs)*100:.1f}%)")
    if successful:
        print(f"  Avg pruning events: {np.mean([r['pruning_events'] for r in successful]):.1f}")
        print(f"  Avg final edges: {np.mean([r['final_edges'] for r in successful]):.1f}")
