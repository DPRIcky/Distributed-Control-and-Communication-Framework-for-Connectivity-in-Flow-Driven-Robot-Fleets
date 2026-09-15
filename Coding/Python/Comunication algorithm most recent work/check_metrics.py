import json

data = json.load(open('experiments/comprehensive_results_with_critical_edge_20260224_220645.json'))

print("="*80)
print("CED METRICS BY SCENARIO")
print("="*80)

ced = [x for x in data if x['method'] == 'CriticalEdgeDetection']
for r in ced:
    print(f"{r['scenario']}")
    print(f"  Mean Control Effort:  {r['mean_control_effort']:.2f}")
    print(f"  Convergence Steps:    {r.get('convergence_steps', 'N/A')}")
    print(f"  Edge Reduction:       {r['edge_reduction_pct']:.2f}%")
    print()

print("\n" + "="*80)
print("COMPARISON: MEAN CONTROL EFFORT BY METHOD (per-step)")
print("="*80)

methods = set(x['method'] for x in data)
import numpy as np

# Check what fields are available
print("\nFields in first result:")
print(list(data[0].keys()))

for method in sorted(methods):
    method_data = [x for x in data if x['method'] == method]
    mean_efforts = [x.get('mean_control_effort') for x in method_data]
    steps = [x.get('steps_executed', x.get('convergence_steps', 0)) for x in method_data]
    
    print(f"\n{method:30s}")
    print(f"  Mean Control Effort: {np.mean(mean_efforts):8.2f} ± {np.std(mean_efforts):8.2f}")
    print(f"  Steps Executed:      {np.mean(steps):8.1f} ± {np.std(steps):8.1f} steps")
    print(f"  Control per 100 steps: {np.mean(mean_efforts)/np.mean(steps)*100:8.2f}")

print("\n" + "="*80)
print("CED DETAILED BREAKDOWN (Scenario D showing high control effort)")
print("="*80)

ced_d = [x for x in data if x['method'] == 'CriticalEdgeDetection' and x['scenario'] == 'D']
for i, r in enumerate(ced_d, 1):
    print(f"Trial {i}:")
    print(f"  Mean Control Effort:  {r['mean_control_effort']:.2f}")
    print(f"  Total Control Effort: {r['total_control_effort']:.2f}")
    print(f"  Steps Executed:       {r['steps_executed']}")
    print(f"  Initial Edges:        {r['initial_edges']}")
    print(f"  Final Edges:          {r['final_edges']}")
    print(f"  Edge Reduction:       {r['edge_reduction_pct']:.2f}%")
    print()
