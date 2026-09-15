import json

data = json.load(open('experiments/comprehensive_results_with_critical_edge_20260224_194948.json'))
ced_d = [r for r in data if r['method'] == 'CriticalEdgeDetection' and r['scenario'] == 'D']

print(f'Scenario D (High Robot Count - 20 robots): {len(ced_d)} trials\n')

for i, r in enumerate(ced_d):
    print(f'Trial {i+1}:')
    print(f'  Success: {r["success"]}')
    print(f'  Steps executed: {r["steps_executed"]}')
    print(f'  Initial edges: {r["initial_edges"]}')
    print(f'  Final edges: {r["final_edges"]}')
    print(f'  Edge reduction: {r["edge_reduction_pct"]:.1f}%')
    if 'error' in r and r['error']:
        print(f'  Error: {r["error"]}')
    print()

# Compare with another method for scenario D
print('\n\n--- Comparison with other methods in Scenario D ---\n')
for method in set(r['method'] for r in data if r['scenario'] == 'D'):
    method_d = [r for r in data if r['method'] == method and r['scenario'] == 'D']
    success = sum(1 for r in method_d if r['success'])
    print(f'{method}: {success}/{len(method_d)} successful')
