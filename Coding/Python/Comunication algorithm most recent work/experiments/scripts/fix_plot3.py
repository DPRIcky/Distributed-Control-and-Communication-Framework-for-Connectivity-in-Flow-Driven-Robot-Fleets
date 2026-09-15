#!/usr/bin/env python
"""Fix plot_3 function to have only 2 subplots"""

with open('generate_all_plots.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

# Find function boundaries
start_idx = next(i for i, line in enumerate(lines) if 'def plot_3_lambda2' in line)
end_idx = next(i for i in range(start_idx+1, len(lines)) if lines[i].startswith('def '))

# New function with 2 subplots
new_function = """def plot_3_lambda2_evolution_and_distribution(results: List[Dict], output_dir: Path):
    \"\"\"Plot 3: Lambda2 evolution and distribution.\"\"\"
    print("Generating Plot 3: Lambda2 Analysis...")
    
    scenarios = ['A', 'B', 'C', 'D', 'E']
    methods = ['ConcurrentPruning', 'AdjacencyConsensus', 'FullGraph', 'CentralizedMST']
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # Subplot 1: Lambda2 evolution for Scenario A only
    ax = axes[0]
    scenario = 'A'
    
    for method in methods:
        trials = [r for r in results if r['method'] == method and r['scenario'] == scenario and r['success']]
        
        if not trials:
            continue
        
        max_len = max(len(t['lambda2_history']) for t in trials)
        lambda2_padded = np.array([
            np.pad(t['lambda2_history'], (0, max_len - len(t['lambda2_history'])), 
                  mode='edge') for t in trials
        ])
        mean_lambda2 = np.mean(lambda2_padded, axis=0)
        std_lambda2 = np.std(lambda2_padded, axis=0)
        
        time = np.arange(len(mean_lambda2))
        ax.plot(time, mean_lambda2, label=METHOD_LABELS[method],
               color=METHOD_COLORS[method], linewidth=1.2)
    
    ax.axhline(y=0.2, color='red', linestyle='--', linewidth=1, alpha=0.6, label='Safety Threshold')
    ax.set_xlabel('Time Step')
    ax.set_ylabel(r'$\\lambda_2$')
    ax.set_title(f'Scenario {scenario}: {SCENARIO_LABELS[scenario]}')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, framealpha=0.9)
    
    # Subplot 2: Lambda2 distribution (bar plot)
    ax_dist = axes[1]
    
    # Prepare data for bar plot
    means = []
    stds = []
    
    for method in methods:
        method_results = [r for r in results if r['method'] == method and r['success']]
        min_lambda2_values = [r['min_lambda2'] for r in method_results]
        means.append(np.mean(min_lambda2_values))
        stds.append(np.std(min_lambda2_values))
    
    # Create bar plot
    x = np.arange(len(methods))
    bars = ax_dist.bar(x, means, 
                       color=[METHOD_COLORS[m] for m in methods],
                       alpha=0.7, edgecolor='black', linewidth=0.8)
    
    # Add safety threshold line
    ax_dist.axhline(y=0.2, color='red', linestyle='--', linewidth=1, alpha=0.6, label='Safety Threshold')
    
    ax_dist.set_xticks(x)
    ax_dist.set_xticklabels([METHOD_LABELS[m] for m in methods], rotation=20, ha='right', fontsize=7)
    ax_dist.set_ylabel(r'Minimum $\\lambda_2$')
    ax_dist.set_title(r'Min $\\lambda_2$ Distribution Across All Scenarios')
    ax_dist.grid(True, alpha=0.3, axis='y')
    ax_dist.legend(fontsize=7)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_3_lambda2_analysis.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_3_lambda2_analysis.png")


"""

# Replace the function
new_lines = lines[:start_idx] + [new_function] + lines[end_idx:]

# Write back
with open('generate_all_plots.py', 'w', encoding='utf-8') as f:
    f.writelines(new_lines)

print(f"✓ Successfully replaced plot_3 function (lines {start_idx+1} to {end_idx})")
print("✓ New function has 2 subplots instead of 6")
