#!/usr/bin/env python
"""Fix plot_4 function to have only Scenario E"""

with open('generate_all_plots.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

# Find function boundaries
start_idx = next(i for i, line in enumerate(lines) if 'def plot_4_control_effort' in line)
end_idx = next(i for i in range(start_idx+1, len(lines)) if lines[i].startswith('def '))

# New function with only Scenario E
new_function = """def plot_4_control_effort(results: List[Dict], output_dir: Path):
    \"\"\"Plot 4: Cumulative control effort over time.\"\"\"
    print("Generating Plot 4: Control Effort...")
    
    methods = ['ConcurrentPruning', 'AdjacencyConsensus', 'FullGraph', 'CentralizedMST']
    
    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    
    # Only Scenario E
    scenario = 'E'
    
    for method in methods:
        trials = [r for r in results if r['method'] == method and r['scenario'] == scenario and r['success']]
        
        if not trials:
            continue
        
        # Check if control_effort_history exists and has data
        valid_trials = []
        for t in trials:
            if 'control_effort_history' in t and len(t['control_effort_history']) > 0:
                # Check if any values are non-zero
                if np.sum(np.abs(t['control_effort_history'])) > 1e-10:
                    valid_trials.append(t['control_effort_history'])
        
        if not valid_trials:
            # If no valid control_effort_history, skip this method
            continue
        
        # Find max length
        max_len = max(len(t) for t in valid_trials)
        
        # Pad all trials to same length
        control_padded = []
        for control_hist in valid_trials:
            if len(control_hist) < max_len:
                padded = np.pad(control_hist, (0, max_len - len(control_hist)), mode='edge')
            else:
                padded = np.array(control_hist)
            control_padded.append(padded)
        
        control_padded = np.array(control_padded)
        
        # Compute cumulative sum across time
        cumulative_control = np.cumsum(control_padded, axis=1)
        mean_control = np.mean(cumulative_control, axis=0)
        
        time = np.arange(len(mean_control))
        ax.plot(time, mean_control, label=METHOD_LABELS[method],
               color=METHOD_COLORS[method], linewidth=1.2)
    
    ax.set_xlabel('Time Step')
    ax.set_ylabel('Cumulative Control Effort')
    ax.set_title('Control erffort over time')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9, framealpha=0.9)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_4_control_effort.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_4_control_effort.png")


"""

# Replace the function
new_lines = lines[:start_idx] + [new_function] + lines[end_idx:]

# Write back
with open('generate_all_plots.py', 'w', encoding='utf-8') as f:
    f.writelines(new_lines)

print(f"✓ Successfully replaced plot_4 function (lines {start_idx+1} to {end_idx})")
print("✓ New function has only Scenario E")
