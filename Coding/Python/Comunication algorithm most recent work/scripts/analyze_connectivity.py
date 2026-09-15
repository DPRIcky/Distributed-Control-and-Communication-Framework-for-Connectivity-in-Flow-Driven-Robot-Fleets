"""Diagnostic tool to analyze connectivity issues."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import json
import numpy as np
import matplotlib.pyplot as plt

def analyze_connectivity_breakdown():
    """Analyze why the graph is disconnecting."""
    
    # Load test data
    with open('test_metrics.json') as f:
        data = json.load(f)
    
    arrays = np.load('test_arrays.npz')
    
    print("\n" + "="*70)
    print("CONNECTIVITY BREAKDOWN ANALYSIS")
    print("="*70)
    
    # Find when graph disconnects
    lambda2_values = np.array(data['lambda2_values'])
    edge_counts = np.array(data['edge_counts'])
    time_series = np.array(data['time_series'])
    
    # Find first disconnection (λ₂ ≈ 0)
    disconnection_threshold = 0.01
    disconnected_steps = np.where(lambda2_values < disconnection_threshold)[0]
    
    if len(disconnected_steps) > 0:
        first_disconnect = disconnected_steps[0]
        print(f"\n⚠️ GRAPH DISCONNECTION DETECTED")
        print(f"   First occurred at step {first_disconnect}, time={time_series[first_disconnect]:.1f}s")
        print(f"   λ₂ = {lambda2_values[first_disconnect]:.6f} (should be > 0.1)")
        print(f"   Edges at that point: {edge_counts[first_disconnect]}")
        print(f"   For 12 robots, minimum connected edges = 11 (spanning tree)")
    else:
        print(f"\n✓ Graph remained connected throughout simulation")
    
    # Analyze edge evolution
    print(f"\n📊 EDGE COUNT EVOLUTION:")
    print(f"   Initial: {edge_counts[0]} edges")
    print(f"   Final: {edge_counts[-1]} edges")
    print(f"   Minimum: {np.min(edge_counts)} edges (at step {np.argmin(edge_counts)})")
    print(f"   Minimum for connectivity: 11 edges (spanning tree for 12 robots)")
    
    # Check pruning vs natural breaks
    pruning_events = data['pruning_events']
    edge_decrease = edge_counts[0] - edge_counts[-1]
    
    print(f"\n📉 EDGE LOSS BREAKDOWN:")
    print(f"   Total edges lost: {edge_decrease}")
    print(f"   Controlled pruning events: {len(pruning_events)}")
    print(f"   Natural breaks (robots drifting): {edge_decrease - len(pruning_events)}")
    
    if edge_decrease - len(pruning_events) > 0:
        print(f"\n⚠️ WARNING: {edge_decrease - len(pruning_events)} edges broke naturally!")
        print(f"   This indicates CBF connectivity control is not strong enough.")
        print(f"   Robots are drifting beyond communication radius.")
    
    # Analyze connectivity violations
    conn_violations = np.array(data['connectivity_violations'])
    print(f"\n🔗 CONNECTIVITY VIOLATIONS:")
    print(f"   Total violations: {sum(conn_violations)}")
    print(f"   Steps with violations: {np.sum(conn_violations > 0)}/{len(conn_violations)}")
    
    # Create diagnostic plot
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    
    # Plot 1: Edge count and λ₂
    ax = axes[0]
    ax.plot(time_series, edge_counts, 'b-', linewidth=2, label='Edge count')
    ax.axhline(y=11, color='r', linestyle='--', linewidth=2, label='Min for connectivity (N-1)')
    ax.set_ylabel('Edge Count', color='b')
    ax.tick_params(axis='y', labelcolor='b')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper left')
    
    ax2 = ax.twinx()
    ax2.plot(time_series, lambda2_values, 'g-', linewidth=2, label='λ₂')
    ax2.axhline(y=0.1, color='orange', linestyle='--', linewidth=2, label='λ₂ threshold')
    ax2.set_ylabel('λ₂ (Algebraic Connectivity)', color='g')
    ax2.tick_params(axis='y', labelcolor='g')
    ax2.legend(loc='upper right')
    ax.set_title('Edge Count vs Algebraic Connectivity')
    
    # Plot 2: Connectivity violations
    ax = axes[1]
    ax.bar(time_series, conn_violations, color='red', alpha=0.6)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Connectivity Violations')
    ax.set_title('Edges Exceeding Communication Radius')
    ax.grid(True, alpha=0.3)
    
    # Plot 3: Min distance (safety)
    ax = axes[2]
    min_distances = np.array(data['min_distances'])
    ax.plot(time_series, min_distances, 'purple', linewidth=2)
    ax.axhline(y=0.6, color='r', linestyle='--', linewidth=2, label='Safety threshold (d_min)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Minimum Inter-Robot Distance (m)')
    ax.set_title('Collision Avoidance')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('connectivity_diagnostic.png', dpi=150)
    print(f"\n💾 Diagnostic plot saved to: connectivity_diagnostic.png")
    
    # Recommendations
    print(f"\n" + "="*70)
    print("🔧 RECOMMENDATIONS:")
    print("="*70)
    
    if len(disconnected_steps) > 0:
        print("\n1. ⚠️ CRITICAL: Graph is disconnecting")
        print("   → Increase CBF connectivity gain (k_conn in ControlConfig)")
        print("   → Reduce communication radius requirement")
        print("   → Ensure pruning algorithm maintains spanning tree")
    
    if sum(conn_violations) > 0:
        print("\n2. ⚠️ Connectivity constraints being violated")
        print("   → Increase CBF activation threshold (closer to R_max)")
        print("   → Increase control authority limit")
    
    if edge_decrease - len(pruning_events) > edge_decrease * 0.5:
        print("\n3. ⚠️ Too many natural edge breaks (not controlled pruning)")
        print("   → This defeats the purpose of the pruning algorithm")
        print("   → CBF should prevent edges from breaking naturally")
    
    print("\n" + "="*70 + "\n")

if __name__ == "__main__":
    analyze_connectivity_breakdown()
