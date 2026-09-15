"""Comprehensive 5-Scenario Comparison with DistributedPruningAlgorithm.

Compares 4 methods across 5 challenging scenarios:
1. FullyConnected (baseline - no pruning)
2. CentralizedMST (oracle MST computation)
3. AdjacencyConsensus (distributed consensus pruning)
4. DistributedPruning (4-phase algorithm from distributed_pruning_algorithm.py)

Uses DistributedPruningAlgorithm as the primary algorithm for experiments.
"""

import sys
import os
from pathlib import Path

# Add parent directory to path
base_path = Path(__file__).parent.parent.parent
sys.path.insert(0, str(base_path))
sys.path.insert(0, str(base_path / "critical_edge_detection_and_pruning_update" / "scripts"))

import json
import time
import argparse
import numpy as np
import networkx as nx
from datetime import datetime
from typing import Dict, List, Tuple, Optional, Set
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

from distributed_pruning_algorithm import DistributedPruningAlgorithm


class MethodComparison:
    """Compare different pruning methods on same topology."""
    
    def __init__(self, num_robots: int, seed: int = 42):
        """Initialize comparison.
        
        Args:
            num_robots: Number of robots/nodes
            seed: Random seed
        """
        self.num_robots = num_robots
        self.seed = seed
        np.random.seed(seed)
        
    def generate_topology(self, connectivity_ratio: float = 0.5) -> Tuple[np.ndarray, Set[Tuple[int, int]]]:
        """Generate random graph topology.
        
        Args:
            connectivity_ratio: Ratio of edges to maximum (0-1)
            
        Returns:
            (positions, edges) tuple
        """
        # Generate random positions
        positions = np.random.rand(self.num_robots, 2) * 10.0
        
        # Create edges based on distance threshold
        edges = set()
        max_distance = 3.5 + 2.0 * connectivity_ratio
        
        for i in range(self.num_robots):
            for j in range(i + 1, self.num_robots):
                dist = np.linalg.norm(positions[i] - positions[j])
                if dist < max_distance:
                    edges.add((min(i, j) + 1, max(i, j) + 1))  # 1-based indexing for algorithm
        
        # Ensure connectivity
        if not self._is_connected(edges, self.num_robots):
            edges = self._ensure_connectivity(edges, positions)
        
        return positions, edges
    
    def _is_connected(self, edges: Set[Tuple[int, int]], n: int) -> bool:
        """Check if graph is connected."""
        if not edges:
            return n <= 1
        
        adj = {i: set() for i in range(1, n + 1)}
        for i, j in edges:
            adj[i].add(j)
            adj[j].add(i)
        
        visited = set([1])
        queue = [1]
        
        while queue:
            node = queue.pop(0)
            for neighbor in adj[node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        
        return len(visited) == n
    
    def _ensure_connectivity(self, edges: Set[Tuple[int, int]], positions: np.ndarray) -> Set[Tuple[int, int]]:
        """Ensure graph is connected by adding MST edges."""
        # Build MST using Kruskal's algorithm on distances
        all_edges = []
        for i in range(self.num_robots):
            for j in range(i + 1, self.num_robots):
                dist = np.linalg.norm(positions[i] - positions[j])
                all_edges.append((dist, (min(i, j) + 1, max(i, j) + 1)))
        
        all_edges.sort()
        
        parent = {i: i for i in range(1, self.num_robots + 1)}
        
        def find(x):
            if parent[x] != x:
                parent[x] = find(parent[x])
            return parent[x]
        
        def union(x, y):
            px, py = find(x), find(y)
            if px != py:
                parent[px] = py
                return True
            return False
        
        mst_edges = set(edges)
        for dist, (i, j) in all_edges:
            if union(i, j):
                mst_edges.add((i, j))
        
        return mst_edges
    
    def create_networkx_graph(self, edges: Set[Tuple[int, int]]) -> nx.Graph:
        """Create NetworkX graph."""
        G = nx.Graph()
        G.add_nodes_from(range(1, self.num_robots + 1))
        G.add_edges_from(edges)
        return G
    
    def test_fully_connected(self, edges: Set[Tuple[int, int]]) -> Dict:
        """Test baseline: maintain all edges (no pruning)."""
        return {
            'method': 'FullyConnected',
            'initial_edges': len(edges),
            'final_edges': len(edges),
            'edge_reduction_pct': 0.0,
            'pruning_events': 0,
            'runtime': 0.0,
            'success': True
        }
    
    def test_centralized_mst(self, positions: np.ndarray, edges: Set[Tuple[int, int]]) -> Dict:
        """Test oracle baseline: compute MST."""
        # Build distance matrix
        distances = {}
        for i, j in edges:
            dist = np.linalg.norm(positions[i-1] - positions[j-1])
            distances[(i, j)] = dist
        
        # Compute MST using Kruskal
        sorted_edges = sorted(distances.items(), key=lambda x: x[1])
        
        parent = {i: i for i in range(1, self.num_robots + 1)}
        
        def find(x):
            if parent[x] != x:
                parent[x] = find(parent[x])
            return parent[x]
        
        def union(x, y):
            px, py = find(x), find(y)
            if px != py:
                parent[px] = py
                return True
            return False
        
        mst_edges = set()
        for (i, j), dist in sorted_edges:
            if union(i, j):
                mst_edges.add((i, j))
                if len(mst_edges) == self.num_robots - 1:
                    break
        
        edge_reduction_pct = (1 - len(mst_edges) / len(edges)) * 100 if edges else 0
        
        return {
            'method': 'CentralizedMST',
            'initial_edges': len(edges),
            'final_edges': len(mst_edges),
            'edge_reduction_pct': edge_reduction_pct,
            'pruning_events': len(edges) - len(mst_edges),
            'runtime': 0.0,
            'success': True
        }
    
    def test_adjacency_consensus(self, positions: np.ndarray, edges: Set[Tuple[int, int]]) -> Dict:
        """Test distributed consensus pruning (estimation)."""
        # Simulate conservative pruning by keeping 1.5x MST edges
        sorted_edges = sorted(edges, 
                            key=lambda e: np.linalg.norm(positions[e[0]-1] - positions[e[1]-1]))
        
        target_edges = max(self.num_robots - 1, int(1.5 * (self.num_robots - 1)))
        target_edges = min(target_edges, len(edges))
        
        pruned_edges = set(sorted_edges[:target_edges])
        edge_reduction_pct = (1 - len(pruned_edges) / len(edges)) * 100 if edges else 0
        
        return {
            'method': 'AdjacencyConsensus',
            'initial_edges': len(edges),
            'final_edges': len(pruned_edges),
            'edge_reduction_pct': edge_reduction_pct,
            'pruning_events': len(edges) - len(pruned_edges),
            'runtime': 0.0,
            'success': True
        }
    
    def test_distributed_pruning(self, positions: np.ndarray, edges: Set[Tuple[int, int]]) -> Dict:
        """Test 4-phase distributed pruning algorithm."""
        try:
            start_time = time.time()
            
            # Create NetworkX graph (1-based indexing)
            G = self.create_networkx_graph(edges)
            
            # Run algorithm
            algo = DistributedPruningAlgorithm(G.copy(), root=1)
            algo.run_full_pipeline()
            
            # Get results
            stats = algo.get_statistics()
            runtime = time.time() - start_time
            
            edge_reduction_pct = (1 - stats['final_edges'] / stats['original_edges']) * 100 \
                                if stats['original_edges'] > 0 else 0
            
            return {
                'method': 'DistributedPruning',
                'initial_edges': stats['original_edges'],
                'final_edges': stats['final_edges'],
                'edge_reduction_pct': edge_reduction_pct,
                'pruning_events': stats['original_edges'] - stats['final_edges'],
                'connectivity_maintained': stats['connected'],
                'runtime': runtime,
                'success': True
            }
            
        except Exception as e:
            return {
                'method': 'DistributedPruning',
                'error': str(e),
                'success': False,
                'runtime': 0.0
            }
    
    def run_all_methods(self, edges: Set[Tuple[int, int]], 
                       positions: np.ndarray) -> List[Dict]:
        """Run all methods on same topology."""
        results = []
        
        # FullyConnected
        results.append(self.test_fully_connected(edges))
        
        # CentralizedMST
        results.append(self.test_centralized_mst(positions, edges))
        
        # AdjacencyConsensus
        results.append(self.test_adjacency_consensus(positions, edges))
        
        # DistributedPruning
        results.append(self.test_distributed_pruning(positions, edges))
        
        return results


class ComprehensiveScenarioComparison:
    """Run comprehensive 5-scenario comparison."""
    
    def __init__(self, output_dir: str = "experiments"):
        """Initialize."""
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.results = []
    
    def define_scenarios(self) -> Dict[str, Dict]:
        """Define 5 scenarios."""
        return {
            "Scenario A: Baseline": {
                'num_robots': 10,
                'connectivity': 0.5,
                'description': 'Standard (10 nodes, moderate connectivity)'
            },
            "Scenario B: Dense Network": {
                'num_robots': 12,
                'connectivity': 0.6,
                'description': 'Dense (12 nodes, high connectivity)'
            },
            "Scenario C: Sparse Network": {
                'num_robots': 8,
                'connectivity': 0.4,
                'description': 'Sparse (8 nodes, low redundancy)'
            },
            "Scenario D: Large Network": {
                'num_robots': 15,
                'connectivity': 0.5,
                'description': 'Large (15 nodes, moderate connectivity)'
            },
            "Scenario E: Very Dense": {
                'num_robots': 10,
                'connectivity': 0.7,
                'description': 'Very dense (10 nodes, high redundancy)'
            }
        }
    
    def run_all_scenarios(self, num_seeds: int = 3) -> List[Dict]:
        """Run all scenarios."""
        scenarios = self.define_scenarios()
        methods = ['FullyConnected', 'CentralizedMST', 'AdjacencyConsensus', 'DistributedPruning']
        
        total_experiments = len(scenarios) * len(methods) * num_seeds
        experiment_count = 0
        
        print("\n" + "="*80)
        print(f"COMPREHENSIVE COMPARISON: {len(scenarios)} Scenarios X {len(methods)} Methods X {num_seeds} Seeds")
        print(f"Total Experiments: {total_experiments}")
        print("="*80)
        
        for scenario_name, scenario_config in scenarios.items():
            print(f"\n{'='*80}")
            print(f"SCENARIO: {scenario_name}")
            print(f"  {scenario_config['description']}")
            print(f"{'='*80}")
            
            for method_name in methods:
                print(f"\n  {method_name}:")
                print("  " + "-"*70)
                
                method_results = []
                
                for seed in range(num_seeds):
                    experiment_count += 1
                    print(f"    [{experiment_count}/{total_experiments}] Seed {seed+1}/{num_seeds}...", end=" ", flush=True)
                    
                    try:
                        # Generate topology
                        comparison = MethodComparison(
                            num_robots=scenario_config['num_robots'],
                            seed=seed * 100 + scenario_config['num_robots']
                        )
                        
                        positions, edges = comparison.generate_topology(
                            connectivity_ratio=scenario_config['connectivity']
                        )
                        
                        # Run method
                        results = comparison.run_all_methods(edges, positions)
                        result = next((r for r in results if r['method'] == method_name), None)
                        
                        if result and result.get('success'):
                            method_results.append(result)
                            print(f"[OK] {result['initial_edges']}->{result['final_edges']} edges")
                        else:
                            print(f"[ERROR]")
                            
                    except Exception as e:
                        print(f"[ERROR] {e}")
                
                # Average across seeds
                if method_results:
                    avg_result = self._average_results(method_results, scenario_name, method_name)
                    self.results.append(avg_result)
        
        return self.results
    
    def _average_results(self, results: List[Dict], scenario: str, method: str) -> Dict:
        """Average results across seeds."""
        avg = {
            'scenario': scenario,
            'method': method,
            'num_seeds': len(results),
            'initial_edges': np.mean([r['initial_edges'] for r in results]),
            'final_edges': np.mean([r['final_edges'] for r in results]),
            'edge_reduction_pct': np.mean([r['edge_reduction_pct'] for r in results]),
            'pruning_events': np.mean([r['pruning_events'] for r in results]),
            'runtime': np.mean([r.get('runtime', 0) for r in results]),
        }
        return avg
    
    def save_results(self, filename: Optional[str] = None) -> str:
        """Save results to JSON."""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"comparison_results_{timestamp}.json"
        
        filepath = self.output_dir / filename
        
        with open(filepath, 'w') as f:
            json.dump(self.results, f, indent=2)
        
        print(f"\n[OK] Results saved to: {filepath}")
        return str(filepath)
    
    def generate_visualizations(self) -> List[str]:
        """Generate comparison plots."""
        if not self.results:
            return []
        
        saved_figures = []
        
        # Plot 1: Edge reduction comparison
        fig = self._create_edge_reduction_plot()
        if fig:
            path = str(self.output_dir / "01_edge_reduction.png")
            fig.savefig(path, dpi=150, bbox_inches='tight')
            saved_figures.append(path)
            plt.close(fig)
        
        # Plot 2: Method comparison across scenarios
        fig = self._create_method_comparison_plot()
        if fig:
            path = str(self.output_dir / "02_method_comparison.png")
            fig.savefig(path, dpi=150, bbox_inches='tight')
            saved_figures.append(path)
            plt.close(fig)
        
        # Plot 3: Summary heatmap
        fig = self._create_summary_heatmap()
        if fig:
            path = str(self.output_dir / "03_summary_heatmap.png")
            fig.savefig(path, dpi=150, bbox_inches='tight')
            saved_figures.append(path)
            plt.close(fig)
        
        print(f"\n[OK] Generated {len(saved_figures)} visualization(s)")
        for path in saved_figures:
            print(f"  - {path}")
        
        return saved_figures
    
    def _create_edge_reduction_plot(self) -> Optional[plt.Figure]:
        """Create edge reduction comparison."""
        if not self.results:
            return None
        
        scenarios = sorted(list(set(r['scenario'] for r in self.results)))
        methods = sorted(list(set(r['method'] for r in self.results)))
        
        fig, axes = plt.subplots(1, len(scenarios), figsize=(15, 5))
        if len(scenarios) == 1:
            axes = [axes]
        
        fig.suptitle('Edge Reduction Comparison Across Scenarios', 
                    fontsize=14, fontweight='bold')
        
        for idx, scenario in enumerate(scenarios):
            ax = axes[idx]
            
            scenario_results = [r for r in self.results if r['scenario'] == scenario]
            edge_reductions = []
            labels = []
            
            for method in methods:
                result = next((r for r in scenario_results if r['method'] == method), None)
                if result:
                    edge_reductions.append(result['edge_reduction_pct'])
                    labels.append(method)
            
            colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#FFA07A']
            bars = ax.bar(range(len(labels)), edge_reductions, color=colors[:len(labels)])
            ax.set_ylabel('Edge Reduction (%)')
            ax.set_title(scenario.split(':')[0], fontsize=10)
            ax.set_xticks(range(len(labels)))
            ax.set_xticklabels(labels, rotation=45, ha='right', fontsize=9)
            ax.set_ylim(0, 100)
            ax.grid(axis='y', alpha=0.3)
        
        plt.tight_layout()
        return fig
    
    def _create_method_comparison_plot(self) -> Optional[plt.Figure]:
        """Create method comparison table."""
        if not self.results:
            return None
        
        fig, ax = plt.subplots(figsize=(12, 8))
        
        methods = sorted(list(set(r['method'] for r in self.results)))
        scenarios = sorted(list(set(r['scenario'] for r in self.results)))
        
        # Build data table
        table_data = [['Method'] + [s.split(':')[0] for s in scenarios]]
        
        for method in methods:
            row = [method]
            for scenario in scenarios:
                result = next((r for r in self.results 
                             if r['method'] == method and r['scenario'] == scenario), None)
                if result:
                    reduction = result['edge_reduction_pct']
                    row.append(f'{reduction:.1f}%')
                else:
                    row.append('-')
            table_data.append(row)
        
        table = ax.table(cellText=table_data, cellLoc='center', loc='center',
                        colWidths=[0.15] + [0.18]*len(scenarios))
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1, 2.5)
        
        # Style header row
        for i in range(len(scenarios) + 1):
            table[(0, i)].set_facecolor('#40466e')
            table[(0, i)].set_text_props(weight='bold', color='white')
        
        # Style method column
        for i in range(1, len(methods) + 1):
            table[(i, 0)].set_facecolor('#E8E8E8')
            table[(i, 0)].set_text_props(weight='bold')
        
        ax.axis('off')
        ax.set_title('Edge Reduction Comparison (%)', fontsize=14, fontweight='bold', pad=20)
        
        plt.tight_layout()
        return fig
    
    def _create_summary_heatmap(self) -> Optional[plt.Figure]:
        """Create summary heatmap."""
        if not self.results:
            return None
        
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle('Summary Statistics Heatmap', fontsize=14, fontweight='bold')
        
        methods = sorted(list(set(r['method'] for r in self.results)))
        scenarios = sorted(list(set(r['scenario'] for r in self.results)))
        
        # Heatmap 1: Edge Reduction
        ax = axes[0]
        matrix = np.zeros((len(scenarios), len(methods)))
        
        for i, scenario in enumerate(scenarios):
            for j, method in enumerate(methods):
                result = next((r for r in self.results 
                             if r['scenario'] == scenario and r['method'] == method), None)
                if result:
                    matrix[i, j] = result['edge_reduction_pct']
        
        im = ax.imshow(matrix, cmap='RdYlGn', vmin=0, vmax=100, aspect='auto')
        ax.set_xticks(range(len(methods)))
        ax.set_yticks(range(len(scenarios)))
        ax.set_xticklabels(methods, rotation=45, ha='right')
        ax.set_yticklabels([s.split(':')[0] for s in scenarios], fontsize=9)
        ax.set_title('Edge Reduction (%)', fontweight='bold')
        
        # Add values
        for i in range(len(scenarios)):
            for j in range(len(methods)):
                text = ax.text(j, i, f'{matrix[i, j]:.0f}',
                             ha="center", va="center", color="black", fontsize=9, fontweight='bold')
        
        plt.colorbar(im, ax=ax)
        
        # Heatmap 2: Pruning Events
        ax = axes[1]
        matrix = np.zeros((len(scenarios), len(methods)))
        
        for i, scenario in enumerate(scenarios):
            for j, method in enumerate(methods):
                result = next((r for r in self.results 
                             if r['scenario'] == scenario and r['method'] == method), None)
                if result:
                    matrix[i, j] = result['pruning_events']
        
        im = ax.imshow(matrix, cmap='Blues', aspect='auto')
        ax.set_xticks(range(len(methods)))
        ax.set_yticks(range(len(scenarios)))
        ax.set_xticklabels(methods, rotation=45, ha='right')
        ax.set_yticklabels([s.split(':')[0] for s in scenarios], fontsize=9)
        ax.set_title('Pruning Events', fontweight='bold')
        
        # Add values
        for i in range(len(scenarios)):
            for j in range(len(methods)):
                text = ax.text(j, i, f'{int(matrix[i, j])}',
                             ha="center", va="center", color="black", fontsize=9, fontweight='bold')
        
        plt.colorbar(im, ax=ax)
        
        plt.tight_layout()
        return fig


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Comprehensive scenario comparison')
    parser.add_argument('--num-seeds', type=int, default=3, help='Number of seeds')
    parser.add_argument('--output-dir', type=str, default='experiments', help='Output directory')
    
    args = parser.parse_args()
    
    # Run comparison
    runner = ComprehensiveScenarioComparison(output_dir=args.output_dir)
    results = runner.run_all_scenarios(num_seeds=args.num_seeds)
    
    # Save and visualize
    runner.save_results()
    runner.generate_visualizations()
    
    print("\n" + "="*80)
    print("EXPERIMENT COMPLETE")
    print("="*80)
    

if __name__ == "__main__":
    main()
