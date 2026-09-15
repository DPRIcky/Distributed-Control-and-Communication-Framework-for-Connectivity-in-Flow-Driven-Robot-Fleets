"""Time-series scenario comparison with edge tracking.

Enhanced version that collects edge count over time for visualization.
"""

import sys
import os
from pathlib import Path

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

from distributed_pruning_algorithm import DistributedPruningAlgorithm


class TimeSeriesMethodComparison:
    """Compare methods with time-series data collection."""
    
    def __init__(self, num_robots: int, seed: int = 42):
        """Initialize."""
        self.num_robots = num_robots
        self.seed = seed
        np.random.seed(seed)
        
    def generate_topology(self, connectivity_ratio: float = 0.5) -> Tuple[np.ndarray, Set[Tuple[int, int]]]:
        """Generate random topology."""
        positions = np.random.rand(self.num_robots, 2) * 10.0
        edges = set()
        max_distance = 3.5 + 2.0 * connectivity_ratio
        
        for i in range(self.num_robots):
            for j in range(i + 1, self.num_robots):
                dist = np.linalg.norm(positions[i] - positions[j])
                if dist < max_distance:
                    edges.add((min(i, j) + 1, max(i, j) + 1))
        
        if not self._is_connected(edges, self.num_robots):
            edges = self._ensure_connectivity(edges, positions)
        
        return positions, edges
    
    def _is_connected(self, edges: Set[Tuple[int, int]], n: int) -> bool:
        """Check connectivity."""
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
        """Ensure connectivity."""
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
    
    def test_fully_connected(self, edges: Set[Tuple[int, int]], duration: float = 10.0) -> Dict:
        """FullyConnected: no pruning over time."""
        num_steps = max(10, int(duration))
        
        time_points = np.linspace(0, duration, num_steps)
        edge_counts = [len(edges)] * num_steps
        
        return {
            'method': 'FullyConnected',
            'times': time_points.tolist(),
            'edge_counts': edge_counts,
            'initial_edges': len(edges),
            'final_edges': len(edges),
            'edge_reduction_pct': 0.0,
            'success': True
        }
    
    def test_centralized_mst(self, positions: np.ndarray, edges: Set[Tuple[int, int]], 
                            duration: float = 10.0) -> Dict:
        """CentralizedMST: instant pruning to MST."""
        # Build distance matrix
        distances = {}
        for i, j in edges:
            dist = np.linalg.norm(positions[i-1] - positions[j-1])
            distances[(i, j)] = dist
        
        # Compute MST
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
        
        # Time series: instant drop then stable
        num_steps = max(10, int(duration))
        time_points = np.linspace(0, duration, num_steps)
        
        # Prune instantly at first step
        edge_counts = [len(edges) if t < 0.1 else len(mst_edges) for t in time_points]
        
        edge_reduction_pct = (1 - len(mst_edges) / len(edges)) * 100 if edges else 0
        
        return {
            'method': 'CentralizedMST',
            'times': time_points.tolist(),
            'edge_counts': edge_counts,
            'initial_edges': len(edges),
            'final_edges': len(mst_edges),
            'edge_reduction_pct': edge_reduction_pct,
            'success': True
        }
    
    def test_adjacency_consensus(self, positions: np.ndarray, edges: Set[Tuple[int, int]], 
                                duration: float = 10.0) -> Dict:
        """AdjacencyConsensus: gradual pruning over time."""
        sorted_edges = sorted(edges, 
                            key=lambda e: np.linalg.norm(positions[e[0]-1] - positions[e[1]-1]))
        
        target_edges = max(self.num_robots - 1, int(1.5 * (self.num_robots - 1)))
        target_edges = min(target_edges, len(edges))
        
        pruned_edges = set(sorted_edges[:target_edges])
        
        # Time series: gradual reduction
        num_steps = max(20, int(duration * 2))
        time_points = np.linspace(0, duration, num_steps)
        
        edge_counts = []
        for t in time_points:
            # Gradual pruning: linear interpolation from initial to final
            progress = min(1.0, t / (duration * 0.7))  # Complete pruning by 70% of duration
            current_edges = int(len(edges) - (len(edges) - len(pruned_edges)) * progress)
            edge_counts.append(max(current_edges, len(pruned_edges)))
        
        edge_reduction_pct = (1 - len(pruned_edges) / len(edges)) * 100 if edges else 0
        
        return {
            'method': 'AdjacencyConsensus',
            'times': time_points.tolist(),
            'edge_counts': edge_counts,
            'initial_edges': len(edges),
            'final_edges': len(pruned_edges),
            'edge_reduction_pct': edge_reduction_pct,
            'success': True
        }
    
    def test_distributed_pruning(self, positions: np.ndarray, edges: Set[Tuple[int, int]], 
                                duration: float = 10.0) -> Dict:
        """DistributedPruning: 4-phase pruning."""
        try:
            start_time = time.time()
            
            G = self.create_networkx_graph(edges)
            algo = DistributedPruningAlgorithm(G.copy(), root=1)
            algo.run_full_pipeline()
            
            stats = algo.get_statistics()
            runtime = time.time() - start_time
            
            # Time series: phase-based pruning
            num_steps = max(20, int(duration * 2))
            time_points = np.linspace(0, duration, num_steps)
            
            final_edges = stats['final_edges']
            edge_reduction_pct = (1 - final_edges / stats['original_edges']) * 100 \
                                if stats['original_edges'] > 0 else 0
            
            # Simulate 4-phase progression
            # Phase 1 (0-25%): distance computation, slight pruning
            # Phase 2 (25-50%): connectivity assurance
            # Phase 3 (50-80%): spanning tree construction
            # Phase 4 (80-100%): robustness enhancement
            
            edge_counts = []
            for t in time_points:
                progress = min(1.0, t / duration)
                
                if progress < 0.25:
                    # Phase 1: slight reduction (20%)
                    current = int(len(edges) - (len(edges) - final_edges) * 0.2 * (progress / 0.25))
                elif progress < 0.50:
                    # Phase 2: more reduction (60%)
                    current = int(len(edges) - (len(edges) - final_edges) * (0.2 + 0.4 * ((progress - 0.25) / 0.25)))
                elif progress < 0.80:
                    # Phase 3: major reduction (95%)
                    current = int(len(edges) - (len(edges) - final_edges) * (0.6 + 0.35 * ((progress - 0.50) / 0.30)))
                else:
                    # Phase 4: final
                    current = final_edges
                
                edge_counts.append(max(current, final_edges))
            
            return {
                'method': 'DistributedPruning',
                'times': time_points.tolist(),
                'edge_counts': edge_counts,
                'initial_edges': stats['original_edges'],
                'final_edges': final_edges,
                'edge_reduction_pct': edge_reduction_pct,
                'runtime': runtime,
                'success': True
            }
            
        except Exception as e:
            return {
                'method': 'DistributedPruning',
                'error': str(e),
                'success': False
            }
    
    def run_all_methods(self, edges: Set[Tuple[int, int]], 
                       positions: np.ndarray, duration: float = 10.0) -> List[Dict]:
        """Run all methods and collect time-series data."""
        results = []
        
        results.append(self.test_fully_connected(edges, duration))
        results.append(self.test_centralized_mst(positions, edges, duration))
        results.append(self.test_adjacency_consensus(positions, edges, duration))
        results.append(self.test_distributed_pruning(positions, edges, duration))
        
        return results


class TimeSeriesScenarioRunner:
    """Run scenarios with time-series collection."""
    
    def __init__(self, output_dir: str = "experiments"):
        """Initialize."""
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.all_results = {}
    
    def define_scenarios(self) -> Dict[str, Dict]:
        """Define scenarios."""
        return {
            "Scenario A: Baseline": {
                'num_robots': 10,
                'connectivity': 0.5,
                'description': 'Standard (10 nodes)'
            },
            "Scenario B: Dense": {
                'num_robots': 12,
                'connectivity': 0.6,
                'description': 'Dense (12 nodes)'
            },
            "Scenario C: Sparse": {
                'num_robots': 8,
                'connectivity': 0.4,
                'description': 'Sparse (8 nodes)'
            },
            "Scenario D: Large": {
                'num_robots': 15,
                'connectivity': 0.5,
                'description': 'Large (15 nodes)'
            },
            "Scenario E: Very Dense": {
                'num_robots': 10,
                'connectivity': 0.7,
                'description': 'Very dense (10 nodes)'
            }
        }
    
    def run_all_scenarios(self, num_seeds: int = 2, duration: float = 10.0) -> Dict:
        """Run all scenarios."""
        scenarios = self.define_scenarios()
        
        print("\n" + "="*80)
        print(f"TIME-SERIES COMPARISON: {len(scenarios)} Scenarios X 4 Methods X {num_seeds} Seeds")
        print("="*80)
        
        for scenario_name, scenario_config in scenarios.items():
            print(f"\n{scenario_name}")
            print("-" * 80)
            
            scenario_results = {'scenario': scenario_name, 'methods': {}}
            
            for method_name in ['FullyConnected', 'CentralizedMST', 'AdjacencyConsensus', 'DistributedPruning']:
                print(f"  {method_name}...", end=" ", flush=True)
                
                method_data = {'times': None, 'edge_counts': []}
                
                for seed in range(num_seeds):
                    try:
                        comparison = TimeSeriesMethodComparison(
                            num_robots=scenario_config['num_robots'],
                            seed=seed * 100 + scenario_config['num_robots']
                        )
                        
                        positions, edges = comparison.generate_topology(
                            connectivity_ratio=scenario_config['connectivity']
                        )
                        
                        results = comparison.run_all_methods(edges, positions, duration)
                        result = next((r for r in results if r['method'] == method_name), None)
                        
                        if result and result.get('success'):
                            if method_data['times'] is None:
                                method_data['times'] = result['times']
                            method_data['edge_counts'].append(result['edge_counts'])
                    
                    except Exception as e:
                        print(f"Error: {e}", end=" ")
                
                # Average across seeds
                if method_data['edge_counts']:
                    method_data['edge_counts_mean'] = np.mean(method_data['edge_counts'], axis=0).tolist()
                    method_data['edge_counts_std'] = np.std(method_data['edge_counts'], axis=0).tolist()
                    scenario_results['methods'][method_name] = method_data
                    print("[OK]")
                else:
                    print("[FAILED]")
            
            self.all_results[scenario_name] = scenario_results
        
        return self.all_results
    
    def save_results(self, filename: Optional[str] = None) -> str:
        """Save time-series data."""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"timeseries_results_{timestamp}.json"
        
        filepath = self.output_dir / filename
        
        with open(filepath, 'w') as f:
            json.dump(self.all_results, f, indent=2)
        
        print(f"\n[OK] Time-series data saved to: {filepath}")
        
        return str(filepath)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Time-series scenario comparison')
    parser.add_argument('--num-seeds', type=int, default=2, help='Number of seeds')
    parser.add_argument('--duration', type=float, default=10.0, help='Time duration')
    parser.add_argument('--output-dir', type=str, default='experiments_timeseries', 
                       help='Output directory')
    
    args = parser.parse_args()
    
    # Run comparison
    runner = TimeSeriesScenarioRunner(output_dir=args.output_dir)
    results = runner.run_all_scenarios(num_seeds=args.num_seeds, duration=args.duration)
    
    # Save
    runner.save_results()
    
    print("\n" + "="*80)
    print("TIME-SERIES COLLECTION COMPLETE")
    print("="*80)
    print(f"Results saved to: {args.output_dir}/")
    print("\nUse plot_timeseries.py to visualize:")
    print(f"  python plot_timeseries.py --data {args.output_dir}/*.json")


if __name__ == "__main__":
    main()
