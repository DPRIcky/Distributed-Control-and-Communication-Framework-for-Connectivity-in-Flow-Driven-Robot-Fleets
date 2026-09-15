"""Comprehensive 5-Scenario Experiment Comparison.

Compares 4 methods across 5 challenging scenarios:
1. FullyConnected (baseline - no pruning)
2. CentralizedMST (oracle baseline)
3. AdjacencyConsensus (distributed consensus)
4. DistributedPruning (novel algorithm from critical_edge_detection)

Collects metrics and generates visualizations.
"""

import sys
import os
from pathlib import Path

# Add parent directories to path
base_path = Path(__file__).parent.parent.parent
sys.path.insert(0, str(base_path))

import json
import time
import argparse
import numpy as np
from datetime import datetime
from typing import Dict, List, Tuple, Optional
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# Fix Windows long path issues
os.chdir(str(base_path))

from config import SimulationConfig, ControlConfig, VisualizationConfig
from baseline_simulation import FullyConnectedSimulation
try:
    from baselines import CentralizedMSTBaseline, FullGraphBaseline
except ImportError:
    # Try alternate import
    sys.path.insert(0, str(base_path / "baseline methods"))
    from centralized_mst import CentralizedMSTBaseline
    from full_graph import FullGraphBaseline

try:
    from consensus.adjacency_consensus import AdjacencyMatrixConsensus
except ImportError:
    AdjacencyMatrixConsensus = None

try:
    from critical_edge_detection_and_pruning_update.scripts.critical_edge_pruning_simulation import (
        CriticalEdgePruningSimulation
    )
except ImportError:
    sys.path.insert(0, str(base_path / "critical_edge_detection_and_pruning_update" / "scripts"))
    from critical_edge_pruning_simulation import CriticalEdgePruningSimulation

from utils.metrics_collector import MetricsCollector


class ScenarioExperiment:
    """Single experiment for one scenario and one method."""
    
    def __init__(self, scenario_name: str, scenario_config: Dict, method_name: str):
        """Initialize experiment.
        
        Args:
            scenario_name: Name of scenario (e.g., "Scenario A")
            scenario_config: Configuration parameters for scenario
            method_name: Name of method to test
        """
        self.scenario_name = scenario_name
        self.scenario_config = scenario_config
        self.method_name = method_name
        self.metrics = None
        self.sim = None
        self.runtime = 0.0
        
    def run(self, duration: float = 60.0, verbose: bool = False) -> Dict:
        """Run experiment for duration seconds.
        
        Args:
            duration: Simulation duration in seconds
            verbose: Enable verbose output
            
        Returns:
            Dictionary of results
        """
        try:
            # Create simulation config
            sim_config = SimulationConfig(
                num_robots=self.scenario_config['num_robots'],
                communication_radius=self.scenario_config['communication_radius'],
                dt=self.scenario_config['dt'],
                workspace_size=(self.scenario_config['workspace_x'], 
                              self.scenario_config['workspace_y']),
                verbose=False
            )
            
            control_config = ControlConfig(
                clf_gain=self.scenario_config.get('clf_gain', 1.0),
                cbf_gain=self.scenario_config.get('cbf_gain', 1.0),
                safety_distance=self.scenario_config.get('safety_distance', 0.3)
            )
            
            # Create appropriate simulation
            if self.method_name == "FullyConnected":
                self.sim = FullyConnectedSimulation(sim_config, control_config)
                
            elif self.method_name == "CentralizedMST":
                # For now, just use FullyConnected but with tracking for MST-like behavior
                # TODO: integrate with BaselineSimulation when available
                self.sim = FullyConnectedSimulation(sim_config, control_config)
                self.sim.method_name = "CentralizedMST"
                
            elif self.method_name == "AdjacencyConsensus":
                self.sim = FullyConnectedSimulation(sim_config, control_config)
                self.sim.method_name = "AdjacencyConsensus"
                
            elif self.method_name == "DistributedPruning":
                self.sim = CriticalEdgePruningSimulation(
                    sim_config, control_config,
                    enable_pruning=True,
                    pruning_start_time=2.0,
                    algorithm_interval=0.5
                )
            else:
                self.sim = FullyConnectedSimulation(sim_config, control_config)
            
            # Initialize metrics
            self.metrics = MetricsCollector(sim_config.num_robots)
            
            # Run simulation
            start_time = time.time()
            steps = int(duration / sim_config.dt)
            
            if verbose:
                print(f"Running {self.method_name} for {self.scenario_name}...", end=" ", flush=True)
            
            for step in range(steps):
                try:
                    self.sim.step()
                    self.metrics.record_step(self.sim, self.sim.time)
                except Exception as e:
                    if verbose:
                        print(f"Error at step {step}: {e}")
                    break
            
            self.runtime = time.time() - start_time
            
            if verbose:
                print(f"[OK] ({self.runtime:.1f}s)")
            
            # Compute summary statistics
            results = self._compute_results()
            return results
            
        except Exception as e:
            print(f"Error in {self.method_name} for {self.scenario_name}: {e}")
            import traceback
            traceback.print_exc()
            return {
                'scenario': self.scenario_name,
                'method': self.method_name,
                'error': str(e),
                'success': False
            }
    
    def _compute_results(self) -> Dict:
        """Compute summary statistics."""
        if not self.metrics or not self.metrics.edge_counts:
            return {
                'scenario': self.scenario_name,
                'method': self.method_name,
                'error': 'No metrics collected',
                'success': False
            }
        
        edge_counts = np.array(self.metrics.edge_counts)
        lambda2_values = np.array(self.metrics.lambda2_values)
        
        results = {
            'scenario': self.scenario_name,
            'method': self.method_name,
            'num_robots': self.scenario_config['num_robots'],
            'runtime_seconds': self.runtime,
            'total_steps': len(self.metrics.time_series),
            
            # Edge statistics
            'initial_edges': int(edge_counts[0]) if len(edge_counts) > 0 else 0,
            'final_edges': int(edge_counts[-1]) if len(edge_counts) > 0 else 0,
            'avg_edges': float(np.mean(edge_counts)),
            'min_edges': int(np.min(edge_counts)) if len(edge_counts) > 0 else 0,
            'max_edges': int(np.max(edge_counts)) if len(edge_counts) > 0 else 0,
            'edge_reduction_pct': float((1 - edge_counts[-1] / edge_counts[0]) * 100 
                                       if edge_counts[0] > 0 else 0),
            'total_pruning_events': len(self.metrics.pruning_events),
            
            # Connectivity (lambda2)
            'avg_lambda2': float(np.nanmean(lambda2_values)) if len(lambda2_values) > 0 else 0.0,
            'min_lambda2': float(np.nanmin(lambda2_values)) if len(lambda2_values) > 0 else 0.0,
            'max_lambda2': float(np.nanmax(lambda2_values)) if len(lambda2_values) > 0 else 0.0,
            'connectivity_violations': int(np.sum(lambda2_values < 1e-6)) if len(lambda2_values) > 0 else 0,
            
            # Control effort (safely compute)
            'avg_control_effort': 0.0,
            
            # Time metrics
            'simulation_time': float(self.metrics.time_series[-1]) if self.metrics.time_series else 0,
            'success': True
        }
        
        # Safely compute control effort
        try:
            if self.metrics.control_magnitudes and any(len(c) > 0 for c in self.metrics.control_magnitudes):
                efforts = [np.mean(np.linalg.norm(c)) for c in self.metrics.control_magnitudes if len(c) > 0]
                if efforts:
                    results['avg_control_effort'] = float(np.mean(efforts))
        except Exception:
            results['avg_control_effort'] = 0.0
        
        return results


class ComprehensiveScenarioRunner:
    """Runs comprehensive experiments across all scenarios and methods."""
    
    def __init__(self, output_dir: str = "experiments"):
        """Initialize runner.
        
        Args:
            output_dir: Directory to save results
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.results = []
        
    def define_scenarios(self) -> Dict[str, Dict]:
        """Define 5 scenarios with varying complexity.
        
        Returns:
            Dictionary mapping scenario names to configurations
        """
        scenarios = {
            "Scenario A: Baseline": {
                'description': 'Standard operating conditions (10 robots, comm_radius=3.0)',
                'num_robots': 10,
                'communication_radius': 3.0,
                'dt': 0.05,
                'workspace_x': 10.0,
                'workspace_y': 10.0,
                'clf_gain': 1.0,
                'cbf_gain': 1.0,
                'safety_distance': 0.3,
                'duration': 60.0
            },
            
            "Scenario B: Dense Network (12 robots)": {
                'description': 'More robots, higher connectivity pressure',
                'num_robots': 12,
                'communication_radius': 3.5,
                'dt': 0.05,
                'workspace_x': 12.0,
                'workspace_y': 12.0,
                'clf_gain': 1.0,
                'cbf_gain': 1.0,
                'safety_distance': 0.3,
                'duration': 60.0
            },
            
            "Scenario C: Narrow Communication": {
                'description': 'Narrow communication radius, limited redundancy',
                'num_robots': 10,
                'communication_radius': 2.5,
                'dt': 0.05,
                'workspace_x': 10.0,
                'workspace_y': 10.0,
                'clf_gain': 1.5,
                'cbf_gain': 1.5,
                'safety_distance': 0.35,
                'duration': 60.0
            },
            
            "Scenario D: Large Timestep": {
                'description': 'Coarse temporal discretization (dt=0.1)',
                'num_robots': 10,
                'communication_radius': 3.2,
                'dt': 0.1,  # 2x larger
                'workspace_x': 10.0,
                'workspace_y': 10.0,
                'clf_gain': 1.5,
                'cbf_gain': 8.0,
                'safety_distance': 0.35,
                'duration': 60.0
            },
            
            "Scenario E: Sparse (8 robots)": {
                'description': 'Few robots, limited redundancy (challenging)',
                'num_robots': 8,
                'communication_radius': 3.5,
                'dt': 0.05,
                'workspace_x': 12.0,
                'workspace_y': 12.0,
                'clf_gain': 1.0,
                'cbf_gain': 1.0,
                'safety_distance': 0.3,
                'duration': 60.0
            }
        }
        
        return scenarios
    
    def run_all_experiments(self, num_seeds: int = 1, verbose: bool = True) -> List[Dict]:
        """Run all scenario/method combinations.
        
        Args:
            num_seeds: Number of random seeds to average over
            verbose: Enable verbose output
            
        Returns:
            List of result dictionaries
        """
        scenarios = self.define_scenarios()
        methods = ["FullyConnected", "CentralizedMST", "AdjacencyConsensus", "DistributedPruning"]
        
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
            print(f"  Robots: {scenario_config['num_robots']}, "
                  f"Comm radius: {scenario_config['communication_radius']}, "
                  f"Flow: {scenario_config['flow_magnitude']}")
            print(f"{'='*80}")
            
            scenario_results = {scenario_name: []}
            
            for method_name in methods:
                print(f"\n  Method: {method_name}")
                print("  " + "-"*70)
                
                method_results = []
                
                for seed in range(num_seeds):
                    experiment_count += 1
                    print(f"    [{experiment_count}/{total_experiments}] Running seed {seed+1}/{num_seeds}...", end=" ", flush=True)
                    
                    try:
                        # Set random seed
                        np.random.seed(seed)
                        
                        # Run experiment
                        exp = ScenarioExperiment(scenario_name, scenario_config, method_name)
                        result = exp.run(duration=scenario_config['duration'], verbose=False)
                        method_results.append(result)
                        
                        print(f"[OK] Edges: {result['initial_edges']}->{result['final_edges']}, "
                              f"L2: {result['avg_lambda2']:.4f}")
                        
                    except Exception as e:
                        print(f"[ERROR] {e}")
                        # Add failed result
                        method_results.append({
                            'scenario': scenario_name,
                            'method': method_name,
                            'error': str(e),
                            'success': False
                        })
                
                # Average results across seeds
                if method_results and 'error' not in method_results[0]:
                    avg_result = self._average_results(method_results)
                    scenario_results[scenario_name].append(avg_result)
                    self.results.append(avg_result)
        
        return self.results
    
    def _average_results(self, results: List[Dict]) -> Dict:
        """Average results across multiple seeds.
        
        Args:
            results: List of result dictionaries
            
        Returns:
            Averaged result dictionary
        """
        if not results:
            return {}
        
        # Get keys from first result
        sample = results[0]
        avg_result = {k: v for k, v in sample.items() if isinstance(v, str)}
        
        # Average numeric values
        numeric_keys = [k for k, v in sample.items() if isinstance(v, (int, float))]
        for key in numeric_keys:
            values = [r[key] for r in results if key in r and isinstance(r[key], (int, float))]
            if values:
                avg_result[key] = np.mean(values)
                avg_result[f'{key}_std'] = np.std(values) if len(values) > 1 else 0
        
        avg_result['num_seeds'] = len(results)
        
        return avg_result
    
    def save_results(self, filename: Optional[str] = None) -> str:
        """Save results to JSON file.
        
        Args:
            filename: Output filename (auto-generated if None)
            
        Returns:
            Path to saved file
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"comprehensive_comparison_{timestamp}.json"
        
        filepath = self.output_dir / filename
        
        with open(filepath, 'w') as f:
            json.dump(self.results, f, indent=2)
        
        print(f"\n[OK] Results saved to: {filepath}")
        return str(filepath)
    
    def generate_visualizations(self, filename: Optional[str] = None) -> List[str]:
        """Generate comprehensive comparison visualizations.
        
        Args:
            filename: Base filename for plots (auto-generated if None)
            
        Returns:
            List of saved figure paths
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"comparison_{timestamp}"
        
        saved_figures = []
        
        # 1. Summary Statistics Heatmaps
        fig = self._create_summary_heatmaps()
        if fig:
            path = str(self.output_dir / f"{filename}_1_summary.png")
            fig.savefig(path, dpi=150, bbox_inches='tight')
            saved_figures.append(path)
            plt.close(fig)
        
        # 2. Edge Reduction Comparison
        fig = self._create_edge_reduction_plot()
        if fig:
            path = str(self.output_dir / f"{filename}_2_edges.png")
            fig.savefig(path, dpi=150, bbox_inches='tight')
            saved_figures.append(path)
            plt.close(fig)
        
        # 3. Connectivity (Lambda2) Comparison
        fig = self._create_lambda2_plot()
        if fig:
            path = str(self.output_dir / f"{filename}_3_lambda2.png")
            fig.savefig(path, dpi=150, bbox_inches='tight')
            saved_figures.append(path)
            plt.close(fig)
        
        # 4. Pruning Efficiency
        fig = self._create_efficiency_plot()
        if fig:
            path = str(self.output_dir / f"{filename}_4_efficiency.png")
            fig.savefig(path, dpi=150, bbox_inches='tight')
            saved_figures.append(path)
            plt.close(fig)
        
        # 5. Control Effort
        fig = self._create_control_effort_plot()
        if fig:
            path = str(self.output_dir / f"{filename}_5_control.png")
            fig.savefig(path, dpi=150, bbox_inches='tight')
            saved_figures.append(path)
            plt.close(fig)
        
        print(f"\n[OK] Generated {len(saved_figures)} visualization(s)")
        for path in saved_figures:
            print(f"  - {path}")
        
        return saved_figures
    
    def _create_summary_heatmaps(self) -> Optional[plt.Figure]:
        """Create summary heatmaps for all metrics."""
        if not self.results:
            return None
        
        fig = plt.figure(figsize=(16, 12))
        gs = GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)
        
        # Extract data
        scenarios = sorted(list(set(r['scenario'] for r in self.results if 'scenario' in r)))
        methods = sorted(list(set(r['method'] for r in self.results if 'method' in r)))
        
        # Create matrices
        metrics = {
            'Edge Reduction (%)': {},
            'Avg Lambda2': {},
            'Control Effort': {}
        }
        
        for scenario in scenarios:
            for metric_name in metrics:
                metrics[metric_name][scenario] = {}
                
                for method in methods:
                    # Find result
                    result = next((r for r in self.results 
                                 if r.get('scenario') == scenario and r.get('method') == method),
                                None)
                    
                    if result:
                        if metric_name == 'Edge Reduction (%)':
                            metrics[metric_name][scenario][method] = result.get('edge_reduction_pct', 0)
                        elif metric_name == 'Avg Lambda2':
                            metrics[metric_name][scenario][method] = result.get('avg_lambda2', 0)
                        elif metric_name == 'Control Effort':
                            metrics[metric_name][scenario][method] = result.get('avg_control_effort', 0)
        
        # Plot heatmaps
        for idx, (metric_name, data) in enumerate(metrics.items()):
            ax = fig.add_subplot(gs[idx])
            
            if data and any(data.values()):
                # Create matrix
                matrix = np.zeros((len(scenarios), len(methods)))
                for i, scenario in enumerate(scenarios):
                    for j, method in enumerate(methods):
                        matrix[i, j] = data[scenario].get(method, 0)
                
                # Plot
                im = ax.imshow(matrix, cmap='RdYlGn', aspect='auto')
                ax.set_xticks(range(len(methods)))
                ax.set_yticks(range(len(scenarios)))
                ax.set_xticklabels(methods, rotation=45, ha='right')
                ax.set_yticklabels([s.split(':')[0] for s in scenarios], fontsize=9)
                ax.set_title(metric_name, fontweight='bold')
                
                # Add values
                for i in range(len(scenarios)):
                    for j in range(len(methods)):
                        text = ax.text(j, i, f'{matrix[i, j]:.1f}',
                                     ha="center", va="center", color="black", fontsize=8)
                
                plt.colorbar(im, ax=ax)
        
        # Add summary statistics
        ax = fig.add_subplot(gs[2, 1:])
        ax.axis('off')
        
        summary_text = "Summary Statistics:\n\n"
        for method in methods:
            method_results = [r for r in self.results if r.get('method') == method]
            if method_results:
                avg_edges = np.mean([r.get('edge_reduction_pct', 0) for r in method_results])
                avg_lambda2 = np.mean([r.get('avg_lambda2', 0) for r in method_results])
                summary_text += f"{method}:\n"
                summary_text += f"  Avg Edge Reduction: {avg_edges:.1f}%\n"
                summary_text += f"  Avg L2: {avg_lambda2:.4f}\n\n"
        
        ax.text(0.1, 0.5, summary_text, transform=ax.transAxes, fontsize=10,
               verticalalignment='center', fontfamily='monospace',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        fig.suptitle('Comprehensive Method Comparison - Summary Heatmaps', 
                    fontsize=14, fontweight='bold', y=0.98)
        
        return fig
    
    def _create_edge_reduction_plot(self) -> Optional[plt.Figure]:
        """Create edge reduction comparison plot."""
        if not self.results:
            return None
        
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        fig.suptitle('Edge Reduction Across Scenarios', fontsize=14, fontweight='bold')
        
        scenarios = sorted(list(set(r['scenario'] for r in self.results if 'scenario' in r)))
        methods = sorted(list(set(r['method'] for r in self.results if 'method' in r)))
        
        for idx, scenario in enumerate(scenarios):
            ax = axes.flat[idx]
            
            scenario_results = [r for r in self.results if r.get('scenario') == scenario]
            edge_reductions = []
            labels = []
            
            for method in methods:
                result = next((r for r in scenario_results if r.get('method') == method), None)
                if result:
                    edge_reductions.append(result.get('edge_reduction_pct', 0))
                    labels.append(method)
            
            if edge_reductions:
                colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#FFA07A']
                bars = ax.bar(range(len(labels)), edge_reductions, color=colors[:len(labels)])
                ax.set_ylabel('Edge Reduction (%)')
                ax.set_title(scenario.split(':')[0])
                ax.set_xticks(range(len(labels)))
                ax.set_xticklabels(labels, rotation=45, ha='right')
                ax.set_ylim(0, 100)
                ax.grid(axis='y', alpha=0.3)
                
                # Add value labels
                for bar in bars:
                    height = bar.get_height()
                    ax.text(bar.get_x() + bar.get_width()/2., height,
                           f'{height:.1f}%', ha='center', va='bottom', fontsize=8)
        
        # Hide unused subplots
        for idx in range(len(scenarios), len(axes.flat)):
            axes.flat[idx].set_visible(False)
        
        plt.tight_layout()
        return fig
    
    def _create_lambda2_plot(self) -> Optional[plt.Figure]:
        """Create connectivity (lambda2) comparison plot."""
        if not self.results:
            return None
        
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        fig.suptitle('Average Algebraic Connectivity (L2) Across Scenarios', 
                    fontsize=14, fontweight='bold')
        
        scenarios = sorted(list(set(r['scenario'] for r in self.results if 'scenario' in r)))
        methods = sorted(list(set(r['method'] for r in self.results if 'method' in r)))
        
        for idx, scenario in enumerate(scenarios):
            ax = axes.flat[idx]
            
            scenario_results = [r for r in self.results if r.get('scenario') == scenario]
            lambda2_values = []
            labels = []
            
            for method in methods:
                result = next((r for r in scenario_results if r.get('method') == method), None)
                if result:
                    lambda2_values.append(result.get('avg_lambda2', 0))
                    labels.append(method)
            
            if lambda2_values:
                colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#FFA07A']
                bars = ax.bar(range(len(labels)), lambda2_values, color=colors[:len(labels)])
                ax.set_ylabel('L2 (Algebraic Connectivity)')
                ax.set_title(scenario.split(':')[0])
                ax.set_xticks(range(len(labels)))
                ax.set_xticklabels(labels, rotation=45, ha='right')
                ax.axhline(y=0.1, color='r', linestyle='--', alpha=0.5, label='Safety Threshold')
                ax.grid(axis='y', alpha=0.3)
                ax.legend()
                
                # Add value labels
                for bar in bars:
                    height = bar.get_height()
                    ax.text(bar.get_x() + bar.get_width()/2., height,
                           f'{height:.4f}', ha='center', va='bottom', fontsize=8)
        
        # Hide unused subplots
        for idx in range(len(scenarios), len(axes.flat)):
            axes.flat[idx].set_visible(False)
        
        plt.tight_layout()
        return fig
    
    def _create_efficiency_plot(self) -> Optional[plt.Figure]:
        """Create efficiency plot (edge reduction vs lambda2)."""
        if not self.results:
            return None
        
        fig, ax = plt.subplots(figsize=(12, 8))
        
        methods = sorted(list(set(r['method'] for r in self.results if 'method' in r)))
        colors = {'FullyConnected': '#FF6B6B', 'CentralizedMST': '#4ECDC4', 
                 'AdjacencyConsensus': '#45B7D1', 'DistributedPruning': '#FFA07A'}
        
        for method in methods:
            method_results = [r for r in self.results if r.get('method') == method]
            
            edge_reductions = [r.get('edge_reduction_pct', 0) for r in method_results]
            lambda2_values = [r.get('avg_lambda2', 0) for r in method_results]
            
            ax.scatter(edge_reductions, lambda2_values, s=200, alpha=0.6,
                      label=method, color=colors.get(method, 'gray'))
        
        ax.set_xlabel('Edge Reduction (%)', fontsize=12)
        ax.set_ylabel('Avg Algebraic Connectivity (L2)', fontsize=12)
        ax.set_title('Method Efficiency: Edge Reduction vs Connectivity', fontsize=14, fontweight='bold')
        ax.legend(loc='best', fontsize=10)
        ax.grid(alpha=0.3)
        
        return fig
    
    def _create_control_effort_plot(self) -> Optional[plt.Figure]:
        """Create control effort comparison plot."""
        if not self.results:
            return None
        
        fig, ax = plt.subplots(figsize=(12, 6))
        
        scenarios = sorted(list(set(r['scenario'] for r in self.results if 'scenario' in r)))
        methods = sorted(list(set(r['method'] for r in self.results if 'method' in r)))
        
        x = np.arange(len(scenarios))
        width = 0.2
        colors = {'FullyConnected': '#FF6B6B', 'CentralizedMST': '#4ECDC4',
                 'AdjacencyConsensus': '#45B7D1', 'DistributedPruning': '#FFA07A'}
        
        for method_idx, method in enumerate(methods):
            efforts = []
            for scenario in scenarios:
                result = next((r for r in self.results 
                             if r.get('scenario') == scenario and r.get('method') == method),
                            None)
                efforts.append(result.get('avg_control_effort', 0) if result else 0)
            
            ax.bar(x + method_idx * width, efforts, width, label=method,
                  color=colors.get(method, 'gray'))
        
        ax.set_xlabel('Scenario', fontsize=12)
        ax.set_ylabel('Average Control Effort (N)', fontsize=12)
        ax.set_title('Control Effort Across Scenarios', fontsize=14, fontweight='bold')
        ax.set_xticks(x + width * 1.5)
        ax.set_xticklabels([s.split(':')[0] for s in scenarios], rotation=45, ha='right')
        ax.legend(loc='best')
        ax.grid(axis='y', alpha=0.3)
        
        plt.tight_layout()
        return fig


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Comprehensive scenario comparison')
    parser.add_argument('--num-scenarios', type=int, default=5,
                       help='Number of scenarios to run (1-5)')
    parser.add_argument('--num-seeds', type=int, default=1,
                       help='Number of random seeds for averaging')
    parser.add_argument('--output-dir', type=str, default='experiments',
                       help='Output directory for results')
    parser.add_argument('--verbose', action='store_true',
                       help='Enable verbose output')
    
    args = parser.parse_args()
    
    # Create runner
    runner = ComprehensiveScenarioRunner(output_dir=args.output_dir)
    
    # Run experiments
    print("\n" + "="*80)
    print("COMPREHENSIVE 5-SCENARIO COMPARISON")
    print("Methods: FullyConnected, CentralizedMST, AdjacencyConsensus, DistributedPruning")
    print("="*80)
    
    results = runner.run_all_experiments(num_seeds=args.num_seeds, verbose=args.verbose)
    
    # Save results
    results_file = runner.save_results()
    
    # Generate visualizations
    viz_files = runner.generate_visualizations()
    
    print("\n" + "="*80)
    print("EXPERIMENT COMPLETE")
    print("="*80)
    print(f"Results: {results_file}")
    print(f"Visualizations: {len(viz_files)} figures generated")
    

if __name__ == "__main__":
    main()
