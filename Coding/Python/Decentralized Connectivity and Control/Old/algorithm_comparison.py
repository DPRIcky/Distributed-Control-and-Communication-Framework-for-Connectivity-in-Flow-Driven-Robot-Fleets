#!/usr/bin/env python3

"""
Comprehensive Comparison Framework: Centralized MLCCST vs Decentralized Algorithms
Compares performance across multiple metrics in underwater environments
"""

import numpy as np
import matplotlib.pyplot as plt
import time
import json
from datetime import datetime

# Import the algorithms
from Base_decentralized_connectivity_with_underwater_flow_field_updated import (
    UnderwaterEnvironment,
    SingleGoalMLCCSTController,
    DistributedNetwork,
    create_clustered_formation,
    generate_single_random_goal,
    generate_random_obstacles,
    CENTRALIZED_NUM_ROBOTS,
    CENTRALIZED_MAX_ITERATIONS
)

class AlgorithmComparator:
    """Comprehensive comparison framework for centralized vs decentralized algorithms"""
    
    def __init__(self, num_robots=40, num_trials=5):
        self.num_robots = num_robots
        self.num_trials = num_trials
        self.results = {
            'centralized': [],
            'decentralized': []
        }
        
    def run_centralized_trial(self, goal, underwater_env, trial_num):
        """Run single trial of centralized MLCCST algorithm"""
        print(f"  🎯 Centralized Trial {trial_num + 1}/{self.num_trials}")
        
        # Create initial formation
        origin = np.array([0.0, 0.0])
        initial_positions = create_clustered_formation(self.num_robots, origin)
        
        # Initialize controller
        controller = SingleGoalMLCCSTController(
            initial_positions=initial_positions,
            goal=goal,
            obstacles=[],
            underwater_env=underwater_env
        )
        
        # Metrics tracking
        metrics = {
            'connectivity_history': [],
            'robots_at_goal_history': [],
            'total_distance': 0.0,
            'goal_reached_count': 0,
            'time_to_first_goal': None,
            'final_connectivity': 0.0,
            'communication_links': [],
            'position_history': []
        }
        
        # Run simulation
        prev_positions = controller.get_positions().copy()
        
        for iteration in range(CENTRALIZED_MAX_ITERATIONS):
            connectivity_ratio = controller.step(iteration)
            current_metrics = controller.get_connectivity_metrics()
            current_positions = controller.get_positions()
            
            # Track metrics
            metrics['connectivity_history'].append(connectivity_ratio)
            metrics['robots_at_goal_history'].append(current_metrics['at_goal'])
            metrics['position_history'].append(current_positions.copy())
            
            # Calculate distance traveled this step
            if iteration > 0:
                step_distances = np.linalg.norm(current_positions - prev_positions, axis=1)
                metrics['total_distance'] += np.sum(step_distances)
            
            # Track first robot reaching goal
            if current_metrics['at_goal'] > 0 and metrics['time_to_first_goal'] is None:
                metrics['time_to_first_goal'] = iteration
            
            # Track communication links
            comm_links = sum(len([r for r in controller.robots if r.parent is not None]))
            metrics['communication_links'].append(comm_links)
            
            prev_positions = current_positions.copy()
        
        # Final metrics
        final_metrics = controller.get_connectivity_metrics()
        metrics['final_connectivity'] = final_metrics['connectivity_ratio']
        metrics['goal_reached_count'] = final_metrics['at_goal']
        
        return metrics
    
    def run_decentralized_trial(self, goal, underwater_env, trial_num):
        """Run single trial of decentralized algorithm"""
        print(f"  🌐 Decentralized Trial {trial_num + 1}/{self.num_trials}")
        
        # Create distributed network
        network = DistributedNetwork(create_default_graph=False, enable_underwater_physics=True)
        
        # Create clustered initial positions
        origin = np.array([0.0, 0.0])
        initial_positions = create_clustered_formation(self.num_robots, origin)
        
        # Add nodes to network
        from Base_decentralized_connectivity_with_underwater_flow_field_updated import Node
        for i, pos in enumerate(initial_positions):
            network.nodes[i+1] = Node(i+1, pos)
        
        # Create initial connectivity (similar to centralized)
        for i in range(len(initial_positions)):
            for j in range(i+1, len(initial_positions)):
                node1, node2 = i+1, j+1
                dist = np.linalg.norm(initial_positions[i] - initial_positions[j])
                if dist < 1.0:  # Initial communication radius
                    network.nodes[node1].add_direct_neighbor(node2)
                    network.nodes[node2].add_direct_neighbor(node1)
        
        # Set underwater environment
        network.underwater_env = underwater_env
        
        # Metrics tracking
        metrics = {
            'connectivity_history': [],
            'robots_at_goal_history': [],
            'total_distance': 0.0,
            'goal_reached_count': 0,
            'time_to_first_goal': None,
            'final_connectivity': 0.0,
            'communication_links': [],
            'position_history': []
        }
        
        # Run goal-seeking simulation (modified for metrics collection)
        goal_x, goal_y = goal[0], goal[1]
        goal_radius = 0.5  # Same as centralized
        
        prev_positions = {node_id: node.position.copy() for node_id, node in network.nodes.items()}
        
        for iteration in range(CENTRALIZED_MAX_ITERATIONS):
            # Update robot positions using decentralized algorithm
            network.update_positions_goal_seeking()
            
            # Calculate connectivity
            total_nodes = len(network.nodes)
            connected_components = network.count_connected_components()
            connectivity_ratio = 1.0 - (connected_components - 1) / max(total_nodes - 1, 1)
            
            # Count robots at goal
            robots_at_goal = 0
            current_positions = []
            
            for node_id, node in network.nodes.items():
                current_positions.append(node.position.copy())
                dist_to_goal = np.linalg.norm(node.position - goal)
                if dist_to_goal < goal_radius:
                    robots_at_goal += 1
            
            current_positions = np.array(current_positions)
            
            # Track metrics
            metrics['connectivity_history'].append(connectivity_ratio)
            metrics['robots_at_goal_history'].append(robots_at_goal)
            metrics['position_history'].append(current_positions.copy())
            
            # Calculate distance traveled
            if iteration > 0:
                total_step_distance = 0
                for node_id, node in network.nodes.items():
                    step_distance = np.linalg.norm(node.position - prev_positions[node_id])
                    total_step_distance += step_distance
                metrics['total_distance'] += total_step_distance
            
            # Track first robot reaching goal
            if robots_at_goal > 0 and metrics['time_to_first_goal'] is None:
                metrics['time_to_first_goal'] = iteration
            
            # Track communication links
            total_links = sum(len(node.direct_neighbors) for node in network.nodes.values()) // 2
            metrics['communication_links'].append(total_links)
            
            prev_positions = {node_id: node.position.copy() for node_id, node in network.nodes.items()}
        
        # Final metrics
        metrics['final_connectivity'] = connectivity_ratio
        metrics['goal_reached_count'] = robots_at_goal
        
        return metrics
    
    def run_comparison(self, save_results=True):
        """Run comprehensive comparison between algorithms"""
        print("🔬 COMPREHENSIVE ALGORITHM COMPARISON")
        print("=" * 80)
        print(f"Parameters:")
        print(f"  • Number of robots: {self.num_robots}")
        print(f"  • Number of trials: {self.num_trials}")
        print(f"  • Max iterations: {CENTRALIZED_MAX_ITERATIONS}")
        print()
        
        # Create underwater environment
        underwater_env = UnderwaterEnvironment(workspace_size=(10, 10))
        
        for trial in range(self.num_trials):
            print(f"🚀 Running Trial {trial + 1}/{self.num_trials}")
            
            # Generate same goal for both algorithms
            bounds = [-5, 5, -5, 5]
            goal = generate_single_random_goal(bounds)
            print(f"  Goal: ({goal[0]:.2f}, {goal[1]:.2f})")
            
            # Run centralized trial
            centralized_metrics = self.run_centralized_trial(goal, underwater_env, trial)
            self.results['centralized'].append(centralized_metrics)
            
            # Run decentralized trial  
            decentralized_metrics = self.run_decentralized_trial(goal, underwater_env, trial)
            self.results['decentralized'].append(decentralized_metrics)
            
            print(f"  ✅ Trial {trial + 1} completed")
            print()
        
        # Analyze and display results
        self.analyze_results()
        
        if save_results:
            self.save_results()
    
    def analyze_results(self):
        """Analyze and display comparison results"""
        print("📊 COMPARISON RESULTS ANALYSIS")
        print("=" * 80)
        
        # Calculate average metrics
        cent_metrics = self.calculate_average_metrics('centralized')
        decent_metrics = self.calculate_average_metrics('decentralized')
        
        # Display comparison table
        print("📈 PERFORMANCE COMPARISON TABLE")
        print("-" * 80)
        print(f"{'Metric':<30} {'Centralized':<15} {'Decentralized':<15} {'Winner':<10}")
        print("-" * 80)
        
        # Connectivity metrics
        print("🔗 CONNECTIVITY METRICS:")
        self.compare_metric("Avg Connectivity", cent_metrics['avg_connectivity'], 
                          decent_metrics['avg_connectivity'], higher_better=True)
        self.compare_metric("Final Connectivity", cent_metrics['final_connectivity'], 
                          decent_metrics['final_connectivity'], higher_better=True)
        self.compare_metric("Min Connectivity", cent_metrics['min_connectivity'], 
                          decent_metrics['min_connectivity'], higher_better=True)
        
        # Goal achievement metrics
        print("\n🎯 GOAL ACHIEVEMENT METRICS:")
        self.compare_metric("Robots at Goal", cent_metrics['avg_goal_reached'], 
                          decent_metrics['avg_goal_reached'], higher_better=True)
        self.compare_metric("Goal Success Rate", cent_metrics['goal_success_rate'], 
                          decent_metrics['goal_success_rate'], higher_better=True)
        self.compare_metric("Time to First Goal", cent_metrics['avg_time_to_goal'], 
                          decent_metrics['avg_time_to_goal'], higher_better=False)
        
        # Efficiency metrics
        print("\n⚡ EFFICIENCY METRICS:")
        self.compare_metric("Total Distance", cent_metrics['avg_total_distance'], 
                          decent_metrics['avg_total_distance'], higher_better=False)
        self.compare_metric("Avg Comm Links", cent_metrics['avg_comm_links'], 
                          decent_metrics['avg_comm_links'], higher_better=False)
        
        print("-" * 80)
        
        # Generate summary
        self.generate_summary(cent_metrics, decent_metrics)
    
    def calculate_average_metrics(self, algorithm):
        """Calculate average metrics across all trials"""
        results = self.results[algorithm]
        
        metrics = {
            'avg_connectivity': np.mean([np.mean(r['connectivity_history']) for r in results]),
            'final_connectivity': np.mean([r['final_connectivity'] for r in results]),
            'min_connectivity': np.mean([np.min(r['connectivity_history']) for r in results]),
            'avg_goal_reached': np.mean([r['goal_reached_count'] for r in results]),
            'goal_success_rate': np.mean([r['goal_reached_count'] / self.num_robots for r in results]),
            'avg_time_to_goal': np.mean([r['time_to_first_goal'] or CENTRALIZED_MAX_ITERATIONS for r in results]),
            'avg_total_distance': np.mean([r['total_distance'] for r in results]),
            'avg_comm_links': np.mean([np.mean(r['communication_links']) for r in results])
        }
        
        return metrics
    
    def compare_metric(self, metric_name, cent_value, decent_value, higher_better=True):
        """Compare and display a single metric"""
        if higher_better:
            winner = "CENTRAL" if cent_value > decent_value else "DECENTRAL"
        else:
            winner = "CENTRAL" if cent_value < decent_value else "DECENTRAL"
        
        print(f"{metric_name:<30} {cent_value:<15.3f} {decent_value:<15.3f} {winner:<10}")
    
    def generate_summary(self, cent_metrics, decent_metrics):
        """Generate overall comparison summary"""
        print("\n🏆 OVERALL PERFORMANCE SUMMARY")
        print("=" * 80)
        
        # Count wins for each algorithm
        cent_wins = 0
        decent_wins = 0
        
        # Connectivity wins (higher better)
        if cent_metrics['avg_connectivity'] > decent_metrics['avg_connectivity']:
            cent_wins += 1
        else:
            decent_wins += 1
            
        if cent_metrics['final_connectivity'] > decent_metrics['final_connectivity']:
            cent_wins += 1
        else:
            decent_wins += 1
            
        # Goal achievement wins (higher better)
        if cent_metrics['avg_goal_reached'] > decent_metrics['avg_goal_reached']:
            cent_wins += 1
        else:
            decent_wins += 1
            
        # Efficiency wins (lower better)
        if cent_metrics['avg_total_distance'] < decent_metrics['avg_total_distance']:
            cent_wins += 1
        else:
            decent_wins += 1
            
        if cent_metrics['avg_time_to_goal'] < decent_metrics['avg_time_to_goal']:
            cent_wins += 1
        else:
            decent_wins += 1
        
        # Determine overall winner
        if cent_wins > decent_wins:
            winner = "🎯 CENTRALIZED MLCCST"
            winner_desc = "Better coordination and goal achievement"
        elif decent_wins > cent_wins:
            winner = "🌐 DECENTRALIZED"
            winner_desc = "Better distributed efficiency"
        else:
            winner = "🤝 TIE"
            winner_desc = "Both algorithms show comparable performance"
        
        print(f"Centralized wins: {cent_wins}/5 metrics")
        print(f"Decentralized wins: {decent_wins}/5 metrics")
        print(f"\nOverall Winner: {winner}")
        print(f"Reason: {winner_desc}")
        print("=" * 80)
    
    def save_results(self):
        """Save comparison results to file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"algorithm_comparison_{timestamp}.json"
        
        # Prepare data for JSON serialization
        save_data = {
            'timestamp': timestamp,
            'num_robots': self.num_robots,
            'num_trials': self.num_trials,
            'results_summary': {
                'centralized': self.calculate_average_metrics('centralized'),
                'decentralized': self.calculate_average_metrics('decentralized')
            }
        }
        
        # Convert numpy arrays to lists for JSON
        for algorithm in ['centralized', 'decentralized']:
            for key in save_data['results_summary'][algorithm]:
                value = save_data['results_summary'][algorithm][key]
                if isinstance(value, np.ndarray):
                    save_data['results_summary'][algorithm][key] = value.tolist()
                elif isinstance(value, np.floating):
                    save_data['results_summary'][algorithm][key] = float(value)
        
        with open(filename, 'w') as f:
            json.dump(save_data, f, indent=2)
        
        print(f"💾 Results saved to: {filename}")


def main():
    """Run the comprehensive comparison"""
    print("🚀 Starting Comprehensive Algorithm Comparison...")
    print("This will compare Centralized MLCCST vs Decentralized algorithms")
    print("across multiple performance metrics in underwater environments.")
    print()
    
    # Create and run comparison
    comparator = AlgorithmComparator(num_robots=40, num_trials=3)  # 3 trials for faster testing
    comparator.run_comparison()


if __name__ == "__main__":
    main()