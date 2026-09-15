# Results and Experimental Evaluation

## Experimental Setup

We evaluate our proposed **Concurrent Pruning** algorithm against three baseline methods across five distinct scenarios to assess performance under varying operational conditions.

### Comparison Methods

**Table 1: Comparison Methods**

| Method | Description | Pruning Strategy |
|--------|-------------|------------------|
| **Concurrent Pruning (Proposed)** | Distributed consensus with concurrent edge pruning during convergence | Lyapunov-based, fully distributed |
| **Adjacency Consensus** | Sequential pruning after full consensus convergence | Post-convergence, distributed |
| **Full Graph** | No pruning, maintains complete communication topology | No pruning (baseline) |
| **Centralized MST** | Oracle with global knowledge computing optimal topology | Centralized (upper bound) |

### Test Scenarios

**Table 2: Experimental Scenarios**

| Scenario | Description | $\Delta t$ (s) | $r_{comm}$ (m) | $N_{robots}$ | Parameters |
|----------|-------------|----------------|----------------|--------------|------------|
| **A: Baseline** | Standard operating conditions | 0.05 | 3.0 | 6 | Nominal |
| **B: Large Timestep** | Increased discretization step | 0.10 | 3.0 | 6 | Fast dynamics |
| **C: Tight Radius** | Reduced communication range | 0.05 | 2.0 | 6 | Limited connectivity |
| **D: High Robot Count** | Increased swarm size | 0.05 | 3.0 | 10 | Scalability test |
| **E: Extreme Parameters** | Combined challenging conditions | 0.10 | 2.0 | 10 | Stress test |

All scenarios maintain $\lambda_2$ safety threshold at 0.2 and goal convergence distance of 0.5m. Each method was evaluated over 10 Monte Carlo trials per scenario.

## Performance Metrics

### Connectivity Maintenance

Figure 3 demonstrates the algebraic connectivity ($\lambda_2$) evolution across all methods and scenarios. Our Concurrent Pruning method maintains $\lambda_2$ above the safety threshold (0.2) in all scenarios while achieving significant edge reduction. The distribution analysis (Figure 3, bottom-right) shows that Concurrent Pruning achieves connectivity comparable to Full Graph while operating with a sparse topology similar to Centralized MST.

**Table 3: Connectivity and Success Rates**

| Method | Avg. Min $\lambda_2$ | Success Rate (%) | Avg. Edge Reduction (%) |
|--------|---------------------|------------------|------------------------|
| Concurrent Pruning | 0.287 ± 0.043 | 96.0 | 42.3 ± 8.2 |
| Adjacency Consensus | 0.251 ± 0.038 | 88.0 | 38.1 ± 9.5 |
| Full Graph | 0.512 ± 0.067 | 100.0 | 0.0 |
| Centralized MST | 0.295 ± 0.041 | 98.0 | 45.7 ± 6.8 |

### Topology Evolution

Figure 1 illustrates representative robot trajectories in Scenario A, showing the transition from a fully connected initial topology to a sparse near-MST configuration. The flow field visualization demonstrates how robots navigate toward the goal while maintaining connectivity constraints. Figure 2 presents edge count evolution across all five scenarios, revealing that Concurrent Pruning achieves rapid convergence to sparse topologies (within 50-100 timesteps) while maintaining network stability.

### Pruning Efficiency

The pruning efficiency analysis (Figure 7 and Figure R4) reveals distinct performance across scenarios. Concurrent Pruning demonstrates superior efficiency in baseline and moderate conditions (Scenarios A, B, C), achieving 40-50% edge reduction. In challenging scenarios (D, E), all methods show reduced pruning capability due to connectivity constraints, but our approach maintains better performance than Adjacency Consensus.

**Table 4: Pruning Efficiency by Scenario**

| Scenario | Concurrent Pruning | Adjacency Consensus | Centralized MST |
|----------|-------------------|---------------------|-----------------|
| A: Baseline | 48.2% ± 5.3% | 43.1% ± 7.2% | 51.3% ± 4.8% |
| B: Large Timestep | 45.7% ± 6.8% | 39.5% ± 8.9% | 49.2% ± 5.5% |
| C: Tight Radius | 38.3% ± 7.1% | 34.2% ± 9.3% | 42.1% ± 6.2% |
| D: High Robot Count | 41.9% ± 6.5% | 36.8% ± 8.7% | 44.6% ± 5.9% |
| E: Extreme | 32.1% ± 9.2% | 28.4% ± 10.5% | 35.8% ± 7.8% |

### Control Effort Analysis

Figure 4 compares cumulative control effort across methods. Despite operating with sparser topologies, Concurrent Pruning achieves control effort within 5-10% of Full Graph baseline, demonstrating efficient energy utilization. The trade-off analysis (Figure R1) reveals that our method achieves an optimal balance: 42% average edge reduction while maintaining $\lambda_2 = 0.287$, compared to Adjacency Consensus (38% reduction, $\lambda_2 = 0.251$) and Full Graph (0% reduction, $\lambda_2 = 0.512$).

### Computational Efficiency

Runtime analysis (Figure R3) shows that Concurrent Pruning introduces minimal computational overhead compared to Full Graph (average increase: 8.3%), while providing substantial communication savings. Centralized MST requires 45% more computation due to global optimization, making it impractical for real-time distributed systems.

**Table 5: Computational Performance**

| Method | Avg. Runtime (s) | Std. Dev (s) | Overhead vs. Full Graph |
|--------|-----------------|--------------|-------------------------|
| Concurrent Pruning | 12.4 | 2.1 | +8.3% |
| Adjacency Consensus | 11.8 | 2.3 | +3.5% |
| Full Graph | 11.4 | 1.9 | 0% (baseline) |
| Centralized MST | 16.5 | 2.8 | +44.7% |

### Success Rate Analysis

The heatmap visualization (Figure 5) confirms high success rates across all scenarios. Concurrent Pruning achieves 96% overall success rate, matching or exceeding Adjacency Consensus (88%) while operating with sparser topologies. Performance degradation in Scenario E (Extreme) is observed across all methods due to combined stressors, but our approach maintains 90% success compared to 82% for Adjacency Consensus.

### Performance Trade-offs

Figure R1 and R2 quantify the fundamental trade-off between network sparsity and connectivity safety. Our Concurrent Pruning method achieves near-Pareto-optimal performance: edge reduction comparable to the centralized oracle while maintaining connectivity margins similar to conservative baselines. This demonstrates the effectiveness of the distributed Lyapunov-based pruning criterion.

## Key Findings

1. **Distributed Optimality**: Concurrent Pruning achieves 92.5% of Centralized MST's edge reduction without global knowledge
2. **Connectivity Safety**: Maintains $\lambda_2 > 0.2$ threshold with 96% success rate across diverse scenarios
3. **Scalability**: Performance scales to 10 robots with only 8-12% reduction in pruning efficiency
4. **Real-time Capable**: Computational overhead under 10% compared to baseline, suitable for embedded systems
5. **Robustness**: Graceful degradation under extreme conditions (Scenario E), maintaining 90% success rate

## Summary

The experimental results validate our concurrent pruning approach as an effective distributed solution for communication topology optimization in multi-robot systems. By integrating pruning decisions into the consensus process rather than treating them as separate phases, our method achieves superior efficiency and connectivity maintenance compared to sequential alternatives, while approaching the performance of centralized methods that require global network knowledge.
