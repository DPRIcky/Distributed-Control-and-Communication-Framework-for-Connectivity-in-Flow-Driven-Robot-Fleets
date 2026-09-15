# Theoretical Foundations: Research Papers

A curated list of foundational and recent papers covering the theoretical basis for this underwater multi-robot consensus system.

---

## Table of Contents
1. [Control Barrier Functions (CBF)](#1-control-barrier-functions-cbf)
2. [Control Lyapunov Functions (CLF)](#2-control-lyapunov-functions-clf)
3. [Combined CLF-CBF Methods](#3-combined-clf-cbf-methods)
4. [Distributed Consensus Algorithms](#4-distributed-consensus-algorithms)
5. [Network Topology Optimization](#5-network-topology-optimization)
6. [Multi-Robot Coordination](#6-multi-robot-coordination)
7. [Underwater Robotics & AUV Swarms](#7-underwater-robotics--auv-swarms)
8. [Graph Theory for Multi-Agent Systems](#8-graph-theory-for-multi-agent-systems)

---

## 1. Control Barrier Functions (CBF)

### Foundational Papers

**[1.1] Robustness of control barrier functions for safety critical control**
- **Authors**: Xu, X., Tabuada, P., Grizzle, J.W., Ames, A.D.
- **Venue**: IFAC-PapersOnLine, 2015
- **Citations**: 792+
- **Link**: https://www.sciencedirect.com/science/article/pii/S2405896315024106
- **Why Read**: Introduces robustness properties of CBFs, essential for understanding safety guarantees in uncertain environments like underwater settings.
- **Key Concepts**: CBF robustness, disturbance rejection, safety-critical control

**[1.2] Discrete control barrier functions for safety-critical control of discrete systems**
- **Authors**: Agrawal, A., Sreenath, K.
- **Venue**: Robotics: Science and Systems, 2017
- **Citations**: 410+
- **Link**: https://m.roboticsproceedings.org/rss13/p73.pdf
- **Why Read**: Extends CBFs to discrete-time systems, directly applicable to simulation timestep-based control.
- **Key Concepts**: Discrete-time CBF, safety constraints, bipedal navigation

**[1.3] Learning for safety-critical control with control barrier functions**
- **Authors**: Taylor, A., Singletary, A., Yue, Y., Ames, A.D.
- **Venue**: Learning for Dynamics and Control (L4DC), 2020
- **Citations**: 359+
- **Link**: https://proceedings.mlr.press/v120/taylor20a.html
- **Why Read**: Combines learning with CBFs, useful for adapting to underwater flow dynamics.
- **Key Concepts**: Learning-based CBF, data-driven safety, adaptive control

### Advanced Topics

**[1.4] Disturbance observers for robust safety-critical control with control barrier functions**
- **Authors**: Alan, A., Molnar, T.G., Daş, E., Ames, A.D.
- **Venue**: IEEE Control Systems Letters, 2022
- **Citations**: 78+
- **Link**: https://ieeexplore.ieee.org/abstract/document/9998997/
- **Why Read**: Handles external disturbances (like flow fields) in CBF framework.
- **Key Concepts**: Disturbance observers, robust CBF, uncertainty handling

**[1.5] Safety-critical control and planning for obstacle avoidance with CBFs**
- **Authors**: Thirugnanam, A., Zeng, J., Zhang, B.
- **Venue**: IEEE International Conference on Robotics and Automation, 2022
- **Citations**: 119+
- **Link**: https://ieeexplore.ieee.org/abstract/document/9812334/
- **Why Read**: Collision avoidance using CBFs, directly relevant to your safety distance constraints.
- **Key Concepts**: Obstacle avoidance, polytope safety, DCBF constraints

---

## 2. Control Lyapunov Functions (CLF)

### Foundational Papers

**[2.1] Control Lyapunov functions for adaptive nonlinear stabilization**
- **Authors**: Krstić, M., Kokotović, P.V.
- **Venue**: Systems & Control Letters, 1995
- **Citations**: 138+
- **Link**: https://www.sciencedirect.com/science/article/pii/0167691194001077
- **Why Read**: Classic paper on CLF for nonlinear systems, foundation for goal-seeking behavior.
- **Key Concepts**: Adaptive CLF, nonlinear stabilization, parameter uncertainty

**[2.2] Learning control Lyapunov function to ensure stability**
- **Authors**: Khansari-Zadeh, S.M., Billard, A.
- **Venue**: Robotics and Autonomous Systems, 2014
- **Citations**: 302+
- **Link**: https://www.sciencedirect.com/science/article/pii/S0921889014000372
- **Why Read**: Learning CLFs from demonstrations, applicable to learned behaviors in robot swarms.
- **Key Concepts**: Learning CLF, global asymptotic stability, robot reaching motions

**[2.3] Control Lyapunov-Razumikhin functions for time delay systems**
- **Authors**: Jankovic, M.
- **Venue**: IEEE Transactions on Automatic Control, 2002
- **Citations**: 467+
- **Link**: https://ieeexplore.ieee.org/abstract/document/935057/
- **Why Read**: Handles communication delays in distributed systems.
- **Key Concepts**: Time delays, CLF with delays, robust stabilization

---

## 3. Combined CLF-CBF Methods

### Essential Reading

**[3.1] Reinforcement learning for safety-critical control using CLF and CBF**
- **Authors**: Choi, J., Castaneda, F., Tomlin, C.J., Sreenath, K.
- **Venue**: arXiv preprint, 2020
- **Citations**: 295+
- **Link**: https://arxiv.org/abs/2004.07584
- **Why Read**: **HIGHLY RELEVANT** - Combines CLF (goal-seeking) and CBF (safety) in QP framework, exactly your approach.
- **Key Concepts**: CLF-CBF-QP, model uncertainty, unified framework

**[3.2] Nonlinear model predictive control with control Lyapunov functions**
- **Authors**: Grandia, R., Taylor, A.J., Singletary, A., Hutter, M., Ames, A.D.
- **Venue**: arXiv preprint, 2020
- **Citations**: 66+
- **Link**: https://arxiv.org/abs/2006.01229
- **Why Read**: Integrates CLF constraints into predictive control for robotic systems.
- **Key Concepts**: NMPC, CLF constraints, robotic systems

---

## 4. Distributed Consensus Algorithms

### Core Theory

**[4.1] Consensus in multi-agent systems: a review**
- **Authors**: Amirkhani, A., Barshooi, A.H.
- **Venue**: Artificial Intelligence Review, 2022
- **Citations**: 454+
- **Link**: https://link.springer.com/article/10.1007/s10462-021-10097-x
- **Why Read**: **COMPREHENSIVE REVIEW** - Covers consensus theory, graph analysis, and distributed algorithms.
- **Key Concepts**: Consensus protocols, spectral analysis, directed graphs

**[4.2] Network topology and communication data rate for consensusability**
- **Authors**: You, K., Xie, L.
- **Venue**: IEEE Transactions on Automatic Control, 2011
- **Citations**: 685+
- **Link**: https://ieeexplore.ieee.org/abstract/document/5978201/
- **Why Read**: Analyzes how network topology affects consensus, relevant for edge pruning.
- **Key Concepts**: Consensusability, data rate, discrete-time multi-agent systems

**[4.3] Multi-agent systems with dynamical topologies: Consensus and applications**
- **Authors**: Chen, Y., Lu, J., Yu, X., Hill, D.J.
- **Venue**: IEEE Circuits and Systems Magazine, 2013
- **Citations**: 206+
- **Link**: https://ieeexplore.ieee.org/abstract/document/6585767/
- **Why Read**: Addresses consensus under changing topologies, directly applicable to your dynamic network.
- **Key Concepts**: Dynamical topology, switching graphs, consensus convergence

---

## 5. Network Topology Optimization

### Highly Relevant

**[5.1] Consensus-based distributed connectivity control in multi-agent systems**
- **Authors**: Griparic, K., Polic, M., Krizmancic, M., Bogdan, S.
- **Venue**: IEEE Transactions on Network Science and Engineering, 2022
- **Citations**: 49+
- **Link**: https://ieeexplore.ieee.org/abstract/document/9674850/
- **Why Read**: **DIRECTLY RELEVANT** - Distributed connectivity control using consensus, similar to your pruning approach.
- **Key Concepts**: Connectivity maintenance, distributed control, graph properties

**[5.2] Optimal network topology design for efficient average consensus**
- **Authors**: Rafiee, M., Bayen, A.M.
- **Venue**: IEEE Conference on Decision and Control, 2010
- **Citations**: 95+
- **Link**: https://ieeexplore.ieee.org/abstract/document/5717719/
- **Why Read**: Optimization of network topology for consensus efficiency, foundation for redundant edge removal.
- **Key Concepts**: Topology optimization, average consensus, network design

**[5.3] Distributed consensus of heterogeneous multi-agent systems with switching topologies**
- **Authors**: Zheng, Y., Wang, L.
- **Venue**: International Journal of Control, 2012
- **Citations**: 133+
- **Link**: https://www.tandfonline.com/doi/abs/10.1080/00207179.2012.713986
- **Why Read**: Handles heterogeneous agents (different dynamics) with changing networks.
- **Key Concepts**: Heterogeneous agents, switching topologies, directed networks

---

## 6. Multi-Robot Coordination

### Key Papers

**[6.1] Network-based leader-following consensus for distributed multi-agent systems**
- **Authors**: Ding, L., Han, Q.L., Guo, G.
- **Venue**: Automatica, 2013
- **Citations**: 425+
- **Link**: https://www.sciencedirect.com/science/article/pii/S0005109813002331
- **Why Read**: Leader-following consensus with network constraints, applicable to goal-seeking.
- **Key Concepts**: Leader-following, network communication, directed graphs

**[6.2] Distributed consensus control with higher order dynamics**
- **Authors**: Su, S., Lin, Z.
- **Venue**: IEEE Transactions on Automatic Control, 2015
- **Citations**: 175+
- **Link**: https://ieeexplore.ieee.org/abstract/document/7122276/
- **Why Read**: Consensus for higher-order dynamics (velocity, acceleration), relevant for robot dynamics.
- **Key Concepts**: Higher-order consensus, directed topologies, distributed protocols

**[6.3] Distributed consensus tracking under attacks**
- **Authors**: Feng, Z., Hu, G., Wen, G.
- **Venue**: International Journal of Robust and Nonlinear Control, 2016
- **Citations**: 258+
- **Link**: https://onlinelibrary.wiley.com/doi/abs/10.1002/rnc.3342
- **Why Read**: Robustness to edge failures (attacks), similar to your topology change handling.
- **Key Concepts**: Attack resilience, consensus tracking, network robustness

---

## 7. Underwater Robotics & AUV Swarms

### Domain-Specific

**[7.1] A survey of underwater multi-robot systems**
- **Authors**: Zhou, Z., Liu, J., Yu, J.
- **Venue**: IEEE/CAA Journal of Automatica Sinica, 2021
- **Citations**: 149+
- **Link**: https://ieeexplore.ieee.org/abstract/document/9583166/
- **Why Read**: **COMPREHENSIVE** - Covers underwater multi-robot challenges, communication, coordination.
- **Key Concepts**: Underwater communication, AUV cooperation, swarm intelligence

**[7.2] Bio-inspired swarm of underwater robots: a review**
- **Authors**: Zhao, Q., Yang, T., Tang, G., Yang, Y., Dong, F.
- **Venue**: Bioinspiration & Biomimetics, 2025
- **Citations**: 6+ (very recent)
- **Link**: https://iopscience.iop.org/article/10.1088/1748-3190/ade215/meta
- **Why Read**: Latest review on underwater swarm robotics, bio-inspired approaches.
- **Key Concepts**: Swarm robotics, bio-inspiration, underwater coordination

**[7.3] Bio-inspired self-organized cooperative control for crowded UUV swarm**
- **Authors**: Liang, H., Fu, Y., Gao, J.
- **Venue**: Applied Intelligence, 2021
- **Citations**: 26+
- **Link**: https://link.springer.com/article/10.1007/s10489-020-02104-5
- **Why Read**: Self-organized control in crowded environments, adaptive topology.
- **Key Concepts**: Self-organization, adaptive interaction topology, UUV swarms

**[7.4] Co-operative control of underwater vehicles with communication constraints**
- **Authors**: Das, B., Subudhi, B., Pati, B.B.
- **Venue**: Transactions of the Institute of Measurement and Control, 2016
- **Citations**: 51+
- **Link**: https://journals.sagepub.com/doi/abs/10.1177/0142331215590010
- **Why Read**: Addresses communication limitations in underwater environments.
- **Key Concepts**: Communication constraints, AUV coordination, leader-follower

**[7.5] Robotic swarm for marine and submarine missions**
- **Authors**: Luvisutto, A., Al Shehhi, A., Mankovskii, N., De Masi, G.
- **Venue**: IEEE Oceans Conference, 2022
- **Citations**: 14+
- **Link**: https://ieeexplore.ieee.org/abstract/document/9965934/
- **Why Read**: Practical challenges in underwater swarm deployment.
- **Key Concepts**: Marine missions, swarm deployment, underwater simulators

---

## 8. Graph Theory for Multi-Agent Systems

### Supporting Theory

**[8.1] Consensus control with leader-following in directed topology**
- **Authors**: Wei, Q., Wang, X., Zhong, X., Wu, N.
- **Venue**: IEEE/CAA Journal of Automatica Sinica, 2021
- **Citations**: 175+
- **Link**: https://ieeexplore.ieee.org/abstract/document/9317712/
- **Why Read**: Directed graph consensus, relevant for parent-child connectivity tree.
- **Key Concepts**: Directed topology, leader-following, heterogeneous disturbances

**[8.2] Recent advances in formations of multiple robots**
- **Authors**: Cohen, S., Agmon, N.
- **Venue**: Current Robotics Reports, 2021
- **Citations**: 15+
- **Link**: https://link.springer.com/article/10.1007/s43154-021-00049-2
- **Why Read**: Overview of formation control methods and coordination strategies.
- **Key Concepts**: Formation control, multi-robot coordination, decoupled systems

---

## Recommended Reading Order

### For Beginners (Building Foundation)

1. **Start with Reviews**:
   - [4.1] Consensus in multi-agent systems: a review
   - [7.1] A survey of underwater multi-robot systems

2. **Core Control Theory**:
   - [2.1] CLF for adaptive nonlinear stabilization
   - [1.1] Robustness of CBF for safety critical control

3. **Combined Methods**:
   - [3.1] Reinforcement learning with CLF and CBF ⭐ **MOST RELEVANT**

### For Understanding Your Code

1. **Control Framework**:
   - [3.1] RL for safety-critical control using CLF-CBF ⭐
   - [1.2] Discrete control barrier functions
   - [2.2] Learning CLF for robot motions

2. **Consensus & Topology**:
   - [5.1] Consensus-based distributed connectivity control ⭐
   - [4.3] Multi-agent systems with dynamical topologies
   - [5.2] Optimal network topology design

3. **Underwater Domain**:
   - [7.1] Survey of underwater multi-robot systems ⭐
   - [7.3] Bio-inspired self-organized control
   - [7.4] Co-operative control with communication constraints

### For Advanced Topics

1. **Robustness**:
   - [1.4] Disturbance observers for robust CBF
   - [6.3] Distributed consensus tracking under attacks
   - [2.3] CLF-Razumikhin for time delay systems

2. **Optimization**:
   - [5.2] Optimal network topology design
   - [4.2] Network topology and consensusability

3. **Recent Innovations**:
   - [7.2] Bio-inspired swarm of underwater robots (2025)
   - [5.1] Consensus-based connectivity control (2022)

---

## How Each Paper Relates to Your Code

### Control Framework
- **CLF** ([2.1], [2.2]): Your `_compute_clf_cbf_control()` goal-seeking term
- **CBF** ([1.1], [1.2]): Your collision avoidance and connectivity maintenance
- **CLF-CBF QP** ([3.1]): Combining both for unified control

### Consensus Mechanism
- **Distributed Consensus** ([4.1], [4.3]): Your `ConsensusPruningSimulation` integration
- **Topology Optimization** ([5.1], [5.2]): Your redundant edge identification
- **Dynamic Topologies** ([4.3]): Your topology stability detection

### Robustness Features
- **Disturbance Handling** ([1.4]): Your flow field modeling
- **Topology Changes** ([4.3], [6.3]): Your multi-layer validation approach
- **Communication Constraints** ([7.4]): Your communication radius limits

### Underwater Domain
- **AUV Dynamics** ([7.1]): Your robot physical properties (mass, drag)
- **Swarm Coordination** ([7.3]): Your connectivity tree and parent-child links
- **Flow Fields** ([7.1]): Your `FlowField` class with currents

---

## Additional Resources

### Books
1. **"Cooperative Control of Multi-Agent Systems"** by Frank L. Lewis et al.
2. **"Graph Theory and Its Applications to Multi-Robot Systems"** by Mesbahi & Egerstedt
3. **"Nonlinear Control Systems"** by Alberto Isidori (CLF/CBF foundations)

### Online Courses
1. **"Multi-Robot Systems"** - Coursera (University of Pennsylvania)
2. **"Underactuated Robotics"** - MIT OpenCourseWare (Russ Tedrake)
3. **"Control Barrier Functions"** - YouTube lectures by Aaron Ames

### Software/Simulators
1. **ROS (Robot Operating System)** - Multi-robot frameworks
2. **Gazebo + UUV Simulator** - Underwater vehicle simulation
3. **MATLAB Robotics Toolbox** - Multi-agent consensus examples

---

## Citation Format (BibTeX)

```bibtex
@article{choi2020reinforcement,
  title={Reinforcement learning for safety-critical control under model uncertainty, using control lyapunov functions and control barrier functions},
  author={Choi, Jason and Castaneda, Fernando and Tomlin, Claire J and Sreenath, Koushil},
  journal={arXiv preprint arXiv:2004.07584},
  year={2020}
}

@article{griparic2022consensus,
  title={Consensus-based distributed connectivity control in multi-agent systems},
  author={Griparic, Karlo and Polic, Marko and Krizmancic, Marsela and Bogdan, Stjepan},
  journal={IEEE Transactions on Network Science and Engineering},
  year={2022}
}

@article{zhou2021survey,
  title={A survey of underwater multi-robot systems},
  author={Zhou, Zihan and Liu, Jianxing and Yu, Jiancheng},
  journal={IEEE/CAA Journal of Automatica Sinica},
  volume={9},
  number={1},
  pages={1--18},
  year={2021}
}
```

---

## Search Keywords for Further Research

- "Control barrier functions safety critical"
- "Control Lyapunov functions stabilization"
- "Distributed consensus multi-agent"
- "Network topology optimization consensus"
- "Underwater robot swarm coordination"
- "AUV multi-robot communication"
- "Dynamic topology switching consensus"
- "CLF-CBF quadratic programming"
- "Connectivity maintenance graph theory"
- "Robust consensus disturbance"

---

**Pro Tip**: Start with the starred (⭐) papers - they're most directly applicable to your hybrid consensus pruning approach with CLF-CBF control in underwater environments.
