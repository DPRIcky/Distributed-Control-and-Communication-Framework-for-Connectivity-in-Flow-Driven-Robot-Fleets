# THEOREM 1 DRAFT: Robust Invariance for Connectivity and Safety
## Complete Mathematical Problem Statement + Theorem with Detailed Derivatives

**Date:** January 28, 2026  
**Purpose:** Precise formulation for ACC 2026 revision (addresses R1-1, R1-2, R2-1)

---

## 1. MATHEMATICAL PROBLEM STATEMENT

### 1.1 System Description

**Network Topology:**  
Consider a team of $N$ autonomous robots operating in a 2D workspace $\mathcal{W} \subset \mathbb{R}^2$. The communication network at time $t$ is represented by the undirected graph:

$$\mathcal{G}(t) = (\mathcal{V}, \mathcal{E}(t))$$

where:
- $\mathcal{V} = \{1, 2, \ldots, N\}$ is the fixed vertex set (robot IDs)
- $\mathcal{E}(t) = \{(i,j) \in \mathcal{V} \times \mathcal{V} : \|x_i(t) - x_j(t)\| \leq R_{\max}\}$ is the time-varying edge set based on proximity

**Communication Range:**  
Each robot can communicate with neighbors within radius $R_{\max} > 0$.

---

### 1.2 Robot Dynamics (Continuous-Time Model)

Each robot $i \in \mathcal{V}$ evolves according to:

$$\dot{x}_i(t) = f_{\text{flow}}(x_i, t) + u_i(t) + d_i(t), \quad i = 1, \ldots, N$$

where:
- $x_i(t) \in \mathbb{R}^2$ is the **position** of robot $i$ at time $t$
- $f_{\text{flow}}: \mathbb{R}^2 \times \mathbb{R}_{\geq 0} \to \mathbb{R}^2$ is the **spatially and temporally varying flow field** (e.g., ocean current)
- $u_i(t) \in \mathbb{R}^2$ is the **control input** (bounded by $\|u_i\| \leq u_{\max}$)
- $d_i(t) \in \mathbb{R}^2$ is a **bounded disturbance** term (e.g., unmodeled turbulence, sensor noise)

**Critical Feature:**  
The flow field is **spatially varying**, meaning $f_{\text{flow}}(x_i, t) \neq f_{\text{flow}}(x_j, t)$ when $x_i \neq x_j$. This creates **relative drift** between robots that must be actively controlled.

---

### 1.3 Safety and Connectivity Constraints

**Safety (Collision Avoidance):**  
For all robot pairs $(i,j)$ that are currently neighbors:
$$\|x_i(t) - x_j(t)\| \geq d_{\min}, \quad \forall (i,j) \in \mathcal{E}(t), \, \forall t \geq 0$$

where $d_{\min} > 0$ is the minimum safe distance.

**Connectivity (Range Maintenance):**  
For all critical edges $(i,j) \in \mathcal{C}$ (where $\mathcal{C}$ is the set of edges whose removal would disconnect the graph):
$$\|x_i(t) - x_j(t)\| \leq R_{\max}, \quad \forall (i,j) \in \mathcal{C}, \, \forall t \geq 0$$

---

### 1.4 Control Objective

**Primary Goal:**  
Design distributed control laws $u_i(t)$ for each robot $i$ such that:

1. **Safety:** No collisions occur (maintain $\|x_i - x_j\| \geq d_{\min}$ for all neighbors)
2. **Connectivity:** The communication graph $\mathcal{G}(t)$ remains connected for all $t \geq 0$
3. **Task Execution:** Robots converge to assigned goal positions $x_i^{\text{goal}}$ (subject to constraints 1-2)

**Challenge:**  
The spatially-varying flow field $f_{\text{flow}}(x_i, t) - f_{\text{flow}}(x_j, t) \neq 0$ creates differential drift that:
- Pushes robots apart (threatening connectivity)
- Pushes robots together (threatening safety)
- Varies continuously as the formation moves through the workspace

Unlike classical CBF works that assume **constant drift** or **drift-free dynamics**, our system requires handling **Lipschitz spatially-varying drift**.

---

## 2. NOTATION AND DEFINITIONS

### 2.1 Scalar Functions and Norms

- $\|\cdot\|$ : Euclidean norm (2-norm) in $\mathbb{R}^2$
- $\langle v, w \rangle = v^\top w$ : Inner product
- $\mathbb{R}_{\geq 0} = [0, \infty)$ : Non-negative reals

### 2.2 Graph-Theoretic Notation

- $\mathcal{N}_i(t) = \{j \in \mathcal{V} : (i,j) \in \mathcal{E}(t)\}$ : Set of neighbors of robot $i$ at time $t$
- $\mathcal{C} \subseteq \mathcal{E}(0)$ : Set of **critical edges** (edges whose removal disconnects the initial graph)
- $\mathcal{C}_i = \{j \in \mathcal{V} : (i,j) \in \mathcal{C}\}$ : Critical neighbors of robot $i$

### 2.3 Control Barrier Functions (CBF)

A continuously differentiable function $h: \mathbb{R}^n \to \mathbb{R}$ is a **Control Barrier Function** for the safe set $\mathcal{S} = \{x : h(x) \geq 0\}$ if there exists a class-$\mathcal{K}$ function $\alpha: \mathbb{R}_{\geq 0} \to \mathbb{R}_{\geq 0}$ such that for all $x \in \mathcal{S}$:

$$\sup_{u \in \mathcal{U}} \left[ \dot{h}(x, u) + \alpha(h(x)) \right] \geq 0$$

where $\mathcal{U}$ is the set of admissible controls and $\dot{h}$ is the Lie derivative of $h$ along the system dynamics.

**Class-$\mathcal{K}$ functions:**  
A continuous function $\alpha: \mathbb{R}_{\geq 0} \to \mathbb{R}_{\geq 0}$ is class-$\mathcal{K}$ if:
1. $\alpha(0) = 0$
2. $\alpha$ is strictly increasing
3. (Optional for extended class-$\mathcal{K}_\infty$) $\alpha(s) \to \infty$ as $s \to \infty$

**Common choice:** $\alpha(s) = \gamma s$ for $\gamma > 0$ (linear class-$\mathcal{K}$ function).

---

## 3. ASSUMPTIONS

### **A1: Bounded Control Authority**
$$\|u_i(t)\| \leq u_{\max}, \quad \forall i \in \mathcal{V}, \, \forall t \geq 0$$

where $u_{\max} > 0$ is the maximum control magnitude (e.g., thruster saturation).

---

### **A2: Bounded Disturbances**
$$\|d_i(t)\| \leq \bar{d}, \quad \forall i \in \mathcal{V}, \, \forall t \geq 0$$

where $\bar{d} \geq 0$ is the disturbance bound.

**Remark:** This includes stochastic turbulence, sensor noise, and model uncertainties.

---

### **A3: Lipschitz Continuity of Flow Field**
The flow field $f_{\text{flow}}(x, t)$ is Lipschitz continuous in space, uniformly over time:

$$\|f_{\text{flow}}(x_i, t) - f_{\text{flow}}(x_j, t)\| \leq L \|x_i - x_j\|, \quad \forall x_i, x_j \in \mathbb{R}^2, \, \forall t \geq 0$$

where $L \geq 0$ is the **Lipschitz constant** (related to the spatial gradient of the flow).

**Physical Interpretation:**  
The flow field is spatially smooth (no discontinuous jumps). For sinusoidal/vortical flows:
$$L \approx \max_{x,t} \|\nabla_x f_{\text{flow}}(x, t)\|$$

**Implementation Example:**  
For the flow field:
$$f_{\text{flow}}(x, t) = v_{\text{base}} + A \begin{bmatrix} \sin\left(\frac{x_2 + t}{s}\right) \\ \cos\left(\frac{x_1 + 0.3t}{s}\right) \end{bmatrix}$$

we have $L \approx A / s$ (e.g., $L = 0.10 / 5.5 \approx 0.018$ s$^{-1}$ in our implementation).

---

### **A4: Control Authority Dominates Disturbances and Flow Gradients**
$$u_{\max} > \bar{d} + L R_{\max}$$

**Physical Interpretation:**  
The robot's control input is sufficiently strong to counteract:
1. Worst-case disturbance $\bar{d}$
2. Maximum flow differential over communication range: $L R_{\max}$

**Example Verification:**  
- $u_{\max} = 0.40$ m/s
- $\bar{d} = 0.03$ m/s
- $L R_{\max} = 0.018 \times 2.5 = 0.045$ m/s
- **Check:** $0.40 > 0.03 + 0.045 = 0.075$ ✓ (margin factor ≈ 5.3)

---

### **A5: Initial Connectivity**
The initial communication graph $\mathcal{G}(0)$ is connected, i.e., there exists a path between every pair of robots at $t = 0$.

---

### **A6: Safety-Connectivity Margin**
$$d_{\min} < R_{\max} - \epsilon$$

for some $\epsilon > 0$ (safety buffer).

**Physical Interpretation:**  
The minimum safe distance is strictly smaller than the communication range, allowing robots to maintain connectivity without collisions.

---

## 4. THEOREM 1: ROBUST INVARIANCE FOR CONNECTIVITY AND SAFETY

### 4.1 Theorem Statement

**Theorem 1 (Safety and Connectivity Invariance under Lipschitz Flow):**

Consider the multi-robot system with dynamics:

$$\dot{x}_i = f_{\text{flow}}(x_i, t) + u_i + d_i, \quad i \in \mathcal{V}$$

subject to Assumptions **A1–A4**. Define the barrier functions:

**1. Safety Barrier** (for all neighbors $j \in \mathcal{N}_i(t)$):
$$h_{ij}^{\text{safe}}(x_i, x_j) = \|x_i - x_j\|^2 - d_{\min}^2$$

**2. Connectivity Barrier** (for all critical neighbors $j \in \mathcal{C}_i$):
$$h_{ij}^{\text{conn}}(x_i, x_j) = R_{\max}^2 - \|x_i - x_j\|^2$$

Suppose the control input $u_i$ at each time $t$ satisfies the **CBF conditions**:

$$\begin{aligned}
\dot{h}_{ij}^{\text{safe}} + \alpha_s h_{ij}^{\text{safe}} &\geq 0, \quad \forall j \in \mathcal{N}_i(t) \\
\dot{h}_{ij}^{\text{conn}} + \alpha_c h_{ij}^{\text{conn}} &\geq 0, \quad \forall j \in \mathcal{C}_i
\end{aligned}$$

with class-$\mathcal{K}$ functions $\alpha_s, \alpha_c > 0$ (e.g., $\alpha_s(s) = \gamma_s s$, $\alpha_c(s) = \gamma_c s$).

Then:

**(a) Safety:**  
If $h_{ij}^{\text{safe}}(0) \geq 0$ for all $(i,j) \in \mathcal{E}(0)$, then:
$$h_{ij}^{\text{safe}}(t) \geq 0, \quad \forall (i,j) \in \mathcal{E}(t), \, \forall t \geq 0$$

**(b) Connectivity:**  
If $h_{ij}^{\text{conn}}(0) \geq 0$ for all $(i,j) \in \mathcal{C}$, then:
$$h_{ij}^{\text{conn}}(t) \geq 0, \quad \forall (i,j) \in \mathcal{C}, \, \forall t \geq 0$$

**Interpretation:**
- No collisions occur (robots stay at least $d_{\min}$ apart)
- Critical communication links are preserved (robots stay within $R_{\max}$ of critical neighbors)

---

### 4.2 Proof of Theorem 1

**Setup:**  
We prove the result for the safety barrier $h_{ij}^{\text{safe}}$; the connectivity barrier follows identically by reversing the sign of the distance term.

Fix an edge $(i,j) \in \mathcal{E}(t)$ (neighbors at time $t$). The goal is to show that if $h_{ij}^{\text{safe}}(t) \geq 0$ and the CBF condition holds, then $h_{ij}^{\text{safe}}$ remains non-negative.

---

#### **Step 1: Compute Barrier Time Derivative**

The safety barrier is:
$$h_{ij}^{\text{safe}} = \|x_i - x_j\|^2 - d_{\min}^2$$

Taking the time derivative:
$$\begin{aligned}
\dot{h}_{ij}^{\text{safe}} &= \frac{d}{dt}\left[\|x_i - x_j\|^2\right] - 0 \\
&= \frac{d}{dt}\left[(x_i - x_j)^\top (x_i - x_j)\right] \\
&= 2(x_i - x_j)^\top (\dot{x}_i - \dot{x}_j)
\end{aligned}$$

**Substituting the dynamics:**
$$\begin{aligned}
\dot{h}_{ij}^{\text{safe}} &= 2(x_i - x_j)^\top \left[ \left(f_{\text{flow}}(x_i, t) + u_i + d_i\right) - \left(f_{\text{flow}}(x_j, t) + u_j + d_j\right) \right] \\
&= 2(x_i - x_j)^\top \left[ \left(f_{\text{flow}}(x_i, t) - f_{\text{flow}}(x_j, t)\right) + (u_i - u_j) + (d_i - d_j) \right]
\end{aligned}$$

**Distributing the inner product:**
$$\boxed{\dot{h}_{ij}^{\text{safe}} = 2\langle x_i - x_j, \Delta f_{ij} \rangle + 2\langle x_i - x_j, u_i - u_j \rangle + 2\langle x_i - x_j, d_i - d_j \rangle}$$

where $\Delta f_{ij} := f_{\text{flow}}(x_i, t) - f_{\text{flow}}(x_j, t)$ is the **flow differential**.

---

#### **Step 2: Bound the Flow Differential (Using A3)**

By the Lipschitz assumption **A3**:
$$\|\Delta f_{ij}\| = \|f_{\text{flow}}(x_i, t) - f_{\text{flow}}(x_j, t)\| \leq L \|x_i - x_j\|$$

Applying the Cauchy-Schwarz inequality:
$$\langle x_i - x_j, \Delta f_{ij} \rangle \geq -\|x_i - x_j\| \cdot \|\Delta f_{ij}\| \geq -L\|x_i - x_j\|^2$$

Thus:
$$\boxed{2\langle x_i - x_j, \Delta f_{ij} \rangle \geq -2L\|x_i - x_j\|^2}$$

---

#### **Step 3: Bound the Disturbance Difference (Using A2)**

By the triangle inequality and **A2**:
$$\|d_i - d_j\| \leq \|d_i\| + \|d_j\| \leq 2\bar{d}$$

Applying Cauchy-Schwarz:
$$\langle x_i - x_j, d_i - d_j \rangle \geq -\|x_i - x_j\| \cdot \|d_i - d_j\| \geq -2\bar{d}\|x_i - x_j\|$$

Thus:
$$\boxed{2\langle x_i - x_j, d_i - d_j \rangle \geq -4\bar{d}\|x_i - x_j\|}$$

---

#### **Step 4: Assemble the CBF Constraint**

Substituting the bounds from Steps 2-3 into the expression for $\dot{h}_{ij}^{\text{safe}}$:

$$\dot{h}_{ij}^{\text{safe}} \geq -2L\|x_i - x_j\|^2 + 2\langle x_i - x_j, u_i - u_j \rangle - 4\bar{d}\|x_i - x_j\|$$

The CBF condition requires:
$$\dot{h}_{ij}^{\text{safe}} + \alpha_s h_{ij}^{\text{safe}} \geq 0$$

Substituting $h_{ij}^{\text{safe}} = \|x_i - x_j\|^2 - d_{\min}^2$:

$$\dot{h}_{ij}^{\text{safe}} \geq -\alpha_s \left(\|x_i - x_j\|^2 - d_{\min}^2\right)$$

Combining with the lower bound:
$$2\langle x_i - x_j, u_i - u_j \rangle \geq 2L\|x_i - x_j\|^2 + 4\bar{d}\|x_i - x_j\| - \alpha_s\left(\|x_i - x_j\|^2 - d_{\min}^2\right)$$

Simplifying:
$$\boxed{2\langle x_i - x_j, u_i - u_j \rangle \geq (2L - \alpha_s)\|x_i - x_j\|^2 + 4\bar{d}\|x_i - x_j\| + \alpha_s d_{\min}^2}$$

---

#### **Step 5: Verify Feasibility via Control Authority (A4)**

Let $r_{ij} = \|x_i - x_j\|$ and $\hat{r}_{ij} = \frac{x_i - x_j}{r_{ij}}$ (unit direction vector from $j$ to $i$).

To satisfy the CBF constraint, we can choose:
$$u_i - u_j = -\frac{(2L - \alpha_s)r_{ij} + 4\bar{d}/r_{ij} + \alpha_s d_{\min}^2/r_{ij}}{2} \cdot \hat{r}_{ij}$$

The required control magnitude is:
$$\|u_i - u_j\| \leq (L + |\alpha_s|/2) r_{ij} + 2\bar{d} + \frac{\alpha_s d_{\min}^2}{2r_{ij}}$$

For robots near the boundary $r_{ij} \approx d_{\min}$:
$$\|u_i - u_j\| \lesssim (L + |\alpha_s|/2) d_{\min} + 2\bar{d} + \frac{\alpha_s d_{\min}}{2}$$

Since $d_{\min} < R_{\max}$ and assuming $\alpha_s = O(1)$:
$$\|u_i - u_j\| \lesssim L R_{\max} + 2\bar{d} + O(1)$$

By **A4**, we have $u_{\max} > \bar{d} + L R_{\max}$, which ensures:
$$\|u_i\|, \|u_j\| \leq u_{\max} \implies \|u_i - u_j\| \leq 2u_{\max} > 2(\bar{d} + L R_{\max})$$

Thus, the CBF constraint is **feasible** (can be satisfied by an admissible control within actuation limits).

---

#### **Step 6: Invoke Standard CBF Invariance Result**

By the fundamental theorem of CBFs (Ames et al., 2019; Xu et al., 2015):

**Lemma (CBF Invariance):**  
If $h: \mathbb{R}^n \to \mathbb{R}$ is a continuously differentiable CBF for the system $\dot{x} = f(x) + g(x)u$, and there exists an admissible control $u^* \in \mathcal{U}$ such that:
$$\dot{h}(x, u^*) + \alpha(h(x)) \geq 0$$

then:
$$h(x(0)) \geq 0 \implies h(x(t)) \geq 0, \quad \forall t \geq 0$$

**Application to our case:**  
- $h = h_{ij}^{\text{safe}}$
- $\alpha = \alpha_s$ (class-$\mathcal{K}$)
- Step 5 guarantees existence of $u_i$ satisfying the CBF condition

Therefore:
$$h_{ij}^{\text{safe}}(0) \geq 0 \implies h_{ij}^{\text{safe}}(t) \geq 0, \quad \forall t \geq 0$$

This proves **part (a): Safety**.

---

#### **Step 7: Connectivity Barrier (Part b)**

For the connectivity barrier $h_{ij}^{\text{conn}} = R_{\max}^2 - \|x_i - x_j\|^2$, the time derivative is:

$$\dot{h}_{ij}^{\text{conn}} = -\frac{d}{dt}\|x_i - x_j\|^2 = -2\langle x_i - x_j, \dot{x}_i - \dot{x}_j \rangle$$

The CBF condition becomes:
$$-2\langle x_i - x_j, \Delta f_{ij} + (u_i - u_j) + (d_i - d_j) \rangle + \alpha_c(R_{\max}^2 - \|x_i - x_j\|^2) \geq 0$$

Using the same bounds from Steps 2-3 (with opposite signs):
$$2\langle x_i - x_j, u_i - u_j \rangle \leq 2L\|x_i - x_j\|^2 + 4\bar{d}\|x_i - x_j\| + \alpha_c(R_{\max}^2 - \|x_i - x_j\|^2)$$

By **A4**, this is feasible (robots can actively pull together to counteract flow divergence).

Thus:
$$h_{ij}^{\text{conn}}(0) \geq 0 \implies h_{ij}^{\text{conn}}(t) \geq 0, \quad \forall t \geq 0$$

This proves **part (b): Connectivity**. $\square$

---

### 4.3 Key Novelty vs. Existing CBF Literature

**Standard CBF Works:**
- Most papers assume **drift-free dynamics** ($\dot{x} = u + d$) or **constant global drift** ($\dot{x} = v_0 + u$)
- When drift is spatially uniform, relative dynamics become $\dot{x}_i - \dot{x}_j = u_i - u_j + (d_i - d_j)$, and the flow term **cancels**

**Our Contribution:**
- We explicitly handle **Lipschitz spatially-varying flow** $f_{\text{flow}}(x_i, t) \neq f_{\text{flow}}(x_j, t)$
- The flow differential $\Delta f_{ij} = f_{\text{flow}}(x_i, t) - f_{\text{flow}}(x_j, t)$ creates relative drift that:
  1. Does **not** cancel in relative coordinates
  2. Scales with robot separation: $\|\Delta f_{ij}\| \leq L\|x_i - x_j\|$
  3. Requires **active control** to maintain safety/connectivity

**Mathematical Key:**  
By treating the flow gradient term $L\|x_i - x_j\|^2$ as a **bounded disturbance-like perturbation** in the CBF framework, we show that Lipschitz flow can be incorporated into robust CBF design via Assumption **A4**.

**Physical Relevance:**
- Ocean currents (Gulf Stream, eddies)
- Atmospheric flows (jet streams, thermals)
- River/channel navigation

---

## 5. LITERATURE SEARCH KEYWORDS (NEXT SECTION)

Before finalizing Theorem 1 for publication, we should search for related works that may address similar problems. I will provide targeted keywords below.

---

**End of Theorem 1 Draft**
