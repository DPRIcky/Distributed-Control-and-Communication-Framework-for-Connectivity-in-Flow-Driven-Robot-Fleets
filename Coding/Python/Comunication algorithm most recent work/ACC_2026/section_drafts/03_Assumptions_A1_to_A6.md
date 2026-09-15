# Formal Assumptions (A1-A6)
## Draft for Review - Replaces current Subsection II.B

---

## PURPOSE:
- Address R1-3 concern: "Statements not rigorous"
- Provide formal foundation for Theorems T1-T4
- Clarify key parameters like $d_{\min}$, $R_{\max}$, flow properties

---

## CURRENT STATE (lines 190-215):
Informal paragraph-style descriptions mixed with disk model explanation

---

## PROPOSED REPLACEMENT:

### Draft Text (LaTeX Format):

```latex
\subsection{Assumptions on Dynamics and Communication}

We now state the formal assumptions that underpin our framework and subsequent theoretical analysis.

\begin{assumption}[Bounded Control]\label{assm:bounded-control}
Each robot has a bounded control authority. Specifically, the control input satisfies
\begin{equation}
\|u_i(t)\| \leq u_{\max}, \quad \forall\, i \in \mathcal{V}, \; t \geq 0,
\end{equation}
where $u_{\max} > 0$ is a known constant representing the maximum thrust or velocity the robot can produce.
\end{assumption}

\begin{assumption}[Bounded Disturbance]\label{assm:bounded-disturbance}
The lumped disturbance term $d_i(t)$, which captures unmodeled drag, turbulence, and stochastic perturbations, is uniformly bounded:
\begin{equation}
\|d_i(t)\| \leq \bar{d}, \quad \forall\, i \in \mathcal{V}, \; t \geq 0,
\end{equation}
where $\bar{d} > 0$ is a known constant.
\end{assumption}

\begin{assumption}[Lipschitz-Continuous Flow Field]\label{assm:lipschitz-flow}
The background flow field $f_{\text{flow}}(x, t) : \mathbb{R}^2 \times \mathbb{R}_{\geq 0} \to \mathbb{R}^2$ is spatially Lipschitz continuous with constant $L_f > 0$:
\begin{equation}
\|f_{\text{flow}}(x_i, t) - f_{\text{flow}}(x_j, t)\| \leq L_f \|x_i - x_j\|, \quad \forall\, x_i, x_j \in \mathbb{R}^2, \; t \geq 0.
\end{equation}
This ensures that nearby robots experience similar but not identical flow velocities, and the flow gradient is bounded.
\end{assumption}

\textit{Remark:} Assumption~\ref{assm:lipschitz-flow} is critical—it ensures that the flow field is spatially varying (i.e., $f_{\text{flow}}(x_i, t) \neq f_{\text{flow}}(x_j, t)$ for $x_i \neq x_j$ in general), which introduces genuine coupling in the relative dynamics between robots. If the flow were uniform (identical for all robots), it would cancel in relative coordinates and connectivity control would be trivial. The Lipschitz bound quantifies the maximum rate at which flow velocity changes over space, which is essential for proving forward invariance of safety and connectivity constraints in Theorem~\ref{thm:robust-invariance}.

\begin{assumption}[Control Dominance]\label{assm:control-dominance}
The control authority is sufficient to counteract the worst-case flow gradient and disturbance within the communication range:
\begin{equation}
u_{\max} > L_f R_{\max} + \bar{d}.
\end{equation}
This guarantees that robots can maintain connectivity and safety constraints even under maximum flow divergence and disturbances.
\end{assumption}

\begin{assumption}[Initial Connectivity]\label{assm:initial-connectivity}
The communication graph at time $t = 0$ is connected, i.e., for every pair of robots $i, j \in \mathcal{V}$, there exists a path of communication links in $\mathcal{G}_0 = (\mathcal{V}, \mathcal{E}_0)$ connecting them.
\end{assumption}

\begin{assumption}[Communication-Safety Range Separation]\label{assm:range-safety-margin}
The minimum safety distance $d_{\min}$ and maximum communication range $R_{\max}$ satisfy
\begin{equation}
d_{\min} < R_{\max} - \varepsilon,
\end{equation}
for some margin $\varepsilon > 0$. This ensures that there exists a feasible interval of distances where robots are safe (not colliding) and still within communication range.
\end{assumption}

\textit{Remark:} The parameter $d_{\min}$ represents the minimum allowable separation distance between any two robots to avoid physical collisions or sensor interference. The margin $\varepsilon$ accounts for controller tracking errors and transient deviations due to disturbances. In our simulations, we use $d_{\min} = 0.6$ m and $R_{\max} = 1.2$ m, giving a comfortable safety margin.

\paragraph{Communication Model}
We adopt the standard range-limited communication model, often called the "disk model" in multi-robot systems. An undirected communication link $(i, j)$ exists at time $t$ if and only if the Euclidean distance between robots $i$ and $j$ does not exceed the communication range:
\begin{equation}
(i, j) \in \mathcal{E}_t \iff \|x_i(t) - x_j(t)\| \leq R_{\max}.
\end{equation}
The resulting time-varying communication graph is $\mathcal{G}_t = (\mathcal{V}, \mathcal{E}_t)$, where $\mathcal{V} = \{1, 2, \ldots, N\}$ is the set of robot indices.

Each robot $i$ can sense and communicate with its neighbors
\begin{equation}
\mathcal{N}(i, t) = \{j \in \mathcal{V} : \|x_i(t) - x_j(t)\| \leq R_{\max}\},
\end{equation}
and can exchange messages (such as neighbor lists, relative positions, and control states) with $j \in \mathcal{N}(i, t)$. No global positioning system, centralized coordinator, or long-range communication infrastructure is assumed. All coordination relies on neighbor-to-neighbor information exchange.
```

---

## PLAIN LANGUAGE SUMMARY:

### Assumption A1: Bounded Control
- **What it means:** Robots can't produce infinite thrust—there's a maximum speed/force
- **Why it matters:** Real underwater thrusters have limits
- **Example:** $u_{\max} = 0.4$ m/s in our simulations

### Assumption A2: Bounded Disturbance
- **What it means:** Random turbulence and drag are unpredictable but not infinite
- **Why it matters:** We can design controllers that handle "reasonable" disturbances
- **Example:** Wave effects, small eddies are bounded in magnitude

### Assumption A3: Lipschitz Flow (CRITICAL!)
- **What it means:** Flow velocity changes smoothly with position—no sudden jumps
- **Why it matters:** THIS IS THE KEY ASSUMPTION that makes flow non-trivial
  - Robot at position A feels flow velocity f(A)
  - Robot at position B feels flow velocity f(B)
  - If A ≠ B, then f(A) ≠ f(B) in general (spatially varying!)
  - The Lipschitz constant $L_f$ bounds how fast flow changes
- **Addresses R1-2:** "Flow term appears trivial" → NO, it's position-dependent!
- **Example:** In a vortex, robots on opposite sides feel opposite flow directions

### Assumption A4: Control Dominance
- **What it means:** Thrusters are powerful enough to fight the flow
- **Why it matters:** Ensures we CAN maintain connectivity even when flow tries to pull robots apart
- **Formula:** Control power > (flow gradient × communication range) + disturbance

### Assumption A5: Initial Connectivity
- **What it means:** We start with all robots connected (e.g., deployed together)
- **Why it matters:** Can't create connectivity from scratch—we preserve what we have
- **Example:** Robots launched from same mothership

### Assumption A6: Range-Safety Margin
- **What it means:** Communication range is bigger than collision avoidance distance
- **Why it matters:** There's a "Goldilocks zone" where robots are safe but can still talk
- **Example:** $d_{\min} = 0.6$ m (don't crash), $R_{\max} = 1.2$ m (can communicate)
  - Robots can be anywhere between 0.6 and 1.2 m apart → safe AND connected

---

## KEY CROSS-REFERENCES TO ADD:

- Assumption A3 → Referenced in Theorem T1 proof (flow gradient in CBF derivative)
- Assumption A4 → Ensures QP feasibility in Section III.B
- Assumption A6 → Used in defining safe and connectivity sets (next section)

---

## COMPARISON TO CURRENT VERSION:

| Current (Informal) | New (Formal) |
|-------------------|--------------|
| "Robots follow single-integrator dynamics" | ✅ Explicit in Remark 1 |
| "A minimum pairwise separation $d_{\min} > 0$ must be maintained" | ✅ Assumption A6 with explicit margin |
| No mention of flow properties | ✅ Assumption A3 with Lipschitz bound |
| No control limits stated formally | ✅ Assumptions A1, A2, A4 |
| Disk model mentioned informally | ✅ Formal equation with $\mathcal{E}_t$ definition |

---

## INTEGRATION NOTES:

1. **Replace entire current subsection II.B** (lines ~190-215)
2. **Add forward references:**
   - "These assumptions are invoked in the proof of Theorem~\ref{thm:robust-invariance} (Section~\ref{sec:theory})"
3. **Notation check:**
   - Ensure $\mathcal{V}$, $\mathcal{E}_t$, $\mathcal{N}(i,t)$ are used consistently throughout paper
4. **Length estimate:** ~1 page with spacing

---

## REFERENCES TO ADD:
- Ames et al. 2019: Control barrier functions (for A1, A2 context)
- Standard multi-robot texts for disk model (e.g., Olfati-Saber 2007)

---

## STATUS: ⬜ DRAFT - Ready for Review

---

## REVIEWER CONCERNS ADDRESSED:
- ✅ R1-2: Lipschitz flow assumption makes spatial variance explicit
- ✅ R1-3: All assumptions now formal with equations
- ✅ R2-2: $d_{\min}$ now clearly defined in A6
- ✅ General: Provides foundation for theorems T1-T4
