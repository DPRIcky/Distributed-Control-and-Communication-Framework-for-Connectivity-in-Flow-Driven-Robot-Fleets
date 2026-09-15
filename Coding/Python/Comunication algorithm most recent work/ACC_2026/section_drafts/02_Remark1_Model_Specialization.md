# Remark 1: Model Specialization
## Draft for Review

---

## PURPOSE:
Addresses Reviewer R1-1 concern about model confusion (advection-diffusion vs. control-affine model)

---

## LOCATION IN PAPER:
Insert AFTER equation (1) (the advection-diffusion model), BEFORE subsection II.B

Current line ~177 in ACC_2026.tex

---

## DRAFT TEXT:

### LaTeX Format:

```latex
\begin{remark}[Model Specialization]\label{rem:model}
The advection-diffusion model introduced above describes the natural physical behavior of passive objects in a flow field and provides the foundation for understanding how uncontrolled robots disperse in the underwater environment. For the purpose of control synthesis and theoretical analysis in Section~\ref{sec:method}, we adopt a simplified control-affine representation:
\begin{equation}\label{eq:control-model}
\dot{x}_i(t) = f_{\text{flow}}(x_i, t) + u_i(t) + d_i(t),
\end{equation}
where $f_{\text{flow}}(x_i, t)$ is the advective flow field evaluated at the robot's current position $x_i$, $u_i(t) \in \mathbb{R}^2$ is the control input, and $d_i(t)$ represents a bounded disturbance term that absorbs unmodeled drag effects, stochastic turbulence, and other perturbations, with $\|d_i(t)\| \leq \bar{d}$ for all $t$.

This control-affine form retains the essential flow coupling—the fact that each robot experiences position-dependent advection—while enabling the application of Control Lyapunov Function (CLF) and Control Barrier Function (CBF) methods for provably safe and goal-directed control. Throughout the remainder of this paper, theoretical results and control design are based on model~\eqref{eq:control-model}, while simulation experiments incorporate the full advection-diffusion dynamics to demonstrate robustness under realistic conditions.
\end{remark}
```

---

## PLAIN LANGUAGE VERSION (for your notes):

**Remark 1: Why Two Models?**

We use two related but distinct models in this paper:

1. **Advection-Diffusion Model (Physics/Simulation):**
   - This is the "real" model that describes how objects actually move in water
   - Includes: drift with flow, drag forces, random Brownian-like turbulence
   - We use this in simulations to test our method under realistic conditions
   - Equation: $dx_i = u_i dt + f_{\text{flow}} dt + F_{\text{drag}} dt + dW_i$

2. **Control-Affine Model (Theory/Control Design):**
   - This is a simplified version for designing controllers and proving theorems
   - Combines drag, turbulence, and other effects into one bounded disturbance term $d_i$
   - Makes the math tractable for CLF-CBF framework
   - Equation: $\dot{x}_i = f_{\text{flow}}(x_i, t) + u_i + d_i$

**Key Point:** The control model is NOT contradicting the physics model—it's an abstraction that keeps the essential features (position-dependent flow, bounded disturbances) while enabling formal analysis. The simulations use the full physics model to show our control strategy works in practice.

---

## WHY THIS MATTERS:

**Reviewer Concern (R1-1):**
> "Paper uses inconsistent models (advection-diffusion vs. single-integrator)"

**Our Response:**
- NOT inconsistent—they serve different purposes
- Advection-diffusion = what we simulate
- Control-affine = what we analyze theoretically
- Remark 1 makes this explicit and clear

---

## KEY TERMINOLOGY TO DEFINE:

- **Control-affine form:** A system where control enters linearly: $\dot{x} = f(x) + g(x)u$
- **Bounded disturbance:** A term $d(t)$ with known upper bound $\|d(t)\| \leq \bar{d}$
- **Flow coupling:** The dependence of $f_{\text{flow}}$ on position $x_i$ (spatially varying)

---

## INTEGRATION CHECKLIST:
- [ ] Review and edit language
- [ ] Add equation reference labels
- [ ] Ensure $\bar{d}$ is defined (it will be in Assumption A2)
- [ ] Check that Section III references this remark when introducing CLF-CBF

---

## STATUS: ⬜ DRAFT - Ready for Review
