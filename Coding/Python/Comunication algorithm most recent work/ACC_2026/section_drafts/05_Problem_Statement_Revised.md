# Problem Statement (Revised)
## Draft for Review - Replaces current Subsection II.C

---

## PURPOSE:
- Formalize the problem statement with references to assumptions and definitions
- Clarify objectives and constraints
- Set up the "Given-Design-Such That" structure

---

## CURRENT STATE (lines 216-243):
Good informal statement, but can be made more rigorous

---

## PROPOSED REVISION:

### Draft Text (LaTeX Format):

```latex
\subsection{Problem Statement}

We now formally state the distributed connectivity-aware control problem that this work addresses.

\paragraph{Given:}
\begin{itemize}
    \item A team of $N$ robots with planar positions $x_i(t) \in \mathbb{R}^2$, $i \in \mathcal{V} = \{1, \ldots, N\}$, evolving according to the control-affine dynamics
    \begin{equation}
    \dot{x}_i(t) = f_{\text{flow}}(x_i, t) + u_i(t) + d_i(t),
    \end{equation}
    subject to Assumptions~\ref{assm:bounded-control}--\ref{assm:range-safety-margin} (bounded control, bounded disturbance, Lipschitz flow, control dominance, initial connectivity, and range-safety margin).
    
    \item An initially connected communication graph $\mathcal{G}_0 = (\mathcal{V}, \mathcal{E}_0)$ with $\lambda_2(\mathcal{L}_0) > 0$.
    
    \item Range-limited communication: edges $(i,j) \in \mathcal{E}_t$ exist if and only if $\|x_i(t) - x_j(t)\| \leq R_{\max}$.
    
    \item Minimum safety separation $d_{\min}$ and communication range $R_{\max}$ satisfying $d_{\min} < R_{\max} - \varepsilon$.
    
    \item Localized information only: each robot $i$ has access to its own state $x_i$, relative states $\{x_j - x_i\}_{j \in \mathcal{N}(i,t)}$ to neighbors, and messages exchanged with neighbors. No global positioning, centralized coordinator, or spectral information (e.g., Laplacian eigenvalues computed globally) is available.
    
    \item Task scenario: locations of interest (targets) are unknown a priori and are detected when a robot enters a sensing footprint. Upon detection, the discovering robot must reach the target while the team maintains overall connectivity.
\end{itemize}

\paragraph{Design:}
A distributed control framework consisting of:
\begin{enumerate}
    \item A \textit{graph layer} that identifies and maintains a sparse critical communication structure $\mathcal{C}_t \subseteq \mathcal{E}_t$ using only neighbor-to-neighbor information exchanges, and
    
    \item A \textit{control layer} where each robot $i$ computes a local control input $u_i(t)$ using only neighbor-relative measurements and the current critical structure.
\end{enumerate}

\paragraph{Such That:}
The framework achieves the following objectives simultaneously:

\begin{enumerate}
    \item \textbf{Safety (Collision Avoidance):} All robot pairs maintain minimum separation:
    \begin{equation}
    \|x_i(t) - x_j(t)\| \geq d_{\min}, \quad \forall\, i \neq j, \; t \geq 0.
    \end{equation}
    
    \item \textbf{Connectivity Preservation:} The communication graph $\mathcal{G}_t$ remains connected for all $t \geq 0$. Equivalently, the algebraic connectivity satisfies
    \begin{equation}
    \lambda_2(\mathcal{L}_t) > 0, \quad \forall\, t \geq 0.
    \end{equation}
    
    \item \textbf{Energy Efficiency:} Control effort is minimized by:
    \begin{itemize}
        \item Exploiting natural flow advection for exploration (robots drift passively when no target is detected),
        \item Maintaining only critical communication links $\mathcal{C}_t$ rather than all possible links $\mathcal{E}_t$, reducing the number of active control constraints, and
        \item Activating goal-seeking control only for robots assigned to detected targets.
    \end{itemize}
    
    \item \textbf{Collective Reconfiguration:} When a target is detected:
    \begin{itemize}
        \item The detecting robot(s) converge to the target location,
        \item The remaining robots act as relays, adjusting their positions to maintain end-to-end connectivity between the target-attending robots and the rest of the fleet, and
        \item The overall network topology adapts dynamically while preserving global connectivity.
    \end{itemize}
    
    \item \textbf{Distributed Implementation:} All computations and decisions are made locally by individual robots using only:
    \begin{itemize}
        \item Own state $x_i$,
        \item Neighbor-relative states $\{x_j - x_i\}_{j \in \mathcal{N}(i,t)}$,
        \item Messages exchanged with neighbors (e.g., neighbor lists, critical edge labels),
        \item Local graph quantities derived from neighbor information (e.g., two-hop neighborhoods).
    \end{itemize}
    No centralized computation, global state, or globally shared spanning tree is assumed.
\end{enumerate}

\textit{Summary:} The core challenge is to design a control framework that balances exploration (dispersion for coverage), exploitation (reaching targets when detected), and connectivity (maintaining a communication backbone)—all in a fully distributed manner under flow disturbances and with limited communication bandwidth. The framework must preserve safety and connectivity guarantees rigorously (addressed via Theorems~\ref{thm:robust-invariance} and~\ref{thm:global-connectivity} in Section~\ref{sec:theory}) while achieving practical energy efficiency (demonstrated in simulation in Section~\ref{sec:result}).
```

---

## PLAIN LANGUAGE VERSION:

### The Problem We're Solving:

**Scenario:**
- You have N underwater robots (e.g., 12 robots)
- They're in a flow-dominated environment (currents, vortices pushing them around)
- They start together (fully connected network)
- They can only talk to nearby neighbors (acoustic communication, limited range)
- Targets appear randomly—when a robot finds one, it should go investigate
- BUT: The whole team must stay connected (so data can get back to base)

**What We Have:**
- Each robot knows:
  - Its own position
  - Relative positions to neighbors (e.g., "Robot 5 is 0.8 meters northeast of me")
  - Messages from neighbors
- Each robot does NOT know:
  - Absolute global positions (no GPS)
  - Positions of non-neighbors
  - Global network topology
  - Anything a central controller would know

**What We Need to Design:**
1. **Smart Graph Pruning:** Algorithm to figure out which communication links are essential vs. redundant
2. **Smart Controllers:** Control law that:
   - Avoids collisions
   - Keeps essential links alive
   - Lets robots drift with flow when safe
   - Moves robots to targets when needed

**Success Criteria:**
1. ✅ **No Crashes:** All robots stay at least $d_{\min}$ apart (e.g., 0.6 m)
2. ✅ **Stay Connected:** Network never breaks into separate groups
3. ✅ **Save Energy:** 
   - Drift with flow when possible (free transport!)
   - Only maintain critical links (not all N² possible links)
   - Only use thrusters when necessary
4. ✅ **Reach Targets:** When target detected, send robot there while keeping network connected
5. ✅ **Fully Distributed:** Each robot makes decisions using only local info

**The Hard Part:**
Balancing all these at once! 
- If you drift too much → lose connectivity
- If you control too much → waste energy
- If you prune too aggressively → network breaks
- If you don't prune → waste communication/computation
- Must handle random flow disturbances

---

## COMPARISON TO CURRENT VERSION:

| Aspect | Current (Informal) | Proposed (Formal) |
|--------|-------------------|-------------------|
| Structure | Paragraph-based | Given-Design-Such That |
| References | General mentions | Explicit assumption/theorem refs |
| Objectives | Listed informally | Numbered with equations |
| Constraints | Implicit | Explicit (safety, connectivity eqns) |
| Distributed aspect | Mentioned | Detailed enumeration of local info |
| Energy efficiency | Mentioned | Decomposed into 3 sub-points |

---

## KEY IMPROVEMENTS:

1. **Explicit assumption references:** "subject to Assumptions 1-6" links to formal foundation
2. **Quantitative objectives:** $\lambda_2(\mathcal{L}_t) > 0$ instead of "graph remains connected"
3. **Clear scope of local information:** Enumerated list of what robots know/don't know
4. **Forward references to theorems:** "addressed via Theorems X and Y" shows paper structure
5. **Summary paragraph:** Helps reader understand the big picture

---

## CROSS-REFERENCES TO ADD:

- Assumptions 1-6: `\ref{assm:bounded-control}` through `\ref{assm:range-safety-margin}`
- Theorem T1: `\ref{thm:robust-invariance}` (safety + connectivity invariance)
- Theorem T3: `\ref{thm:global-connectivity}` (connectivity via critical edges)
- Section III: `\ref{sec:method}` (methodology)
- Section III-C: `\ref{sec:theory}` (theoretical guarantees)
- Section IV: `\ref{sec:result}` (simulation results)

---

## INTEGRATION NOTES:

1. **Replace current subsection II.C** (lines 216-243)
2. **Renumber:** This becomes subsection II.D (after new II.C on graph definitions)
3. **Check forward refs:** Ensure theorem labels match what's in Section III.C
4. **Length:** ~1 page (slightly longer than current, but more precise)

---

## OPTIONAL ENHANCEMENTS (if space permits):

Could add a small table summarizing the information asymmetry:

| Information Type | Available Locally? | Requires Global Comm? |
|-----------------|-------------------|---------------------|
| Own position $x_i$ | ✅ Yes | ❌ No |
| Neighbor positions $\{x_j\}_{j \in \mathcal{N}(i)}$ | ✅ Yes (relative) | ❌ No |
| Neighbor lists | ✅ Yes (via messages) | ❌ No |
| Critical edges in neighborhood | ✅ Yes (via algorithm) | ❌ No |
| Full graph topology | ❌ No | ✅ Yes |
| Laplacian eigenvalues | ❌ No | ✅ Yes |
| Global MST | ❌ No | ✅ Yes |

This emphasizes the distributed nature of the problem.

---

## REVIEWER CONCERNS ADDRESSED:
- ✅ Formal problem statement (general rigor improvement)
- ✅ Clear scope of local vs. global information
- ✅ Explicit connection to theoretical results (T1, T3)
- ✅ Quantifiable objectives

---

## STATUS: ⬜ DRAFT - Ready for Review
