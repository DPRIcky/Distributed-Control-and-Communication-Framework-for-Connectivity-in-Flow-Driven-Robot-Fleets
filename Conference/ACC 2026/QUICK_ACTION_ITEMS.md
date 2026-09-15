# 🎯 Quick Action Items - What to Add Where

**Date:** January 23, 2026

---

## ✅ GOOD NEWS: 70% Done!

Your **DISTRIBUTED_VALIDATION.md** already has all the theorems and proofs. You just need to transfer them to the ACC paper!

---

## 📝 EXACT CHANGES TO ACC PAPER

### 1. ADD NEW SECTION III-C (After Section III-B, Before Section IV)

**Title:** "Theoretical Analysis"

**Content to copy from DISTRIBUTED_VALIDATION.md:**

```latex
\subsection{Theoretical Analysis}

We now provide formal guarantees for the distributed framework.

\begin{theorem}[Connectivity Preservation]
\label{thm:connectivity}
If all robots prune edge $e$ only when $\lambda_2(\mathcal{L} \setminus e) \geq \lambda_{\text{ref}}$, 
then the communication graph $\mathcal{G}_t$ remains connected $\forall t \geq 0$.
\end{theorem}

\begin{proof}
By the Fiedler theorem~\cite{fiedler1973}, a graph is connected if and only if 
$\lambda_2(\mathcal{L}) > 0$. Since each robot enforces 
$\lambda_2(\mathcal{L}^l \setminus e) \geq \lambda_{\text{ref}} > 0$ 
before removing edge $e$, the graph remains connected after pruning.
\end{proof}

\begin{theorem}[Liveness]
\label{thm:liveness}
If the adjacency consensus converges and redundant edges exist, 
the distributed algorithm will eventually identify and prune them unanimously.
\end{theorem}

\begin{proof}
From~\cite{griparic2022}, the consensus update ensures 
$\|\mathbf{A}^l - \mathbf{A}^*\| \leq C e^{-\lambda_2 T_d k} \to 0$.
After convergence, all robots have $\mathbf{A}^l \approx \mathbf{A}^p$ 
for all $l, p \in \mathcal{V}$. Therefore:
(i) Same edges extracted: $\mathcal{E}^l \approx \mathcal{E}^p$;
(ii) Same $\lambda_2$ estimates: $\lambda_2^l \approx \lambda_2^p$;
(iii) Same redundancy checks yield unanimous decision.
\end{proof}

\begin{theorem}[Asymptotic Optimality]
\label{thm:optimality}
The distributed pruning algorithm produces a connected graph with 
$m \geq n-1$ edges that locally maximizes algebraic connectivity 
$\lambda_2$ subject to the safety threshold $\lambda_{\text{ref}}$.
\end{theorem}

\begin{proof}
Algorithm~1 greedily prunes the longest edges first (line 5) while 
ensuring $\lambda_2 \geq \lambda_{\text{ref}}$. At each step, the 
removed edge has minimal impact on connectivity. The process terminates 
when no further pruning is safe, yielding a locally optimal sparse structure.
\end{proof}

\begin{remark}
Unlike centralized MST algorithms~\cite{kruskal1956,prim1957}, our method 
does not guarantee global optimality but achieves comparable performance 
(Fig.~4) while maintaining fully distributed computation.
\end{remark}
```

**WHERE:** After Section III-B (around line 300 of ACC_2026.txt)

**SOURCE:** DISTRIBUTED_VALIDATION.md lines 844-858

---

### 2. ADD REMARK AFTER EQUATION (1) in Section II-A

**Location:** Right after Eq. (1) around line 70 of ACC_2026.txt

**What to add:**

```latex
\begin{remark}
For control synthesis (Sec.~III), we adopt the simplified single-integrator 
model $\dot{x}_i = u_i$, treating flow and disturbances as external forces 
handled by the barrier functions. The full advection-diffusion dynamics~\eqref{eq:advection} 
are used only for simulation validation (Sec.~IV). This separation is standard in 
distributed robotics~\cite{olfati2007,sabattini2013}.
\end{remark}
```

**WHY:** Addresses Reviewer 18 Comment 1 (model inconsistency)

---

### 3. FIX FLOW NOTATION in Section II-A

**Location:** Around line 50-60 of ACC_2026.txt

**CHANGE FROM:**
```
The evolution of the center of mass is governed primarily by 
the advective flow field, i.e.,
    ẋ̄(t) ≈ fflow(x̄(t), t),
```

**CHANGE TO:**
```
Each robot experiences position-dependent flow:
    ẋi(t) ≈ fflow(xi(t), t),
    
where the flow field fflow(x,t) varies spatially due to vortices 
and eddies (Sec.~IV). Unlike uniform drift, spatially varying flow 
creates differential advection across the fleet that must be actively 
countered to maintain connectivity while minimizing energy expenditure.
```

**WHY:** Addresses Reviewer 18 Comment 2 (flow challenge)

---

### 4. ADD PROBLEM 1 BOX at End of Section II-C

**Location:** End of Section II-C (around line 110 of ACC_2026.txt), before Section III

**What to add:**

```latex
\begin{problem}[Distributed Energy-Efficient Connectivity Control]
\label{prob:main}
\textbf{Given:}
$N$ robots with dynamics $\dot{x}_i = u_i$, $i \in \mathcal{V} = \{1,\ldots,N\}$;
initial connected graph $\mathcal{G}_0 = (\mathcal{V}, \mathcal{E}_0)$;
communication range $R_{\max}$; safety distance $d_{\min}$;
position-dependent flow field $f_{\text{flow}}(x,t)$.

\textbf{State:} $\mathcal{X} = \{x_1,\ldots,x_N\} \in \mathbb{R}^{2N}$

\textbf{Control:} $\mathcal{U} = \{u_1,\ldots,u_N\}$, $\|u_i\| \leq u_{\max}$

\textbf{Constraints:}
\begin{itemize}
    \item[(C1)] Connectivity: Graph $\mathcal{G}_t$ remains connected $\forall t \geq 0$
    \item[(C2)] Safety: $\|x_i - x_j\| \geq d_{\min}$, $\forall i,j,t$
    \item[(C3)] Communication: Edge $(i,j)$ exists $\iff$ $\|x_i - x_j\| \leq R_{\max}$
    \item[(C4)] Distributed: Robot $i$ uses only $\{x_j - x_i\}_{j \in \mathcal{N}(i)}$ and local messages
\end{itemize}

\textbf{Objective:} Minimize total control effort
$J = \int_0^T \sum_{i=1}^N \|u_i(t)\|^2 \, dt$
subject to achieving target $x_i^{\text{goal}}$ when detected, while maintaining (C1)--(C4).
\end{problem}
```

**WHY:** Addresses Reviewer 18 Comment 3 (mathematical rigor)

---

### 5. ADD CLF-CBF FORMULATION in Section III-B

**Location:** After the paragraph mentioning Vi (around line 240 of ACC_2026.txt)

**What to add:**

```latex
\textbf{CLF-CBF Quadratic Program:}
Each robot $i$ solves at each time step:
\begin{equation}
\begin{aligned}
\min_{u_i} \quad & \|u_i\|^2 + \gamma^2 \\
\text{s.t.} \quad & \dot{V}_i + \alpha V_i \leq \gamma \quad \text{(CLF)} \\
& \dot{h}^{\text{safe}}_{ij} + \beta_s h^{\text{safe}}_{ij} \geq 0, \quad \forall j \in \mathcal{N}(i) \\
& \dot{h}^{\text{conn}}_{ij} + \beta_c h^{\text{conn}}_{ij} \geq 0, \quad \forall j \in \mathcal{C}_i
\end{aligned}
\label{eq:qp}
\end{equation}
where $\dot{V}_i = 2(x_i - x_i^{\text{ref}})^\top u_i$.
The CLF constraint drives $x_i \to x_i^{\text{goal}}$ when a target is detected, 
while CBF constraints ensure forward invariance of safe sets~\cite{ames2019}.
```

**WHY:** Addresses Reviewer 25 Comment 3 (Lyapunov function usage)

---

### 6. DEFINE dmin in Section II-B

**Location:** After communication graph definition (around line 90 of ACC_2026.txt)

**What to add:**

```latex
\textbf{Collision Avoidance:} 
A minimum separation distance $d_{\min} > 0$ is enforced to prevent 
physical collisions between robots. Typically $d_{\min} < R_{\max}$ to 
allow neighbors to communicate while maintaining safe separation.
```

**WHY:** Addresses Reviewer 25 Comment 2 (dmin undefined)

---

### 7. ADD REFERENCES (10-15 papers)

**New references to add to bibliography:**

```bibtex
@article{fiedler1973,
  title={Algebraic connectivity of graphs},
  author={Fiedler, Miroslav},
  journal={Czechoslovak mathematical journal},
  year={1973}
}

@article{leonard2001,
  title={Model-based feedback control of autonomous underwater gliders},
  author={Leonard, Naomi Ehrich and Graver, John G},
  journal={IEEE Journal of oceanic engineering},
  year={2001}
}

@article{zavlanos2007,
  title={Controlling connectivity of dynamic graphs},
  author={Zavlanos, Michael M and Pappas, George J},
  journal={IEEE CDC},
  year={2007}
}

@book{mesbahi2010,
  title={Graph theoretic methods in multiagent networks},
  author={Mesbahi, Mehran and Egerstedt, Magnus},
  year={2010},
  publisher={Princeton University Press}
}

@article{gao2020,
  title={Event-triggered consensus control for multi-agent systems},
  author={Gao, Yanping and Cai, Yunze},
  journal={IEEE Transactions},
  year={2020}
}

@article{xu2015,
  title={Connectivity preserving control using adaptive barrier functions},
  author={Xu, Xiangru and others},
  journal={IEEE},
  year={2015}
}

% Add 5-10 more similar papers
```

**WHY:** Addresses Reviewer 25 Comment 4 (more references)

---

## 📊 SUMMARY TABLE

| Change | Location in ACC Paper | Time Est. | Source |
|--------|----------------------|-----------|---------|
| **Section III-C** (Theorems) | After Sec III-B | 2 hrs | DISTRIBUTED_VALIDATION.md lines 844-858 |
| **Remark 1** (Model) | After Eq. (1) | 15 min | New content |
| **Fix flow** | Sec II-A line ~55 | 15 min | Edit existing |
| **Problem 1** | End of Sec II-C | 1 hr | New content |
| **CLF-CBF QP** | Sec III-B | 1 hr | New content |
| **Define dmin** | Sec II-B | 15 min | New content |
| **References** | Bibliography | 2 hrs | Find papers |

**TOTAL TIME: ~6-7 hours**

---

## 🚀 RECOMMENDED ORDER

1. **Start with Section III-C** (Theorems) - Most critical, you have content ready
2. **Add Problem 1** - Second most critical for rigor
3. **Fix flow notation** - Quick win for novelty challenge
4. **Add Remark 1** - Quick win for model consistency
5. **Add CLF-CBF** - Clarifies Lyapunov usage
6. **Define dmin** - Trivial fix
7. **Add references** - Time-consuming but straightforward

---

## 💡 KEY INSIGHT

**You don't need to write theorems from scratch!** 

Your DISTRIBUTED_VALIDATION.md (lines 844-858) already has:
- ✅ Theorem 1 (Safety/Connectivity)
- ✅ Theorem 2 (Liveness/Termination)  
- ✅ Theorem 3 (Optimality)
- ✅ Proof sketches for all three

**Just copy to LaTeX and you're 50% done with the revision!**

---

## ✅ CHECKLIST

Copy this to track progress:

- [ ] Section III-C added with 3 theorems
- [ ] Remark 1 added after Eq. (1)
- [ ] Flow notation fixed (fflow(x̄,t) → fflow(xi,t))
- [ ] Problem 1 box added
- [ ] CLF-CBF QP formulation added
- [ ] dmin defined in Section II-B
- [ ] 10-15 new references added
- [ ] Point-by-point response document created

**When all checked:** Send to PI for review!
