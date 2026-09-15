# Graph Notation and Constraint Set Definitions
## Draft for Review - NEW Subsection II.C

---

## PURPOSE:
- Address R1-3: "Informal descriptions of graph topology and constraint sets"
- Provide rigorous mathematical definitions before Problem Statement
- Set up notation used in Theorems T1, T3, T4

---

## LOCATION IN PAPER:
NEW subsection after Assumptions (current subsection II.B), BEFORE Problem Statement

---

## DRAFT TEXT:

### LaTeX Format:

```latex
\subsection{Graph Notation and Constraint Sets}

Having established the assumptions on robot dynamics and communication, we now introduce the formal graph-theoretic and set-theoretic notation used throughout this work.

\paragraph{Communication Graph}
The communication topology at time $t$ is represented as an undirected graph
\begin{equation}
\mathcal{G}_t = (\mathcal{V}, \mathcal{E}_t),
\end{equation}
where $\mathcal{V} = \{1, 2, \ldots, N\}$ is the vertex set (robots), and the edge set is defined by the range-limited communication model:
\begin{equation}
\mathcal{E}_t = \bigl\{ (i, j) \in \mathcal{V} \times \mathcal{V} : i < j, \; \|x_i(t) - x_j(t)\| \leq R_{\max} \bigr\}.
\end{equation}
We use the convention that edges are unordered pairs, so $(i, j) = (j, i)$, and self-loops are excluded.

The \textit{degree} of vertex $i$ is the number of neighbors:
\begin{equation}
\deg(i, t) = |\mathcal{N}(i, t)|,
\end{equation}
where $\mathcal{N}(i, t) = \{j \in \mathcal{V} : (i, j) \in \mathcal{E}_t\}$ is the neighbor set.

The graph $\mathcal{G}_t$ is \textit{connected} if for every pair of distinct vertices $i, j \in \mathcal{V}$, there exists a path of edges in $\mathcal{E}_t$ linking $i$ to $j$. Equivalently, the second smallest eigenvalue of the graph Laplacian, $\lambda_2(\mathcal{L}_t)$, is strictly positive (where $\mathcal{L}_t$ is the Laplacian matrix of $\mathcal{G}_t$). This eigenvalue is known as the \textit{algebraic connectivity} of the graph.

\paragraph{Critical Edge Set}
An edge $(i, j) \in \mathcal{E}_t$ is called \textit{critical} (or a \textit{bridge}) if its removal would disconnect the graph $\mathcal{G}_t$. The set of all critical edges is denoted
\begin{equation}
\mathcal{C}_t \subseteq \mathcal{E}_t.
\end{equation}
Critical edges form the essential backbone that maintains global connectivity. Non-critical edges
\begin{equation}
\mathcal{E}_t^- = \mathcal{E}_t \setminus \mathcal{C}_t
\end{equation}
lie within cycles and can potentially be pruned without disconnecting the graph, provided alternative paths exist.

\paragraph{Safe Set (Collision Avoidance)}
To ensure collision-free motion, we define the pairwise safe set for robots $i$ and $j$ as
\begin{equation}
\mathcal{C}_{\text{safe}}^{ij} = \bigl\{ (x_i, x_j) \in \mathbb{R}^2 \times \mathbb{R}^2 : \|x_i - x_j\|^2 \geq d_{\min}^2 \bigr\}.
\end{equation}
The overall safe configuration set for the fleet is the intersection:
\begin{equation}
\mathcal{C}_{\text{safe}} = \bigcap_{i, j \in \mathcal{V}, \, i \neq j} \mathcal{C}_{\text{safe}}^{ij}.
\end{equation}
Any configuration $(x_1, x_2, \ldots, x_N) \in \mathcal{C}_{\text{safe}}$ satisfies the minimum separation constraint between all pairs of robots.

\paragraph{Connectivity Set (Communication Maintenance)}
Similarly, for each pair of robots that must maintain a communication link (typically critical neighbors), we define the connectivity set:
\begin{equation}
\mathcal{C}_{\text{conn}}^{ij} = \bigl\{ (x_i, x_j) \in \mathbb{R}^2 \times \mathbb{R}^2 : \|x_i - x_j\|^2 \leq R_{\max}^2 \bigr\}.
\end{equation}
For the full fleet to remain connected, it suffices that all critical edges remain within their respective connectivity sets:
\begin{equation}
\mathcal{C}_{\text{conn}} = \bigcap_{(i,j) \in \mathcal{C}_t} \mathcal{C}_{\text{conn}}^{ij}.
\end{equation}

\paragraph{Feasible Operating Region}
Combining safety and connectivity requirements, the feasible state space for the team is
\begin{equation}
\mathcal{C}_{\text{feasible}} = \mathcal{C}_{\text{safe}} \cap \mathcal{C}_{\text{conn}}.
\end{equation}
Assumption~\ref{assm:range-safety-margin} ensures that $\mathcal{C}_{\text{feasible}}$ is non-empty: there exists a range of distances $d_{\min} < \|x_i - x_j\| < R_{\max}$ where robots are both safe and connected.

\textit{Remark:} The constraint sets defined above serve as the foundation for the Control Barrier Function (CBF) formulation in Section~\ref{sec:method}. Specifically, the safe set $\mathcal{C}_{\text{safe}}^{ij}$ corresponds to the safety barrier $h_{ij}^{\text{safe}} = \|x_i - x_j\|^2 - d_{\min}^2$, and the connectivity set $\mathcal{C}_{\text{conn}}^{ij}$ corresponds to the connectivity barrier $h_{ij}^{\text{conn}} = R_{\max}^2 - \|x_i - x_j\|^2$. The forward invariance of these sets under the controlled dynamics is established in Theorem~\ref{thm:robust-invariance}.
```

---

## PLAIN LANGUAGE SUMMARY:

### Communication Graph $\mathcal{G}_t$
- **Vertices:** Robots (labeled 1, 2, ..., N)
- **Edges:** Communication links (exist when distance ≤ $R_{\max}$)
- **Time-varying:** The graph changes as robots move
- **Connected:** Every robot can reach every other robot through some path

### Critical Edges $\mathcal{C}_t$
- **Definition:** Edges you CANNOT remove without breaking the network
- **Example:** In a chain A—B—C, both edges are critical (remove either → disconnect)
- **Example:** In a triangle, NO edges are critical (remove any one → still connected)
- **Our Goal:** Identify and preserve only critical edges to save energy/communication

### Safe Set $\mathcal{C}_{\text{safe}}$
- **Definition:** All robot positions where no collisions occur
- **Math:** Distance between any two robots ≥ $d_{\min}$
- **Analogy:** "Personal space" around each robot
- **Controller job:** Keep the team inside this set forever (forward invariance)

### Connectivity Set $\mathcal{C}_{\text{conn}}$
- **Definition:** All robot positions where critical links stay in range
- **Math:** Distance between critical neighbors ≤ $R_{\max}$
- **Analogy:** "Walkie-talkie range" for important connections
- **Controller job:** Keep critical pairs inside this set (maintain network)

### Feasible Region $\mathcal{C}_{\text{feasible}}$
- **Definition:** Safe AND connected (intersection of both sets)
- **Sweet spot:** Not too close (collision), not too far (lose link)
- **Example:** For $d_{\min} = 0.6$ m and $R_{\max} = 1.2$ m:
  - Feasible range: 0.6 m < distance < 1.2 m
  - Too close (<0.6 m): unsafe
  - Too far (>1.2 m): disconnected

---

## VISUAL ANALOGY (for your notes, not in paper):

Think of the constraint sets like zones on a map:
- **Safe Set:** Green zone (robots are separated enough, no crashes)
- **Connectivity Set:** Blue zone (robots are close enough to communicate)
- **Feasible Region:** Overlap of green and blue (sweet spot!)
- **Controller's job:** Keep everyone in the overlap forever, even when flow pushes them around

---

## KEY NOTATION INTRODUCED:

| Symbol | Meaning |
|--------|---------|
| $\mathcal{G}_t$ | Communication graph at time $t$ |
| $\mathcal{V}$ | Vertex set (robots) |
| $\mathcal{E}_t$ | Edge set (communication links) |
| $\mathcal{N}(i,t)$ | Neighbor set of robot $i$ |
| $\deg(i,t)$ | Degree of robot $i$ (number of neighbors) |
| $\mathcal{C}_t$ | Critical edge set (bridges) |
| $\mathcal{E}_t^-$ | Non-critical edges (can be pruned) |
| $\mathcal{C}_{\text{safe}}$ | Safe configuration space |
| $\mathcal{C}_{\text{conn}}$ | Connectivity configuration space |
| $\mathcal{C}_{\text{feasible}}$ | Feasible region (safe ∩ connected) |
| $\lambda_2(\mathcal{L}_t)$ | Algebraic connectivity (Fiedler value) |

---

## CROSS-REFERENCES TO ADD:

- **Forward to Section III.A:** "The critical edge set $\mathcal{C}_t$ is identified distributedly using the method in Section~\ref{sec:prune}"
- **Forward to Section III.B:** "Barrier functions are constructed from these sets in Section~\ref{sec:clf-cbf}"
- **Forward to Theorem T1:** "Forward invariance of $\mathcal{C}_{\text{feasible}}$ is proven in Theorem~\ref{thm:robust-invariance}"
- **Backward to Assumption A6:** "Assumption~\ref{assm:range-safety-margin} ensures $\mathcal{C}_{\text{feasible}} \neq \emptyset$"

---

## FIGURES TO CONSIDER (optional):

Could add a simple diagram showing:
- 3 robots in a line
- Safe zones (circles of radius $d_{\min}$ around each)
- Connectivity zones (circles of radius $R_{\max}$ around each)
- Feasible region highlighted

**Note:** Only add if space permits; text definitions are sufficient.

---

## INTEGRATION CHECKLIST:
- [ ] Review mathematical notation consistency
- [ ] Check that $\mathcal{L}_t$ (Laplacian) is defined or referenced
- [ ] Ensure $\lambda_2$ is explained (many readers unfamiliar with algebraic connectivity)
- [ ] Add equation labels for future cross-referencing
- [ ] Verify set notation matches what's used in Theorems section

---

## LENGTH ESTIMATE:
~0.7-1.0 pages (with equations and spacing)

---

## REVIEWER CONCERNS ADDRESSED:
- ✅ R1-3: Rigorous definitions now provided
- ✅ Foundation for Theorem T1 (CBF invariance of these sets)
- ✅ Foundation for Theorem T3 (connectivity via $\mathcal{C}_t$)
- ✅ Foundation for Theorem T4 (pruning preserves connectivity)

---

## STATUS: ⬜ DRAFT - Ready for Review
