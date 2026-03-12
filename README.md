# TFE — Trajectory Function Engine

A physics engine built on a **fundamentally different mathematical object** than every mainstream engine in existence.

At its foundation, TFE is a geometric expression of **Gauss's Principle of Least Constraint** — the theorem that bridges analytical mechanics and the numerical constraint solvers found in every production physics engine. TFE's goal is to make that principle *exact and non-iterative* by working in trajectory function space rather than in sampled frame state.

---

## The Core Idea

Every mainstream physics engine (Unity, Bullet, Box2D, Havok) shares one architecture:

```
state = (position, velocity)
each frame: integrate → detect → resolve → repeat
```

This engine does not do that.

Instead, each body's entire future motion is encoded as a **closed-form trajectory function** — a parabola that is exact for all time under constant gravity:

```js
class TF {
  pos(t) { return p0 + v0*(t-t0) + 0.5*g*(t-t0)^2 }
  vel(t) { return v0 + g*(t-t0) }
}
```

`pos(t)` and `vel(t)` are not simulated — they are **evaluated**. There is no numerical integration. The physics loop does not touch free-flying bodies at all.

Events (collisions) are detected by solving the quadratic:

```
|p_a(t) - p_b(t)| = r_a + r_b
```

This has a closed-form solution. The exact collision time `tc` is computed analytically, placed in a **min-heap**, and processed in temporal order. At each event a new `TF` is seeded with the post-collision velocity. The old function is not deleted — it is kept in the history array `funcs[id]`, enabling **exact time reversal** for free.

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│  TF (Trajectory Function)                               │
│  ─ p0, v0, t0, g baked at creation                     │
│  ─ pos(t), vel(t), KE(t), PE(t), E(t) — all O(1)       │
│  ─ seed(tc, v_new) → new TF, generation++               │
└─────────────────────────────────────────────────────────┘
         │                          │
         ▼                          ▼
┌─────────────────┐      ┌──────────────────────────┐
│  detectSS(a,b)  │      │  detectSW(body, wall)    │
│  Quadratic root │      │  Quadratic root          │
│  → exact tc     │      │  → exact tc              │
└────────┬────────┘      └────────────┬─────────────┘
         │                            │
         ▼                            ▼
┌──────────────────────────────────────────────────────┐
│  Min-Heap  { tc, ia, ib, generation_a, generation_b }│
│  Events sorted by exact physical time                │
└───────────────────────────┬──────────────────────────┘
                            │
                            ▼
┌──────────────────────────────────────────────────────┐
│  processEvents(t_now)                                │
│  ─ pop stale events (generation mismatch)           │
│  ─ resolveSS → Chatterjee-Ruina impulse              │
│  ─ resolveSW → wall reflection                      │
│  ─ seed new TFs, re-predict affected bodies         │
└──────────────────────────────────────────────────────┘
```

### Key Properties

| Property | How it falls out |
|---|---|
| **Zero tunneling** | Collision time is exact — no discrete step can skip past it |
| **Zero energy drift** | No integrator error; energy is computed from closed-form functions |
| **Exact time reversal** | TF history is never deleted; rewind just evaluates `pos(t)` at past `t` |
| **Sparse compute** | Free-flying bodies cost exactly 0 CPU. Physics only fires at collision events |
| **Symmetry cache** | Collisions with identical mass ratio + speed + angle hash to the same impulse |

---

## Theoretical Foundation: Gauss's Principle

TFE is a geometric implementation of **Gauss's Principle of Least Constraint** (1829). The principle states that the true motion of a constrained mechanical system is the one that minimizes the deviation from unconstrained motion, measured in a metric defined by the system's mass distribution.

Formally, given:

- $f : \mathbb{R} \to Q$ — the **unconstrained trajectory function** (free parabolic flight under gravity)
- $\{M_i\}$ — a set of **smooth constraint manifolds** in configuration space $Q$
- $\mathbf{M}$ — the **mass matrix**, which defines the Riemannian metric on $Q$

the true constrained trajectory $\gamma$ is:

$$\gamma = \underset{q \in \bigcap_i M_i}{\arg\min} \; \langle f - q,\; \mathbf{M}(f - q) \rangle$$

In words: project the unconstrained trajectory onto the intersection of all constraint manifolds, using the mass-weighted inner product as the distance measure.

$$\boxed{\gamma = \text{constrain}(f,\; \{M_i\},\; \mathbf{M})}$$

### Mapping to TFE's Architecture

Each component of Gauss's Principle corresponds directly to a concrete part of the engine:

#### 1. The Parabolic Trajectory Function ($f$) — unconstrained forward dynamics

`TF.pos(t)` and `TF.vel(t)` are the closed-form unconstrained trajectory. This is the **prediction step** before any constraint resolution: where would each body go if nothing stopped it?

In traditional engines this step is the explicit Euler or RK4 integration pass. In TFE it is exact — a quadratic polynomial with no truncation error.

#### 2. The Constraint Manifolds ($\{M_i\}$) — bilateral and unilateral constraints

A manifold $M_i$ defines the set of configurations that satisfy constraint $i$. Examples:

| Constraint | Manifold definition |
|---|---|
| Non-penetration (collision) | $\|p_a - p_b\| \geq r_a + r_b$ — half-space boundary in $(p_a, p_b)$ space |
| Rigid rod | $\|p_a - p_b\| = L$ — sphere in $(p_a, p_b)$ space |
| Ball-and-socket joint | $p_a = p_b$ — diagonal subspace |
| Planar constraint | $n \cdot p = d$ — hyperplane |

The **intersection** $\bigcap_i M_i$ is the set of all configurations that satisfy every constraint simultaneously. This is the only region in $Q$ where a valid trajectory may live.

#### 3. The Riemannian Metric ($\mathbf{M}$) — the mass matrix warps space

Deviation is not measured with ordinary Euclidean distance. The configuration space is equipped with a Riemannian metric defined by the system's mass matrix:

$$\|x\|_\mathbf{M}^2 = x^\top \mathbf{M} x$$

This has a direct physical consequence: **heavier bodies are harder to deflect**. Minimizing $\|\gamma - f\|_\mathbf{M}^2$ subject to the constraint manifolds automatically respects momentum conservation — lighter bodies receive larger velocity corrections, heavier bodies smaller ones, in exact proportion to their masses. No separate momentum conservation law needs to be enforced; it falls out of the metric.

In TFE's current collision resolver (`resolveSS`), this metric appears explicitly as the reduced mass:

$$m_r = \frac{m_a \cdot m_b}{m_a + m_b}$$

This is the closed-form mass-metric projection for a single sphere-sphere non-penetration constraint. The generalization to multiple simultaneous constraints is what the constraint manifold algebra needs to provide.

#### 4. The Constrained Trajectory ($\gamma$) — the resolved output

$\gamma$ is the new `TF` seeded by the resolver. It is the **minimum-deviation valid trajectory**: the parabolic path that satisfies all active constraints while departing as little as possible from the unconstrained prediction, where "as little as possible" is measured in the mass metric.

Engines like **MuJoCo** formulate this as a convex optimization problem and solve it numerically every frame using their Linear Complementarity Problem (LCP) solver. **Bullet** and **PhysX** solve it iteratively with Projected Gauss-Seidel. TFE's goal is to solve it **exactly, once, in closed form** — a geometric projection rather than an iterative numeric approximation.

---

## Scenes

| Scene | Demonstrates |
|---|---|
| **CHAOS** | 40 spheres in 3D; speed-colored by velocity magnitude |
| **SNIPER** | Ultra-fast projectiles; classical engines would tunnel, TFE cannot |
| **NEWTON'S CRADLE** | Exact chain propagation through sequential heap events |
| **BILLIARDS** | Equal-mass collisions; watch symmetry cache hit rate saturate |
| **RAINFALL** | Falling spheres cost zero compute until they hit something |

---

## What Is Implemented

- [x] `TF` class — parabolic trajectory functions with baked gravity
- [x] Sphere-sphere exact collision detection (`detectSS` — quadratic root)
- [x] Sphere-wall exact collision detection (`detectSW` — quadratic root)
- [x] Chatterjee-Ruina impulse resolution with tangential friction coefficient
- [x] Min-heap event queue sorted by exact physical time
- [x] Generation tagging — stale events from superseded trajectory functions are silently discarded
- [x] TF history per body — full function chain kept for rewind
- [x] Exact time reversal (REWIND mode) — zero additional cost
- [x] Symmetry cache — keyed on mass ratio, speed class, impact angle, restitution
- [x] Garbage collection of old TF segments outside the rewind window
- [x] Energy tracking — KE + PE computed from closed-form functions
- [x] Live gravity mutation — reseeds all active TFs with new gravity without resetting
- [x] Three.js 3D render with speed-colored spheres, trails, velocity arrows
- [x] Five demo scenes with configurable parameters

---

## What Is Missing

### 1. Constraint Manifold Algebra

This is the central unsolved piece. The current engine handles **free bodies** and **collision events**, but has no mechanism for **persistent constraints** (hinges, rods, sliders, joints, soft bodies, cloth, chains).

Traditional engines solve this iteratively — Gauss-Seidel, sequential impulses, or an LCP solver applied every frame. That is a numerical approximation of the projection $\gamma = \text{constrain}(f, \{M_i\}, \mathbf{M})$. It is incompatible with TFE's architecture because TFE has no per-frame state to correct; correction is baked into the new `TF` at event time.

The correct formulation solves the projection **exactly and once**, producing a closed-form post-constraint trajectory function.

---

#### What Makes the Projection Hard

For a **single** non-penetration constraint (sphere-sphere collision), the mass-metric projection has a closed form. This is what `resolveSS` already computes:

$$J^\top (J \mathbf{M}^{-1} J^\top)^{-1} J \Delta v = \text{impulse}$$

where $J$ is the constraint Jacobian (the contact normal). For one constraint this is a scalar equation. Done.

For **$k$ simultaneous constraints** the projection becomes:

$$\mathbf{M} \ddot{q} = f_\text{ext} - J^\top \lambda, \quad J\ddot{q} = -\dot{J}\dot{q}$$

Solving for $\lambda$ (the Lagrange multipliers / constraint forces) requires inverting $J \mathbf{M}^{-1} J^\top$, a $k \times k$ matrix. This is the LCP that MuJoCo and Bullet solve numerically.

TFE needs to make this **geometric and closed-form**. The key question: for which classes of constraint manifolds does $\bigcap_i M_i$ have a known structure under which the mass-metric projection is analytic?

---

#### The Non-Intersection Property Is the Key

TFE already solves one constraint class geometrically — it just doesn't frame it that way.

Field lines of the trajectory functions cannot intersect. The collision resolver enforces this: **no two trajectories may occupy the same point at the same time**. This is not an ad-hoc rule. It is the **non-intersection condition on the trajectory field**, which is exactly the non-penetration constraint manifold evaluated at its boundary.

```
constraint  =  the condition that prevents field line intersection
            =  tangency condition at the constraint surface
            =  geodesic on the contact manifold
```

A rigid rod between bodies $a$ and $b$ is the constraint $\|p_a(t) - p_b(t)\| = L$ for all $t$. This defines a sphere $S^2$ in $(p_a, p_b)$ space. The rod-constrained trajectory must be a **geodesic on this sphere under the mass metric** — exactly the same geometric condition used for collision resolution, but held continuously at the tangency boundary rather than triggered at a crossing event.

The collision resolver and the constraint resolver are **the same resolver at different boundary conditions**:

| Event type | Manifold | Boundary condition |
|---|---|---|
| Elastic collision | $\|p_a - p_b\| \geq r_a + r_b$ | Crossing inward — impulse, then free |
| Rod constraint | $\|p_a - p_b\| = L$ | Tangency maintained — continuous geodesic |
| Contact resting | $\|p_a - p_b\| \geq r_a + r_b$ | At boundary, zero normal velocity — sleep |

You do not need a separate constraint engine. Constraints are **non-intersection geometry** at the tangency boundary.

---

#### The Stack Is a Composed Manifold

When multiple constraints are active simultaneously, the valid configuration space is:

$$M_\text{stack} = M_1 \cap M_2 \cap \cdots \cap M_k$$

This intersection is itself a manifold (when the constraints are independent — i.e., when the Jacobians $\{J_i\}$ are linearly independent at the point of evaluation). A pendulum is $M_\text{rod} \cap M_\text{gravity\text{-}field}$. A Newton's cradle is $\bigcap_{i=1}^{N} M_{\text{rod}_i} \cap \bigcap_{j=1}^{N-1} M_{\text{contact}_j}$.

The claim is:

> **Constraint manifolds defined by quadratic conditions compose geometrically under the mass metric. The geodesic on the composed manifold is computable in closed form. No iteration.**

If this holds, the entire constraint solver stack collapses to one geometric operation:

```
compose(M_a, M_b) → M_combined       // fiber product of constraint manifolds
geodesic(f, M, M_matrix) → γ         // mass-metric projection, closed form
```

The mathematical question — which is open — is exactly what algebraic structure $\bigcap_i M_i$ has when every $M_i$ is defined by a quadratic condition (all the useful constraints are: distance = $\|p_a - p_b\|^2 = L^2$, angle, planar, ball-socket). Quadratic constraint manifolds are algebraic varieties. Their intersection is a variety. The geodesic problem on an algebraic variety under a quadratic metric has known structure in algebraic geometry — this is the thread to pull.

---

#### Comparison to Existing Theory

| Field | What it gives | What is missing for TFE |
|---|---|---|
| Gauss's Principle (1829) | Correct variational statement of the projection | Does not tell you when the projection is closed-form |
| Riemannian geometry | Geodesics on smooth manifolds | No algebra of composed constraints; no event-driven formulation |
| Dirac constraint mechanics | Constrained Hamiltonian systems, primary/secondary constraint classification | Hamiltonian framing; iterative reduction; not trajectory function algebra |
| Geometric mechanics (Marsden-Ratiu) | Symplectic reduction, Lie group structure, momentum maps | Does not formulate as a trajectory function algebra |
| Algebraic geometry | Structure of intersections of algebraic varieties | Does not address the metric projection problem |

None of them give you `compose(M_a, M_b)` as a rule that produces a manifold whose mass-metric geodesic is analytic. That is the gap.

---

### 2. Rigid Body Rotation

Currently all bodies are point masses (spheres with radius for collision geometry only). Missing:

- Orientation state $SO(3)$ baked into TF
- Angular velocity integration (trivial for free bodies — linear in time)
- Torque from off-center impulses
- Moment of inertia tensor in impulse resolution

The trajectory function for a free rigid body extends naturally:

```
orientation(t) = orientation_0 * exp(omega_0 * (t - t0))   // Lie group form
```

This is still closed-form. The architecture does not change.

---

### 3. Non-Constant Force Fields

The current `TF` bakes in constant gravity. For non-constant fields (drag, magnetic, user-applied forces) the trajectory function is no longer a parabola and `pos(t)` has no simple closed form.

Options:

- **Piecewise linear force**: Sub-divide into segments where force is constant — each segment is a TF. Adds events to the heap.
- **Perturbation theory**: Express the trajectory as a parabola plus a correction term, valid to first order for slowly varying fields.
- **Symplectic integrator as fallback**: For bodies under complex forces, fall back to a high-order symplectic method and re-enter the TF regime when the force returns to constant. The history mechanism already supports this — just seed a new TF.

---

### 4. Broad Phase Culling

Currently `predictFor(id)` does an $O(N)$ scan against all other bodies. For large $N$, this is the bottleneck.

Needed:

- Spatial hash or BVH over trajectory bounding volumes (not point positions — the volumes swept by parabolic arcs over the prediction horizon)
- Trajectory AABB: the parabola $\mathbf{p}(t)$ over $[t_0, t_0 + T]$ has a computable axis-aligned bounding box — use this for culling

---

### 5. Continuous Collision Detection for Rotating Bodies

Once rotation is added, the swept volume of a rotating non-spherical body is complex. Conservative advancement or GJK on trajectory-expanded shapes will be needed.

---

### 6. Sleeping / Deactivation

Bodies that have come to rest (low $KE$, on a resting contact) should be put to sleep — their trajectory function degenerated into a constant. The heap should not contain events for sleeping bodies.

---

## The Next Thing to Build

The single highest-leverage item is the constraint manifold algebra. The path is:

1. **Prove or disprove the closed-form claim for a single distance constraint** — for two equal-mass bodies connected by a rod of length $L$, derive the mass-metric geodesic explicitly. Does it produce a pair of parabolic trajectory functions? Or does the constraint introduce curvature that breaks the parabola? This is the test.

2. **Characterize the `compose` operation for quadratic constraints** — the intersection of two quadric hypersurfaces in $\mathbb{R}^n$ is a variety of known degree. Determine whether the mass-metric projection onto this intersection has a closed-form expression. If yes: the whole stack collapses. If no: identify which subclasses do (e.g., orthogonal constraints, constraints with disjoint support).

3. **Implement a single hinge as a test case** — two spheres, one rod constraint, no iteration. The resolver seeds a new `TF` pair whose trajectories lie on $M_\text{rod}$. Measure: does energy drift? Does the constraint hold exactly? Can you rewind through it?

4. **Extend to a chain** — $N$ bodies, $N-1$ rod constraints. The composed manifold is $\bigcap_{i=1}^{N-1} M_{\text{rod}_i}$. This is a cloth or rope. If step 3 works, this is just `compose` applied $N-2$ times.

If steps 1–3 work, rods, chains, hinges, cloth, and articulated figures follow. If the closed-form projection fails in the general case, the fallback is a **one-shot Newton step** on the LCP rather than iterative Gauss-Seidel — still a fixed-cost operation, still compatible with TFE's event-driven architecture.

---

## Running

Open `index.html` directly in a browser. No build step, no server required. All dependencies (Three.js, OrbitControls) load from CDN.

**Keyboard shortcuts:**

| Key | Action |
|---|---|
| `1`–`5` | Switch scene |
| `Space` / `P` | Pause / resume |
| `R` | Rewind |
| `S` | Slow motion |
| `L` | Live speed |
| `V` | Toggle velocity vectors |
| `Enter` | Reset current scene |
| `+` | Spawn random body |

---

## References

**Foundation**
- Gauss, C.F. (1829). *Über ein neues allgemeines Grundgesetz der Mechanik* — the least-constraint principle; the root theorem of this engine
- Chatterjee, A. & Ruina, A. (1998). *A new algebraic rigid-body collision law* — basis of `resolveSS`

**Constraint mechanics**
- Dirac, P.A.M. (1950). *Generalized Hamiltonian dynamics* — primary/secondary constraint classification
- Udwadia, F.E. & Kalaba, R.E. (1992). *A new perspective on constrained motion* — explicit closed-form for constraint forces via Gauss; closest existing work to TFE's goal
- Udwadia, F.E. & Phohomsiri, P. (2006). *Explicit equations of motion for constrained mechanical systems with singular mass matrices* — handles degenerate mass matrices

**Geometric mechanics**
- Marsden, J.E. & Ratiu, T.S. (1999). *Introduction to Mechanics and Symmetry* — symplectic reduction, Lie group structure
- Arnold, V.I. (1989). *Mathematical Methods of Classical Mechanics* — contact geometry, Lagrangian submanifolds

**Numerical constraint solvers (what TFE replaces)**
- Todorov, E. (2014). *Convex and analytically-invertible dynamics with contacts and constraints: Theory and implementation in MuJoCo* — the convex optimization formulation of Gauss's Principle
- Catto, E. (2009). *Modeling and Solving Constraints* (GDC) — Projected Gauss-Seidel as used in Box2D / Bullet

**Algebraic geometry (the open thread)**
- Harris, J. (1992). *Algebraic Geometry: A First Course* — structure of intersections of algebraic varieties; relevant when constraint manifolds are quadrics
