---
layout: post
title:  Underactuated Robotics - Lecture 10 - Trajectory Optimization (1)
date:   2025-01-30 16:15:00 +0200
categories: underactuated-robotics lecture
---
<script type="text/javascript" id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js"></script>
<script>
  MathJax = {
    tex: {
      inlineMath: [['$', '$']]
    }
  };
</script>

> This is part of a series of posts on the course [MIT 6.8210: Underactuated Robotics](https://underactuated.csail.mit.edu/Spring2024/index.html) by [Prof. Russ Tedrake](https://locomotion.csail.mit.edu/russt.html). The goal is to document the key concepts and takeaways for future reference. These are <i>not</i> actual lecture notes, but a summary of the key points covered in the lecture and my thoughts on them. The official course material is available here: [lecture videos on Youtube](https://www.youtube.com/playlist?list=PLkx8KyIQkMfU5szP43GlE_S1QGSPQfL9s) \| [lecture notes](https://underactuated.csail.mit.edu)

In the previous lectures, we've been developing a lot of tools: 
- Dynamic programming/optimal control
  - The tabular case (mesh-based approximation, works up to about 5-6 dimensions) 
  - The continuous case and the HJB equation
  - LQR
  - Value iteration with neural networks (need a sampling distribution)
- Lyapunov analysis
  - SOS optimization (works up to about 10-20 dimensions)

After developing all of these tools mentioned above, why do we a new one?
In all of these, we were fighting the curse of dimensionality; the bigger the state space is, the harder it is to find a policy that works for all possible initial conditions. To try and avoid the curse of dimensionality, we won't look for a policy that works for all initial conditions, but rather a policy that works for a **specific initial condition**. This is where we go from policies to **trajectories**.
Today, we're going to go from $\forall x$ to just one $x[0]=x_0$, and everything is easier and better in one dimension.

# Linear Discrete Time Traj. Opt.

1. Choose a finite horizon length $N$.
2. Optimize:
$$
  \begin{aligned}
  \min_{x[\cdot], u[\cdot]} & \sum_{n=0}^{N-1} \ell(x[n],u[n]) + \ell_f(x[N]) & 
  \\ \text{s.t. } & & 
  \\ & x[n+1] = A x[n] + B u[n] & \forall n \in [0,N-1] 
  \\ & x[0] = x_0 &
  \\ & u[n] \in [-1,1] & \forall n \in [0,N-1]
  \\ & \text{+ other constraints}
  \end{aligned}
$$

where:
  - $x[\cdot] = x[0]=x_0, x[1], \dots, x[N]$ denotes the state trajectory.
  - $u[\cdot] = u[0], u[1], \dots, u[N-1]$ denotes the control trajectory.
  - $\ell(x,u)$ is the running cost. 
  - $\ell_f(x)$ is the terminal cost.
  - $x[n+1] = A x[n] + B u[n]$ is the dynamics.


We often choose $\ell$ to be a convex quadratic function, i.e. $\ell(x,u) = x^T Q x + u^T R u$. If we do that, then we have linear constraints and a quadratic objective, which is a convex optimization problem (Quadratic Programming).

But why would we want to solve a quadratic problem instead of using LQR? 

| LQR | Trajectory Optimization |
|---|---|
| Riccati equation | QP |
| Infinite horizon | Finite horizon |
| Suffers from curse of dimensionality | **We can add constraints!** |
| Converges to a fixed point, but doesn't necessarily arrive | We can add a constraint of the form $x[N] = x_f$ |

Is it limiting to have a finite horizon? It might be, but an idea to go back and forth between finite and infinite horizon is to define $\ell_f (x) = x^T S x $ where $S$ is the solution to the Riccati equation (LQR).

### What about $N$?

In the way we formulated our optimization problem, $N$ was not a variable to be optimized. That means that we should know $N$ beforehand. But what if we don't know $N$? 

We can leverage the robustness of our solvers to find $N$. Given an optimization problem, our solvers either find a solution or tell us that the problem is infeasible. Therefore, if we know that the infinite time hosizon problem is feasible, then there must be some $N$ for which the finite horizon problem is feasible, and we search for that $N$, e.g. with a binary search.

### Side Note: from Continuous to Discrete in the Linear Case

Suppose we have a linear continuous-time system: $ \dot x = A_c x + B_c u $. One way to discretize this system is to use Eular integration: $ x[n+1] = x[n] + h \cdot  (A_d x[n] + B_d u[n]) $. We don't have to do that for a linear system, though. We can actually define $A_d = e^{A_c h}$.

### Direct Transcription vs. Direct Shooting

| Method | Decision Variables | Dynamics |
|---|---|---|
| Direct Transcription | $x[\cdot], u[\cdot]$ | $x[n+1] = Ax[n] + Bu[n] $ |
| Direct Shooting | $x_0, u[\cdot]$ | $ x[n] = A^n x_0 + \sum_{k=0}^{n-1} A^{n-k-1} B u[k] $ |

Direct shooting has fewer decision variables, but it is not necessarily better in all cases. For instance, if $N$ is large, then we're taking high powers of $A$ which can be numerically unstable.

# Model-Predictive Control (MPC)

In the introductory discussion to traj opt, we said that we're giving up the ability to find control policies from any state and we limit ourselves to only one initial condition. However, we can overcome this limitation if we can solve traj opt problems quickly:

**Model-Predictive Control (MPC)**:

1. Estimate the current state $\hat x$.
2. Solve (linear) traj opt with $x_0 = \hat x$.
3. Execute $u[0]$.
4. Repeat.

A very important property for MPC is **Recursive Feasibility**, that is, the property that if the optimization problem is feasible at time $t$, then it is feasible at time $t+1$. 

# Nonlinear Trajectory Optimization

Now, suppose we have the following system: $x[n+1] = f(x[n], u[n]) $ for some nonlinear function $f$, possibly non-convex. In fact, both direct transcription and direct shooting still work, but now it will be non-convex optimization problems, so we'll just have to use a non-convex solver, e.g. SNOPT (Sparse Nonlinear Programming).
