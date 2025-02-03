---
layout: post
title:  Underactuated Robotics - Lecture 11 - Trajectory Optimization (2)
date:   2025-01-31 15:18:00 +0200
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

In the previous lecture, we started discussing *traj opt*. We defined the **direct transcription** as follows:

$$
\begin{aligned}
  \min_{x[\cdot], u[\cdot]} & \sum_{n=0}^{N-1} \ell(x[n],u[n]) + \ell_f(x[N]) &
  \\ \text{s.t. } & &
  \\ & x[n+1] = f(x[n],u[n]) & \forall n \in [0,N-1]
  \\ & x[0] = x_0 &
  \\ & \text{+ other constraints, e.g. } &
  \\ & u[n] \in [-1,1] & \forall n \in [0,N-1]
\end{aligned}
$$

We also discussed **indirect shooting**: since $x[n]$ is completely defined by $x[0]$ and the control sequence $u[0],\dots,u[n-1]$, we don't necessarily need to minimize over the state variables $x[n]$ and we can just plug the control sequence into the dynamics to get the state trajectory whenever needed.

We then discussed approaches to continuous time, where the dynamics are given by $\dot x = f_c(x,u)$. 
The simplest (and common) thing to do is to discretize the problem using Euler integration: $ x[n+1] = x[n] + h f_c(x[n],u[n]) $, where $h$ is the time step. 

What's interesting here is that once we go into nonlinear optimization, we can even minimize over the time step $h$ itself, that is, $ \min_{x[\cdot], u[\cdot], h} (\cdots) $. We can even optimize over $h[\cdot]$ as a sequence, and allow it to vary over time, that is, $ \min_{x[\cdot], u[\cdot], h[\cdot]} (\cdots) $. Note that if we choose to optimize over $h$ as well, we need to be careful about the constraints on $h$ (e.g. $h\in[0.1,0.2]$), because we've given the optimizes "enough rope to hang itself."

### Direct Collocation Transcription

Another transcription method we might use, which is more common in practice and might be more natural, is **direct collocation**. The idea is to approximate the continuous dynamics by interpolating the state trajectory between the time steps.

<p align="center">
  <img src="https://underactuated.csail.mit.edu/data/collocation.svg">
</p>

Given $x[\text{samples}], \dot x [\text{samples}]$, we can define $x(t)$ as a cubic spline of these samples. We can thus constrain that the dynamics are satisfied at the collocation points, that is, $ \dot x(t) = f_c(x(t),u(t)) $ at the collocation points. We can therefore write the optimization problem as:

$$ 
\begin{aligned}
\min_{x[\cdot], u[\cdot]} & \sum_{n=0}^{N-1} h \ell_c (x[n], u[n]) &
\\ \text{s.t. } & \dot x(t) = f_c(x(t),u(t)) & \forall t \in \text{[collocation points]}
\end{aligned}
$$

# Trajectory Stabilization

Consider the Acrobot swing-up problem. 
If we do traj opt, we get trajectories $ x[\cdot], u[\cdot] $ that are optimal for the given cost function. Furthermore, if we did direct collocation, we get a continuous trajectory $ x(t), u(t) $ that is optimal for the given cost function.
However, these policies used constraints that used some numerical integration, and might not actually result in swinging up the acrobot. Certainly, if we have uncertainty about the model or other randomness (e.g. random initial conditions), the trajectory might not work in simulation.

The question is therefore, how do we stabilize the trajectory?

We've already talked about one idea: **Model-Predictive Control (MPC)**. However, MPC requires recursive feasibility, which is not guaranteed in the general case. So we will usually not be using it to find global minima. Nevertheless, there is a very different optimization problem, which is "I have a nominal guess of the trajectory, and I want to go back to the trajectory." In that regime, we're just stabilizing a trajectory as opposed to trying to find one out of thin air. **Stabilization is easier than global optimization**.

In fact, we don't actually need MPC to stabilize a trajectory. MPC is a slight generalization over what a linear controller would do to stabilize a trajectory The key idea is **linearizing around a trajectory**.

### Linearizing around a trajectory

Recall that we discussed the linearization of the pendulum, where we approximated the dynamics around the fixed point (at the top). 
If we linearize around a nominal point $(x_0, u_0)$:

$$ 
\begin{aligned}
  \dot x & = f(x,u) 
  \\ & \approx f(x_0, u_0) + \left. \frac{\partial f}{\partial x} \right\vert_{(x_0,u_0)} (x-x_0) + \left. \frac{\partial f}{\partial u} \right\vert_{(x_0,u_0)} (u-u_0)
\end{aligned}
$$

Since we chose an unconstrained nominal point, we do not necessarily get $ f(x_0, u_0) = 0 $, since $\dot \theta$ is not zero. So we got an affine approximation of the dynamics.

A better way to think about it would be to consider the trajectory. Let's define $x_0 (t), u_0 (t)$ as the nominal trajectory. Let our error coordinates be $ \bar x = x - x_0, \bar u = u - u_0 $. Note that this means that we're moving the coordinate system with the trajectory (e.g., if you stand still, the coordinate system is moving away from you). In the error coordinates, we do get a linear (and not affine) approximation of the dynamics:

$$
  \dot{\bar{x}} = A(t) \bar x + B(t) \bar u
$$

And this is something linear optimal control can easily handle.

### Finite-Horizon LQR for Trajectory Stabilization

$$
\begin{aligned}
  \min_{u(t)} &  \int_0^{t_f} \left( x^T Q x + u^T R u \right) dt & 
  \\ \text{s.t. } & &
  \\ & \dot x = A x + B u &
\end{aligned}
$$

Since we have finite time, the cost-to-go is now also a function of time $J(x,t)$. We thus get the following *time-varying* equations:

$$ 
\begin{aligned}
  J(x,t) = & x^t S(t) x & \text{(Cost-to-go)}
  \\ u(t) = & -K(t) x & \text{(Control)}
  \\ -\dot S(t) = & S(t) A + A^T S(t) - S(t) B R^{-1} B^T S(t) + Q & \text{(Riccati Equation)}
\end{aligned}
$$

This variation of the Riccati equation is known as the **Riccati Differential Equation**. Note that the Riccati equation can be solved offline, and then we can just do some matrix multiplications online.

As it turns out, everything above works for time-varying linear systems! That is, if $A(t), B(t)$ are time-varying, and even if $Q(t), R(t)$ are time-varying, we can still solve the Riccati equation and get the optimal control policy. 