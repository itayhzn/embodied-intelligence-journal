---
layout: post
title:  Underactuated Robotics - Lecture 3
date:   2025-01-12 13:49:00 +0200
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

In the previous lecture, we explored the dynamics of a simple pendulum. We derived the equations of motion and discussed the stability of the system. In this lecture, we will continue our discussion on the simple pendulum and explore how we can control it, that is, what happens when $ u $ stops being 0. How does that change the dynamics of the system? Or better yet, how should we change $u$ to make the pendulum go to a specific point?

In this lecture we will discuss Dynamic Programming. The agenda will be as follows:
 - We will talk about the philosophy of control as an optimization problem.
 - We will see an example of the Double Integrator.
 - We will discuss Numerical Dynamic Programming.
 - We will talk about the intuition behind value functions.
 - And finally, we will see how to solve the pendulum problem using Dynamic Programming.

# Review of the Simple Pendulum

- The dynamics of the simple pendulum are given by: $ m \ell ^2 \ddot{\theta} + b \dot \theta + mg\ell \sin\theta = u $ .
- When the system is undamped, the graphical representation of $ \dot \theta $ vs $\theta$ is given by "concentric eyes". When the system is damped, the eyes become collasing "spirals".
- We talked about one idea for control: **feedback cancellation**.
   - Example 1: Choose $ u = 2mgl \sin\theta $. We get $ m \ell ^2 \ddot{\theta} + b \dot \theta - mg\ell \sin\theta = 0 $, i.e. we flipped the gravity. This causes the "eyes" will shift to the right. The motor should have sufficient torque to keep the pendulum in the upright position. If insufficient torque is applied, $ u = sat(2mgl\sin \theta, -1, +1) $.

# Control as Optimization Problem

First, note the following interesting observation about control in general: consider our drawings in which we plotted the vector field of the dynamics in the $ \dot \theta $ vs $\theta$ plane. In our plots, for every point $ [\theta, \dot \theta]^T $ we drew the vector $[ \dot \theta, \ddot \theta ]^T$. Notice that we do not control the $x$ axis of every vector, it is always $ \dot \theta $. We only control the $y$ axis, i.e. the acceleration. This is a general property of control systems. We can only control the acceleration, not the velocity. This is a fundamental property of control systems.

The philosophy of control as an optimization problem is as follows: 
Given trajectory $ (x(\cdot), u(\cdot)) $, assign a **score** (scalar) to the trajectory. (in RL, this is called the **reward**).
Many optimization formulations admit constraints.

- Examples: 
   - Score function: time to reach the goal, energy consumed, average distance, etc.
   - Constraints: torque limits (e.g. $ \Vert u \Vert \leq 1 $), velocity limits, at time $ t_{\text{final}} $ we have reached $ x \left( t_{\text{final}} \right) = x_{\text{goal}} $.

# Example: The Minimum Time for Double Integrator

Consider the double integrator system: $ \ddot q = u $, where $ | u | \leq 1 $.
The mechanical analog of the double integrator is a brick sliding on ice. The control input is the force applied to the brick. The state of the system is the position of the brick. The dynamics of the system are given by $ \ddot q = u $. The goal is to move the brick from $ q_0 $ to $ q_{\text{goal}} $ in minimum time (and stop at the goal, not just slide past it).

<p align="center">
  <img src="https://underactuated.csail.mit.edu/figures/double_integrator_brick.svg">
</p>

The optimal policy, usually referred to as the **bang-bang control**, is to apply maximum acceleration in the direction of the goal, and then "break at the lask minute" (this, of course, assumes that the initial velocity is 0). 

To solve this formally, let's plot the phase portrait of $ \ddot q = u $ given the constraint $ |u| \leq 1 $ . 
- First, consider the extreme case where $ u = -1 $ , i.e. maximum deceleration. In this case: $ \dot q (t) = \dot q (0) + \ddot q t = \dot q(0) - t $ and $ q (t) = q(0) + \dot q (0) t - \frac{1}{2} \ddot q t^2  = q(0) + \dot q (0) t - \frac{1}{2}t^2 $. This is a parabola with a negative coefficient for the quadratic term.

<p align="center">
  <img src="https://underactuated.csail.mit.edu/figures/double_integrator_orbits.svg">
</p>

- Now, consider the case where $ u = 1 $ , i.e. maximum acceleration. This will give us a parabola with a positive coefficient for the quadratic term, i.e. rightward opening parabola.
- This gives rise to the following optimal solution trajectories:

<p align="center">
  <img src="https://underactuated.csail.mit.edu/figures/double_integrator_mintime_orbits.svg">
</p>

In some way, minimum-time problems are like shortest-path problems (e.g. in graph search), thus we can use similar techniques to solve them. Consider, for instance, taking the phase portrait in the $ \dot q $ vs $ q$ plane, and make it "discrete" by discretizing the space into a grid. We can then use dynamic programming to find the optimal path from the initial state to the goal state.

### Dynamic Programming
Given discrete states $ s_i \in S $, discrete actions $ a_j \in A $, a transition/time function $ s[n+1] = f(s[n], a[n]) $, and an "edge cost" $ g(s,a) $, the total cost of some trajectory will be $ \sum_{n=0}^{N-1} g(s[n], a[n]) $.  The key idea here is that this is an **additive cost**. 

For the *minimum time* cost, we can define the cost function as follows: $ g(s,a) = 1 $ if $ s \neq s_{\text{goal}} $ , and $ g(s,a) = 0 $ otherwise. This is the simplest cost function we can define for the minimum time problem.

DP is a recursive algorithm that solves "backwards from the goal". We define the **cost-to-go** function $ J^* (s) $ (in RL terminology, the **value function**) as the minimum cost-to-go from state $ s $ to the goal. The cost-to-go function is defined as follows:

$$ J^* (s_i) = \min_{\vec a [\cdot]} \sum_{n=0}^\infty g(s[n], a[n]) \;\; ; \; s[0] = s_i $$

However, an easier to compute definition uses its recursive nature:

$$ J^* (s_i) = \min_{a_i} \left[ g(s_i, a_i) + J^* (f(s_i, a_i)) \right] $$

This definition is known as the **Bellman Equation**, and we can use this recursive definition as an algorithm to compute the cost-to-go function. We start from the goal state and work our way back to the initial state. This is the essence of dynamic programming.

**The Dynamic Programming/Value Iteration Algorithm**:
 - Let $ \hat{J}^* (s) $ be our estimate of the optimal cost-to-go function $ J^* (s) $.
 - Initialize $ \hat{J}^* = 0 $.
 - Repeat until convergence:
   - For all $ s_i \neq s_{\text{goal}} $, update $ \hat{J}^* (s_i) = \min_{a_i} \left[ g(s_i, a_i) + \hat{J}^* (f(s_i, a_i)) \right] $.

This algorithm is guaranteed to converge to the optimal cost-to-go function $ J^* (s) $.

**Caveats**:
 - Accuracy: due to discretization errors (and they build up). They are especially bad in systems that have discontinuities.
 - Scalability: this only works if we can make a fine enough grid to approximate the continuous state space (dimensions ~ 5). This is known as the Bellman Curse of Dimensionality.
 - Assumes model is known. 
 - Assumes cost function is known.
 - Assumes "full state" feedback, i.e. we know the full state of the system.

