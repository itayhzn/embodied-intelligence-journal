---
layout: post
title:  Underactuated Robotics - Lecture 1
date:   2025-01-09 13:51:00 +0200
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


# Introduction

Generally speaking, the current SOTA techniques in robotics are:
 - Most bipeds (e.g. Atlas) use planning with mechanics and optimization.
 - Most quadrupeds (Anymal) use reinforcement learning.
 - In manipulation, the most successful approaches are based on imitation learning.

There are subtle reasons that explain why this is currently the state of the art. One basic answer for why quadrupeds rely on RL whereas bipeds usually don't is that quadrupeds are inherently stable, whereas bipeds fall more often. However, to get a better explanation, we need to think of what makes control difficult. Some of the reasons are:
 - Uncertainty or partial observability,
 - Actions have long-term consequences,
 - Nonlinearity,
 - High-dimensional state and action spaces (for some problems)

There's going to be a lot of nonlinear differential equations (most of which do not have analytic solutions, so there's a lot of numerical computation involved). The typical notation we'll use is:
 - **Dynamics model**: $ \dot{x} = f(x, u) $ where $ x $ is the state and $ u $ is the control input (and $ \dot{x} $ is the derivative of $ x $ with respect to time).
 - **Observation model**: $ y = g(x) $ where $ y $ is the observation (sensor reading) and $ g $ is the observation function.

Since we care a lot about mechanics, and mechanical systems are typically of the 2nd order (e.g. $ F = ma = m\ddot{x} $), then our functions will generally look like $ \ddot{q} = f(q, \dot{q}, u) $, where $q$ is the generalized positions (e.g. joint angles) and $\dot{q}$ is the generalized velocities. In vector notations:

$$ x = \begin{bmatrix} x \\ \dot{x} \end{bmatrix} \; \Rightarrow \; \dot{x} = \bar{f}(x,u) = \begin{bmatrix} \dot{q} \\ f(q,\dot{q}, u) \end{bmatrix}  $$

As it turns out, for basically every mechanics model, if $ u $ is a torque, then $u$ only appears in an affine way: $ \ddot{q} = f_1 (q, \dot{q}) + f_2 (q, \dot{q}) u $. This is called **"control affine" nonlinear systems**.  

**Definition**: Let $ \ddot{q} = f_1 (q, \dot{q}) + f_2 (q, \dot{q}) u $ be a control affine system, and let $[q,\dot{q}]^T $ be some state. Let $ \dim q = m $ and $ \dim u = n $, hence $f_1 \in \mathbb{R}^m $ and $ f_2 \in \mathbb{R}^{m\times n} $.
 1. The system is **fully actuated** in state $[q, \dot{q}]^T $ iff $ f_2 (q, \dot{q}) $ is full row rank, i.e. $ \text{rank} [f_2(q,\dot{q})] = m  $.
 2. The system is **underactuated** in state $ [q, \dot{q}]^T $ iff $ \text{rank} [f_2(q,\dot{q})] < m  $. 
 
For many systems, these hold for all $ q, \dot{q} $, in which case we say the *system* is fully actuated or underactuated.

## Feedback Equivalence

Given a system $ \ddot{q} = f_1 (q, \dot{q}) + f_2 (q, \dot{q}) u $, and given a desired acceleration $ \ddot{q}^d $, if the system is fully-actuated, then there $ f_2^{-1} $ exists, and hence

$$ u = f_2^{-1} (q,\dot{q}) \left[ \ddot{q}^d-f_1 (q,\dot{q}) \right], $$

that is, if the system is fully actuated, then given a desired acceleration, I can give you a $u$ that will achieve that acceleration. This means that we are now **feedback equivalent** to $ \ddot{q}=u $ (double integrator, which we have optimal solutions for).

What does that mean? That means that if you give me any robot, and put enough motors on it, and I can make it behave however I want it to behave. 

**Breaking Feedback Equivalence**:
 
 - with input saturations, e.g. $ u \in [-1,1] $,
 - with state constraints,
 - with model uncertainty.

## Manipulator Equation
The manipulator equation (for rigid body mechanics) is a special case of a control affine system. It is the equation of motion for a robot arm. It is given by:

$$ M(q) \ddot{q} + C(q, \dot{q}) \dot{q} = \tau_g (q) + Bu $$

where $ M(q) $ is the mass matrix, $ C(q, \dot{q}) $ is the Coriolis matrix, and $ \tau_g (q)$ is the torque due to gravity ("gravity terms"), $ B $ is the actuator selector matrix, and $u$ is the torque input.

$ M(q) > 0 $ is a positive definite matrix, and the general form for kinetic energy is $ T = \frac{1}{2} \dot{q}^T M(q) \dot{q} $. Therefore, $ \ddot{q} = M^{-1} (q) \cdot \left[ \tau_g (q) + Bu - C(q, \dot{q}) \dot{q} \right] $.

