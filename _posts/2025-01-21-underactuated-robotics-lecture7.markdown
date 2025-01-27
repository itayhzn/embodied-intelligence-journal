---
layout: post
title:  Underactuated Robotics - Lecture 7
date:   2025-01-21 17:59:00 +0200
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

In the previous lectures, we discussed DP and value iteration, and how to solve control problems using optimization. 

A quick recap of the topics we discussed:
 - DP in the tabular case, where everything is discrete and the problem is basically a graph search. We said that the na\'ive approximation for discretization doesn't scale very well.
 - Linear dynamics and the convex cost case (LQR). We saw how linearinearizing the dynamics can work well around a fixed point, but not necessarily well for the whole state space.
 - Approximate DP, with neurals nets in particular. The hope was that it could do some of the things discretization could do (i.e. the tabular case) but not suffer from the curse of dimensionality as much.
 - All three approaches tried to compute $ J^* (x) $, the cost-to-go function.

Today we will discuss Lyapunov functions, which are a different way to think about stability and control. The goal is to make the problem easier, in the sense that we need to compute a controller that's **not necessarily optimal** cost-to-go function, **but good enough**.
In particular, we will show this through an example on the stability analysis of the simple pendulum/

# Example: The Simple pendulum

Recall, the simple pendulum is described by the following equations:

$$
  m \ell^2 \ddot{\theta} + mg\ell \sin \theta = - b \dot{\theta} ,\quad \text{where} \; b = 0
$$

First, observe that the damped pendulum has a stable fixed point at the bottom.

**Proof**:
- At the top, the energy is a combination of potential and kinetic energy: $ E = \frac{1}{2} m \ell^2 \dot \theta^2 - mg\ell\cos\theta $.
- Consider how the energy evolves in time as a function of the state $x$, i.e. 

$$ 
\frac{dE(x)}{dt} = \frac{\partial E}{\partial x} \dot x = \frac{\partial E}{\partial \theta} \dot \theta + \frac{\partial E}{\partial \dot \theta} \ddot \theta = \dot\theta m g \ell \sin \theta + m \ell^2 \ddot \theta \dot \theta = \dot \theta \left( m g \ell \sin \theta + m \ell^2 \ddot \theta \right) = - b \dot \theta^2 
$$

- Since $ b > 0 $ and $ \dot \theta^2 \geq 0 $, then $ \frac{dE(x)}{dt} = -b\dot\theta^2 \leq 0$, i.e. the energy is always decreasing whenever $\dot\theta \neq 0$. Also, $E$ is bounded below by $-mg\ell$.
- So what's left to prove is that the system doesn't get "stuck" somewhere along its way to the bottom, and it turns out that this is a little bit subtle, and this is what Lyapunov theory is all about.

# Lyapunov Theory

## Generalized Energy Functions

Recall the definition of stable in the sense of Lyapunov (i.s.L.):
$ x^* $ is stable i.s.L. if 

$$ \forall \epsilon > 0 ,\; \exists \delta > 0 \; \text{s.t.}  \left[ 
  (\Vert x(t=0)-x^* \Vert < \delta)
  \;\; \Rightarrow \;\; 
  (\forall t,\; \Vert x(t)-x^* \Vert < \epsilon) \right]$$

Given some system $\dot x = f(x)$, we want to prove stability at $ x^* = 0 $. Note that we don't have a control input. To do this, we construct a differentiable function $ V(x) $ such that:
 - $V \succ 0$ is a **positive definite function**: 
   - $ V(0) = 0 $,
   - $ V(x) > 0 $ for $ x \neq 0 $.
 - $ \dot V \preceq 0 $ is a **negative semi-definite function**:
   - $ \dot V(0) = 0 $,
   - $ \dot V(x) \leq 0 $ for $ x \neq 0 $.

If we can find such a function, then we can conclude that the system is stable i.s.L. (in the sense of Lyapunov) at $ x^* = 0 $, and $ V(x) $ is called a **Lyapunov function**.

