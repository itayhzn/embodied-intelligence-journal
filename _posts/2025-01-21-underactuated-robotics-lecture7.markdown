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

If we can find such a function, then we can conclude that the system is stable i.s.L. at $ x^* = 0 $, and $ V(x) $ is called a **Lyapunov function**.

**Variations**:
- For asymptotic stability, we need to show that $ V(x) $ is a **strictly decreasing function**, i.e. $ V \succ 0 $ and $ \dot V \prec 0 $
- For global asymptotic stability (GAS), also requires that $ V(x) $ is a **radially unbounded function**, i.e. $ V(x) \to \infty $ as $ \Vert x \Vert \to \infty $.
- For regional stability, we require that $V$ satisties the requirements above but only when limited to some region $ \mathcal{D} \subseteq \mathbb R^n $ that contains $x^*$ (namely, $ x^* \in \mathcal D $).
- For exponential stability, we require that $V \succ 0$ and that $ \dot V(x) \preceq -\alpha V(x) $ for some $ \alpha > 0 $.

### Some examples

#### The simple pendulum
In the example of the simple pendulum, we can define $ V(x) = E(x) + mg\ell $, which is a Lyapunov function, and hence the simple pendulum is stable i.s.L. at $ x^* = 0 $.

#### A simple linear system
Consider the system $ \dot x = -x $. We can define $ V(x) = x^2 $, which is a Lyapunov function, and hence the system is stable i.s.L. at $ x^* = 0 $. Observe that $ \dot V (x) = \frac{\partial V}{\partial x} \cdot \dot x = 2x \cdot -x = -2x^2 \leq 0 $. Therefore:
 - The system is stable i.s.L. since $ V \succ 0 $ and $ \dot V \preceq 0 $.
 - The system is asymptotically stable since $ V \succ 0 $ and $ \dot V \prec 0 $.
 - The system is exponentially stable since $ V \succ 0 $ and $ \dot V \preceq -2V $.
 - The system is globally asymptotically stable since $ V \to \infty $ as $ \Vert x \Vert \to \infty $. [^1]

#### A simple non-linear system
Consider the system $ \dot x = f(x) = -x + x^3 $. If we plot $ f(x) $ vs $x$, we find that $ x^* = 0$ is a fixed point with region of attraction $ x \in (-1, 1) $.[^2] To prove that, we can define $ V(x) = x^2 $. If we do the math, we find that $ \dot V(x) = -2x^2 + 2x^4 = 2x^2(x^2-1) $. Observe that:

$$ \dot V (x) = \begin{cases} 0 & , x=0 \\ >0 & , \vert x \vert > 1 \\ < 0 & , \vert x \vert < 1\end{cases} $$

## General Form of ROAs (Regions of Attraction)

If $ V(x) \succ 0$ and $ \dot V(x) \prec 0 $ for every $ x \in \left\{x \mid  V(x) \leq \rho \right\} $ (for some $ \rho > 0 $), 
then 

$$ V(x(0)) \leq \rho \;\; \Rightarrow \;\;\left[ \lim_{t\to\infty} V(x(t)) = 0 \;\wedge \; \lim_{t\to\infty} x(t) = 0 \right]$$

Furthermore, the **invariant set** $ \left\{x \mid  V(x) \leq \rho \right\} $ must be inside the region of attraction (ROA) of the fixed point $ x^* = 0 $.

> La Salles's Theorem gives a slightly stronger condition.

# Lyapunov Functions and Control

How does Lyapunov relate to dynamic programming? 

Recall that in (continuous time) DP, the Hamilton-Jacobi-Bellman (HJB) equation we're trying to solve is:

$$ \min_u \left[ \ell (x,u) + \frac{\partial J^* }{\partial x} f(x,u) \right] = 0 $$

Recall that Lyapunov functions don't take control inputs into account. So, how do we relate the two? If the optimal policy $ \pi^* $ is a function of $ x $, then for $ \pi^* $:

$$ 
 0 = \ell (x,\pi^*(x)) + \frac{\partial J^* }{\partial x} f(x,\pi^*(x))
$$

And since $ \frac{\partial J^* }{\partial x} f(x,\pi^*(x)) = \dot J^* (x) $, then we can write the above equation as: 

$$
\dot J^* (x) = - \ell (x,\pi^*(x))
$$

Let's compare that with the Lyapunov condition $ \dot V(x) \preceq 0 $. If $\ell > 0$ everywhere, then choosing $ V(x) = J^* (x) $ is a Lyapunov function. But the more interesting perspective is that a Lyapunov function is a relaxation of the HJB cost-to-go function.

 <br><br><br>

 -----
 [^1] If a linear system is stable, then it is always globally asymptotically stable. This is a special property of linear systems.

 [^2] Regions of attraction are always open sets.