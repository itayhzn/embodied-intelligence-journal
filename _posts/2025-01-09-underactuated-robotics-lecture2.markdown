---
layout: post
title:  Underactuated Robotics - Lecture 2
date:   2025-01-09 15:59:00 +0200
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

Following up on the previous lecture, we continue to explore the concept of underactuated systems and how they can be controlled. The key takeaway from the previous lecture is that we need to embrace dynamics and mechanics to understand the control of underactuated systems. And apparently, to understand the core concepts, we do not require too complex systems, and for that reason, we'll start with a simple pendulum.

# "Simple" Pendulum

<p align="center">
<img src="https://underactuated.csail.mit.edu/figures/simple_pend.svg">
</p>

Developing the dynamics is relatively easy:
 - Kinetic Energy: $ T = \frac{1}{2} m \ell^2 \theta^2 $
 - Potential Energy: $ V = -m g \ell \cos\theta $ , where the reference point is the base of the pendulum.
 - The lagrangian: $ m\ell^2 \ddot{\theta} + mg\ell \sin \theta = Q $ , where $Q$ is the generalized force.  
 - The generalized force $ Q $ will be the torque applied around the joint (pivot point). We will model two types of forces there: a torque due to damping (assume linear damping due to friction) and a motor torque. Therefore, $ Q = -b \dot{\theta} + u $.

Thus, the dynamic equation becomes:
$$ m\ell^2 \ddot{\theta} + mg\ell \sin \theta + b \dot{\theta} = u. $$
Note that this is exactly of the form we discussed in the previous lecture: $ M(q) \ddot{q} + C(q, \dot{q}) \dot{q} = \tau_g (q) + Bu $.

We might think that given $ \theta(0), \dot{\theta}(0) $ we can compute $\theta(t)$, but solving this differential equation cannot be done analytically. Instead of asking what happens in time $t$, let's ask other questions:
 - what happens to $ \theta(t) $ in the limit as $t \to \infty$?
 - Will my robot fall down?
 
This lecture is dedicated to graphical analysis. Since we have a differential equation in one variable, and the differential equation is of the second order, we will need two variables to talk about it. So we'll have to simplify it a bit.

## Scenario 1: Heavily Damped Pendulum (First Order System)

Imagine we put the pendulum in molasses. The damping term $b$ is very large. In this case, the damping term will dominate the dynamics, and it loses its oscilatory 2nd order dynamics. 
We'd want to write something like $ b \dot{\theta} \gg m\ell^2 \ddot{\theta}$. However, these quantities have different units: the righthand side is $ \text{kg} \cdot \frac{m^2}{s^2} $, whereas the lefthand side is $ \text{kg} \cdot \frac{m^2}{s} $. So we're missing a factor of $ \frac{1}{s} $ in the damping term. We thus need a characteristic time constant in order to compare these terms. We will use the natural frequency of the pendulum is $ \sqrt{\frac{g}{\ell}} $, which has units $ \frac{1}{s} $. We will therefore define the heavily damped regime as: $ b \sqrt{\frac{\ell}{g}} \gg m\ell^2 $.

In this case, our dynamics equation becomes: $$ b \dot\theta = u-mg\ell\sin\theta $$
Just to draw the graphical analysis, let ignore the fact that $\theta$ wraps around, and just treat it like a real number $ x\in\mathbb R $. We can therefore plot the dynamics as: $ b \dot x = u - mg\ell \sin x $, and simply plot $ \dot x $ vs $ x $.

<p align="center">
<img src="https://underactuated.csail.mit.edu/figures/pend_sinx_annotated.svg">
</p>

Every time this curve crosses the $x$-axis, the pendulum will stop (since $ \dot x $, i.e. velocity, is 0). Consider two adjacent fixed points. If the curve between these two points is above the $x$-axis, then the pendulum will move towards the right (since velocity would have a positive sign). If the curve is below the $x$-axis, then the pendulum will move towards the left. Therefore, the filled circles are stable fixed points, whereas the open circles are unstable fixed points.

**Definitions of (local) stability**:

| Property | Intuition | Definition |
| --- | --- | --- |
| in the sense of Lyapunov (i.s.L.) | if I start near the region, I won't go too far away from the region | $ \forall \epsilon \exists \delta $ s.t. $ \Vert x(0) - x^* \Vert \leq \delta \Rightarrow \forall t \; \Vert x(t) - x^* \Vert \leq \epsilon $ |
| locally attractive | if I start near the region, I'll converge to the region | $ \lim_{t\to\infty} x(t) = x^* $ |
| asymptotically stable | both attractive and i.s.L. |  |
| exponentially stable | attractive and the rate of convergence is exponential | $ \Vert x(t) - x^* \Vert < Ce^{-\alpha t} $ |

### Simple Recurrent Neural Network

Suppose we have a simple neuron that computes $ f(x) = \tanh (w^T x) $. Now, suppose we take its output and plug it into the input, i.e. $ x = f(x) $. This is a simple recurrent neural network (RNN). Now, consider the dynamics of how the activation changes in the neuron. The dynamics of a simple RNN are given by: $ \dot x = -x + \tanh x $. This is known as an Autapse (fun fact: it happens frequently when you grow neurons in a dish. They try to find things to connect to, and they often connect back to themselves). 

Plotting the dynamics of the Autapse, we get the following graph:

<p align="center">
<img src="https://underactuated.csail.mit.edu/figures/pend_autapse.svg">
</p>

There are 3 fixed points, but only the outer two are stable. 

Russ mentioned Hopfield Networks, which are a type of RNN used for associative memory. These networks have several stable fixed points, and the dynamics of the network converge to one of these fixed points. He showed a nice example of how the network can be used to store and recall memories. 

## Scenario 2: Second Order System
The second order system is a bit harder to draw, but it is still possible.

Let's go back to our original dynamics equation: $ m\ell^2 \ddot{\theta} + mg\ell \sin \theta + b \dot{\theta} = u $. 
To write the dynamics in a simplified form, we denote $ x = [ \theta , \dot{\theta} ]^T $. We therefore get a vector differential equation of the form $ \dot x = f(x, u) $ as the following:
$$ \begin{bmatrix} \dot{\theta} \\ \ddot{\theta} \end{bmatrix} = \begin{bmatrix} \dot{\theta} \\ \frac{1}{m\ell^2} (u - mg\ell \sin \theta - b \dot{\theta}) \end{bmatrix}  $$

We can now plot the vector field of this system: the axes are $ \theta $ and $ \dot{\theta} $, and at every point in this space, we plot the vector $ f(x, u) = [ \dot\theta, \ddot \theta]^T $.

First, consider the case where $ u = 0 $ and $ b = 0 $. In this case, the dynamics are given by $ \begin{bmatrix} \dot{\theta} \\ \ddot{\theta} \end{bmatrix} = \begin{bmatrix} \dot{\theta} \\ -\frac{g}{\ell} \sin \theta \end{bmatrix} $. When we plot this vector field, we get the following:

<p align="center">
<img src="https://underactuated.csail.mit.edu/figures/pend_undamped_phase.svg">
</p>

When we add back damping ($b > 0$), we get another vertical component to the vector field, pulling it towards the $x$-axis, and so the spiral collapses inwards. The vector field will now look like this:

<p align="center">
<img src="https://underactuated.csail.mit.edu/figures/pend_damped_phase.svg">
</p>

# Conclusion
The question we ask in this course is what happens when $ u $ stops being 0, i.e. when we start applying a control input. How does that change the dynamics of the system? Or better yet, how should we change $u$ to make the pendulum go to a specific point?