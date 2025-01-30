---
layout: post
title:  Underactuated Robotics - Lecture 5 - Acrobots and Cart-Pole Systems
date:   2025-01-15 17:35:00 +0200
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

In the previous lecture, we discussed dynamic programming in the discrete (tabular) case and in the continuous case. 
 - In the tabular case, everything is discrete, and the problem is basically a graph search (a generalization of MDPs).
 - We then discussed linear dynamics with convex optimization (where we gave the example of LQR).

This lecture will focus on Acrobots (a 2-link arm), the cart-pole system (a cart with a pole on top), and Quadrotors. 

# Introducing the systems

The Acrobot is a 2-link arm with a motor at the elbow. Note that this is not just a double pendulum, because it assumes the arms are rigid.

<p align="center">
  <img src="https://underactuated.csail.mit.edu/figures/acrobot.svg">
</p>

The cart-pole systems is basically an inverted pendulum. It is a cart
(a double integrator) with a pole on top. The goal is to balance the pole on top of the cart.

<p align="center">
  <img src="https://underactuated.csail.mit.edu/figures/cartpole.svg">
</p>

The goal in both systems is to balance the mass at the top. 

# Equations of Motion

For all of these systems, they look pretty much the same:

$$ M(q) \ddot q + C(q,\dot q) \dot q = \tau_g (q) + Bu $$

| Symbol | Acrobot | Cart-pole |
|---|---|---|
| $ q $ | $ [\theta_1, \theta_2]^T $ | $ [x_{\text{cart}}, \theta_{\text{pole}}]^T $ |
| $ \dot q $ | $ [\dot \theta_1, \dot \theta_2]^T $ | $ [\dot x, \dot \theta]^T $ |
| B | $ [0, 1]^T $ | $ [1, 0]^T $ |
| $ u $ | $ [\tau_{\text{elbow}}] $ | $ [F_{\text{cart}}] $ |
| Constraints | $ \vert u \vert \leq u_{\max} $ | $ \vert u\vert \leq u_{\max} \;;\; \vert x \vert \leq x_{\max} $ |

# Discussion on Possible Techniques to Swing Up and Balance

Consider both the Acrobot and the Cart-pole systems. The goal is to swing up the system and balance it at the top.

#### Tabular Value Iteration
Both problems are 4-dimension problems; each problem's has two positions and two velocities. They are therefore solvable with tabular value iteration (recall that we said it is not scalable to high dimensions, and fails when the dimensionality is over 5/6). However, this is suprisingly **very** hard, because it requires a very fine discretization to get the right answer, and the required high resolution comes in unexpected places. 
In conclusion - possible, but not elegant.

#### Linear Quadratic Regulator (LQR)
This would be a beautiful solution for balancing at the top, but we'd need more for the "swing-up" part.

#### Spoiler Alert
> We will end up with a hybrid system, where we will use LQR for balancing at the top, and a different controller for the swing-up part.

# LQR for Nonlinear Systems
Recall that LQR was for a linear system $ \dot x = Ax + Bu $, yet in the general form, we have a nonlinear system $ \dot x = f(x,u) $.

To solve the general case, we will "linearize" the system by taking the Taylor approximation around a particular nominal fixed point $ (x_0, u_0) $, and then apply LQR to the linearized system: 

$$ \dot x \approx f(x_0, u_0) + \left.\frac{\partial f}{\partial x}\right\vert_{(x_0, u_0)} \cdot (x-x_0) + \left.\frac{\partial f}{\partial u}\right\vert_{(x_0, u_0)} \cdot (u-u_0) $$

If we define $ A = \left.\frac{\partial f}{\partial x}\right\vert_{(x_0, u_0)} $ and $ B = \left.\frac{\partial f}{\partial u}\right\vert_{(x_0, u_0)} $, we get:

$$ \dot x \approx Ax + Bu + \left[ f(x_0, u_0) - A x_0 - B -u_0 \right] $$

This is still not the correct form, because there are some affine terms. To get rid of them, we will change coordintes to $ \bar x = x-x_0 $ and $ \bar u = u-u_0 $. Note that this means that in the new system $ \dot {\bar x} = \dot x - \dot x_0 $, and if $ \dot x_0 \neq 0 $ then it is like the coordinate system is moving on you (not impossible to deal with, but requires tracking some extra terms). Substituting this in, we get:

$$ \dot x \approx \dot x_0 + A \bar x + B \bar u  \implies \dot x - \dot x_0 = A \bar x + B \bar u \implies \dot{\bar x} = A \bar x + B \bar u $$

The stability of a linear system is determined by the eigenvalues of the matrix $ A $. Recall the definition of eigenvectors and eigenvalues: $ A v = \lambda v $, and $ \text{det} (A - \lambda I) = 0 $. Note that the two vectors $ v_1, v_2 $ span the $ \dot \theta $ vs $ \theta $ plane, i.e. any $x$ can be represented as $ x = \alpha _1 v_1 + \alpha 2 v_2 $. And if $ \dot x = A x $, we get that $ \dot x = \alpha _1 A v_1 + \alpha _2 A v_2 = \alpha _1 \lambda _1 v_1 + \alpha _2 \lambda _2 v_2 $, i.e. the dynamics are just a scaling of the eigenvectors. We got that in the eigenvector space, figuring out the derivative is multiplication by the eigenvalues.

> ## Example: Simple Pendulum
>
> $$ m \ell^2 \ddot \theta + b\dot \theta + mg\ell \sin \theta = u $$
>
> $$ \text{where: }\; q = [\theta],\; x = [\theta, \dot \theta]^T,\; u = [\tau] $$
> 
> The dynamics are given by:
> 
> $$ \dot x = \begin{bmatrix} \dot \theta  \\  \frac{1}{m\ell^2} (\tau - b\dot \theta - mg\ell\sin \theta) \end{bmatrix} $$
> 
> We will linearize this system about the upright position $ \theta = \pi,\; \dot \theta = 0,\; u = 0 $, that is: $ x_0 = [\pi, 0]^T,\; u_0 = 0 $. Not that in this point $ \dot\theta $ is indeed 0, i.e. this is a fixed point. The linearization will be:
> 
> $$ A = \left.\frac{\partial f}{\partial x}\right\vert_{(x_0, u_0)} = \left.\begin{bmatrix} 0 & 1 \\ -\frac{g}{\ell} \cos \theta & -\frac{b}{m\ell^2} \end{bmatrix}\right\vert_{(x_0, u_0)} = \begin{bmatrix} 0 & 1 \\ \frac{g}{\ell} & -\frac{b}{m\ell^2} \end{bmatrix} $$
> 
> $$ B = \left.\frac{\partial f}{\partial u}\right\vert_{(x_0, u_0)} = \begin{bmatrix} 0 \\ \frac{1}{m\ell^2} \end{bmatrix} $$
> 
> Let's set $m = 1, \ell = 1, b = 0, g = 10$. We will then get:
> 
> $$ A = \begin{bmatrix} 0 & 1 \\ 10 & 0 \end{bmatrix},\; B = \begin{bmatrix} 0 \\ 1 \end{bmatrix} $$
> 
>
> Computing the eigenvalues, we get the characteristic equation $ \lambda^2 - 10 = 0 $, which gives us $ \lambda = \pm \sqrt{10} $. The corresponding eigenvectors are $ v_1 = [1, \sqrt{10}]^T $ and $ v_2 = [1, -\sqrt{10}]^T $. The system is stable because the real part of the eigenvalues is negative.

Now, given the dynamics equation $ \dot{\bar x} = A \bar x + B \bar u $ and the cost-to-go function $ J = \int_0^\infty \left( x^T Q x + u^T R u \right) dt $, we can solve the LQR problem by solving the Riccati equation and get $ K $ such that the optimal control policy is $ u = -Kx $. Note that if we substitute this in the dynamics equation, we get $ \dot x = (A - BK) x $, which is the closed-loop dynamics. This means that the eigenvectors of $ A - BK $ define the stability of the system.

This also means that for any choice of $Q$ and $R$, we are guaranteed to find a $K$ that stabilizes the system! However, an interesting thing happens when we introduce discounting for future rewards (as is often done in RL). In some sense, this means we're optimistic about the system's ability to stabilize itself, and this makes guarantee (that for every $Q$ and $R$ there is a $K$) not hold anymore.

> For the simple pendulum, a good choice would be
>
> $$ Q = \begin{bmatrix} 10 & 0 \\ 0 & 1 \end{bmatrix} \;\;,\;\; R = [1] $$
>
> To understand why, think about the units of these entries. Since $Q$ is multiplied by $x^T$ and $x$, then the first entry of $Q$ is multiplied by $\theta$, i.e. will have units of $[\text{rad}]^2$, and the second entry is multiplied by $\dot \theta$, i.e. will have units of $[\text{rad/s}]^2$. So, we need the entries to "scale" then so they are roughly the same. And how can we go that? Well, $\text{rad/s}$ is pretty much $ \sqrt{g/\ell} $, so to scale correctly, we need to multiply the first entry by $ g / \ell \approx 10 $. This is a great rule of thumb! If your robot is about a meter long, use this matrix for $Q$.
>
> Similarly, for the double pendulum, a good choice would be 
>
> $$ Q = \begin{bmatrix} 10 & 0 & 0 & 0 \\ 0 & 10 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} \;\;,\;\; R = [1] $$


Interesting fact: LQR controllers are always stabilizing, but not all stabilizing controllers are LQR. This is because LQR controllers are linear, and a stabilizing controller might not be linear.

## Controlability and Stabilizability

As we saw, some systems can be underactuated but still controllable. 

Given $ \dot x = A x + B u $, there are two major questions:
- Is $(A,B)$ ***controllable***? Namely, given $x(t=0)$ can you find $ u(t) \forall t\in [t_0, t_f]$ s.t. $x(t_f) = x_{\text{goal}}$?
- Is $(A,B)$ ***stabilizable***? Namely, given $x(t=0)$ can you find $ u(t) \forall t\in [t_0, \infty)$ s.t. $\lim_{t\to \infty} x(t) = 0$?

The difference between controllable and stabilizable is that in the stabilizable case, you are allowed to take infinite time to get to the goal. 

For linear systems, stabilizable is a weaker condition than controllable. For example, a system that is stable and has no control inputs is stabilizable, but not controllable.

