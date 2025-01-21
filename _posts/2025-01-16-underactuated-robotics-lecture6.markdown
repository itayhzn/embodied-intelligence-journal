---
layout: post
title:  Underactuated Robotics - Lecture 6
date:   2025-01-16 19:30:00 +0200
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

In the previous lecture, we talked about value iteration for the acrobat and the cart-pole system. The key ideas we discussed in previous lectures were:

1. local linearization of the dynamics
2. Linear optimal control is pretty powerful if we can linearize the dynamics
3. We basically used the same cost function for all the systems we've seen so far ($Q,R$ were all of the same form). There was no cost function optimization.

We also saw that we can use value iteration on the mesh for the swing up (e.g. a hybrid controller).

In this lecture we'll dig into the nuances of value iteration and to inspect more carefully what happens in the more complicated systems (e.g., if we use a NN to approximate the value function).

# Neural-Fitted Value Iteration, or: Using NN to Approximate the Cost-to-Go Function

Today, we'll use a neuarl net to approximate $ \hat J^* (x) $! 

Recall that our value iteration update was:

$$ \forall s\in S, \quad \hat J(s) = \min_{a\in A} \left[ \ell(s,a) + \hat J(f(s,a)) \right] $$ 

What is the extension of that to NNs? what would the value iteration update look like?

The analogy is that we're basically going to take a bunch of sample points (because we can't easily evaluate for all $x$, but we can samply densely). [^1]

A training iteration:

- Step 1: Compute $ J^d$:
  - Draw samples $ X_S \subset X $
  - Draw samples $ U_S \subset U $
  - $ \forall x_i\in X_s,\;\forall u_j \in U_S $, let $ x_{ij}' = f(x_i, u_j), \; \ell_{ij} = \ell (x_i, u_j), $ and define the "desired loss" as:

$$ J_i^d = \min_{j} \left[ \ell_{ij} + \hat J(x_{ij}') \right] $$ 

- Step 2: Optimize the NN for $ \hat J(x) $
  - Using the samples $ (x_i, J_i^d) $, train by minimizing loss $ \sum_i \left( J_i^d - \hat J(x_i) \right)^2 $.

### Target network
Sometimes these two steps are written a bit differently. In the other formulation, we keep two NNs for $J$, called $J_\alpha$ and $J_\beta$, where $J_\beta$ is changed less frequently than $J_\alpha$. $J_\beta$ becomes something like a low-pass filter for $J_\alpha$, and is sometimes called the **target network**. Then, the optimization can be written in a single line as follows:

$$ \text{minimize  } \sum_i \left[ J_\alpha (x_i) - \min_j \left[ \ell_{ij} + J_\beta (x_{ij}') \right] \right]^2 $$

### Special case: Linear NN

Denote the linear NN as $ \hat J_\alpha (x) = \alpha^T \Phi(x) $, where $\Phi(x)$ is a nonlinear basis for the input features. Suppose $ \Phi(x) $'s weights are frozen. In this setting, $ \min_\alpha \sum_i \left[ \alpha^T \Phi(x_i) - J_i^d \right]^2 $ is linear least squares, has a closed form solution, and SGD is guaranteed to converge to the optimal solution.

**Tsitsiklis and van Ray (1997)** proved that value iteration (and even temporal difference learning $ TD(\lambda) $) with linear neural networks converges.

### How does it work for discrete-time LQR?

The NN computes $ J^* (x) = x^T S x $, where $ S $ are the parameters of the network (and they enter the equation linearly, as desired). Russ applies the algorithm above on the double integrator, and showed that it doesn't work. 

What made it unstable? LQR is trying to solve an *infinite horizon* problem, and we have to be careful about the cost not running to infinity.
How can we do that?

1. Discounted costs: instead of $ \sum_{n=0}^\infty \gamma^n \ell(x[n], u[n]) $ for $ \gamma \in (0,1) $. This is a common trick in reinforcement learning, and this can fix it. As it turns out,

$$ K,S = \text{DiscountedLQR} (A,B,Q,R) = \text{LQR}(\sqrt \gamma A, B, Q, \frac{1}{\gamma} R) $$

2. Use optimal $u_j$: recall in CTLQR (continuous time LQR), we have $ u^* = -Kx = -R^{-1} B^T S^* x $. In the DT case, we get $ u^* = -Kx = - (R+B^T S^* B)^{-1} B^T S^* A x $. This is longer to write, but still solvable with the Riccati equation. Instead of sampling $u$, let's just sample $X_S \subseteq X$, and compute $ \hat u_i^ * = - \hat K x_i $ for all $ x_i \in X_S $. We can then compute the desired loss as:

$$ J^d_i = x_i ^T Q x_i + \hat u_i ^ T R \hat u_i  + \hat J_S (Ax_i + B \hat u_i) $$ 

In the more general case, given $ \hat J_\alpha (x) , \; \ell(x,u), \; f(x,u)$, when can I solve $ \hat u_i $ in closed form? That is, when can I write $ \hat u_i = \text{argmin}_u \left[ \ell(x_i,u) + \hat J_\alpha (f(x_i,u)) \right] $ in closed form? 

There are two elements in there, $\ell$ and $\hat J_\alpha$. 

- First, it is wise to make life easier for ourselves by choosing $\ell(x,u)$ to be quadratic in $u$, i.e., $ \ell(x,u) = u^T R u $. We rarely need to do something more complicated than that. 

- Now for the $\hat J_\alpha$ part. In the continuous case, it is even easier, because we need to solve $ 0 = \min_u \left[ \ell(x, u) + \frac{\partial \hat J}{\partial x} f(x,u) \right] $. In the infinitecimal case, we don't need to go through the NN, we just need its gradient. Here too, if we restrict $f$ to be control affine (i.e., $f(x,u) = f_1 (x) + f_2 (x) u$), this really simplifies the problem. 

If we make these choices for $\ell$ and $f$, we can solve for $\hat u_i$ in closed form:

$$ \hat u_i = - \frac{1}{2} R^{-1} f_2^T (x) \left.\left(\frac{\partial \hat J}{\partial x}\right)^T\right\vert_{x=x_i}  $$



<br><br><br>

---

[^1]: We can make it fully continuous, and in fact, we'll do it later. But for now, let's just think about it as a discrete set of samples.