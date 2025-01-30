---
layout: post
title:  Underactuated Robotics - Lecture 9 - Lyapunov Analysis (3)
date:   2025-01-29 17:41:00 +0200
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

In the previous lectures, we were working with the standard Lyapunov conditions: $ V(x) \succ 0 $, $ \dot V(x) \prec 0 $. One of the first things we did was talk about the case where we have a linear neural network, and said that if we sample a set $ \\{ x_i \\} $ and require that the constraints are satisfied $\forall x_i$, them we have a linear program. The question now is how to from from samples to the entire space, i.e. $ \forall x_i \; \Rightarrow \; \forall x \in \mathbb R^n $.

We discussed the simple example of a linear system $ \dot x = A x $ with constraints $ V(x) = x^T P x \succ 0 $ and $ \dot V(x) = x^T PA x + x^T A^T P x \prec 0 $. We said that $ \left[ V(x) \succ 0 \; \iff \; P \succ 0 \right] $ and that $ \left[ \dot V(x) \prec 0 \; \iff \; (PA+A^T P) \prec 0 \right]$.

We formulated the problem as a semidefinite program (SDP) where we want to find matrices $ P, P_2 $ such that $ P, P_2 \succ 0 $ and $ PA + A^T P = - P_2 $. 

## Sum of Squares (SOS) Decomposition

Given a function $ f(x) $, how do you show that $ f(x) > 0 $ for all $x$?

If you can write $f$ as $ f(x) = \Phi(x)^T P \Phi(x) $ where $ P \succ 0 $ and $\Phi$ is a basis to the function space, then $ \forall x \; f(x) > 0 $. 
If you can write $f$ in such a way, then this is called a *sum-of-squares* (SOS) decomposition.

This also works with a relaxed condition: $ f(x) \geq 0 $, and in that case $ P \succeq 0 $. Note that the existence of an SOS decomposition is sufficient but not necessary to prove that $ \forall x, \; f(x) \geq 0 $.

An interesting special case is if $f$ is a polynomial. In this case, we can take the monomial basis $ \Phi(x) = [1, x, y, x^2, xy, y^2, \ldots] $ up to rank $\sqrt{\deg f}$. And then we decompose $ f(x) = \Phi(x)^T P \Phi(x) $.

> Example: 
>
> $$ 2 - 4x + 5x^2 = \begin{bmatrix} 1 & x \end{bmatrix} \begin{bmatrix} P_{11} & P_{12} \\ P_{21} & P_{22} \end{bmatrix} \begin{bmatrix} 1 \\ x \end{bmatrix} = \begin{bmatrix} 1 & x \end{bmatrix} \begin{bmatrix} 2 & -2 \\ -2 & 5 \end{bmatrix} \begin{bmatrix} 1 \\ x \end{bmatrix} $$
>
> Recall, by the way, that for a matrix to be positive definite, the standard definition requires that it is symmetric.

A nice observation is that since robot dynamics are rigid and preserve Euclidean distances, they are in fact polynomial in $ \cos \theta, \sin \theta $, etc. Therefore, we can use the SOS decomposition to show that the Lyapunov conditions are satisfied.

Recall that we defined SDP as: linear objective + linear constraints + positive semidefinite constraints. 
There is a generalization of SDP called *sum-of-squares programming* (SOS) where: linear objective + linear constraints + PSD constraints + SOS constraints (e.g. $ P_\alpha (x)$ is SOS).


## Using convex optimization to solve non-convex problems

Convex optimization doesn't solve everything, because some problems aren't convex. However, the real convex optimization experts know how to work around this limitation.

A nice example is the six-hump camel function: $ p(x,y) = 4x^2 - 2.1x^4 + \frac{x^6}{3} + xy - 4y^2 + 4y^4 $. This function is clearly non-convex, and has six local minima and one global minimum. 

<p align="center">
  <img src="https://gael-varoquaux.info/scipy-lecture-notes/_images/sphx_glr_plot_2d_minimization_002.png" />
</p>

To solve this problem, we solve the following optimization problem:

$$ \min_\gamma -\gamma \quad \text{subject to} \quad p(x,y)-\gamma \text{ is SOS} $$

That is, we find the smallest $\gamma$ such that if we shift the function by $\gamma$, the function will be positive everywhere (i.e. SOS).

> Note that the alternative optimization problem we defined finds the minimum value $\gamma$ of $p$, but not the minimizer $(x^*, y^*)$. Finding the minimizer requires a few additional tricks, namely going through the dual problem.

The general idea here was to take a class of functions that is guaranteed to be positive, and try to make our function as close to that class as possible. We can apply the same idea to neural networks. 

#### Doing the same, but with neural networks

Suppose again that you want to prove that $ \forall x \; f(x) \geq 0$. We can try to approximate $f$ using a NN in the following way:

$$
  \min_{\text{NN}} \left\Vert f(x) - \text{NN}^T(x) \cdot \text{NN}(x) \right\Vert,
$$

where $\text{NN}(x)$ is a neural network. Note that we multiply the neural network by its transpose to guarantee it is positive.

## Back to Lyapunov Functions

In Lyapunov analysis, we get to choose $V(x)$ to be whatever we want, so let's choose it to be polynomial. Also, assume that our dynamics $ \dot x = f(x) $ are restricted to be polynomial. Therefore, $ \dot V(x) = \frac{\partial V}{\partial x} \cdot f(x) $ is also polynomial.

The Lyapunov conditions are them translated to SOS constraints: $V$ is SOS, and $- \dot V$ is SOS.

### Example

Suppose we have the following two dimensional system: 

$$ 
  \dot x_1 = -x_1 +2x_2^2 \quad \text{and} \quad \dot x_2 = -x_2 - x_1 x_2 -2x_2^3.
$$

It is known that the following SOS function is a Lyapunov function for this system: $ V(x) = x_1^2 +2x_2^2 $. The only question now is whether $ -\dot V$ is also SOS. If it is, then we have a certificate that the system is stable.

### Regions of Attraction: The $S$-Procedure

In the examples above, we used SOS optimization as some kind of oracle that replies either yes or no to the following type of question: "Is $p(x) \geq 0$ for all $x \in \mathbb R ^n$?". To test whether some $\mathcal D \subset \mathbb R^n$ is a region of attraction, we need to be able to ask our oracle: "Is $p(x) \geq 0$ for all $x \in \mathcal D$?".

To do so, we need to come up with a modification to $p$ that is equal to $p$ on $\mathcal D$ and is trivially positive outside $\mathcal D$. Then, we can ask our oracle whether this modified function is SOS, and if it is, then $\mathcal D$ is a region of attraction. This is a bit tricky, though, because we want the modified function to be differentiable.

First, assume $\mathcal D$ using a polynomial constraint $g(x)$, i.e., consider $\mathcal D \triangleq \\{x \mid g(x) \leq 0\\} $. Now, we ask the oracle the following question:

$$
  \text{find } \alpha \text{ such that } p(x) + \lambda_\alpha^T (x) g(x) \text{ is SOS, and } \lambda_\alpha (x) \text{ is SOS}.
$$

$\lambda_\alpha$ is similar to a Lagrange multiplier, but with a whole polynomial of finite degree as our multiplier: $ \lambda_\alpha(x) = \sum_i^N \alpha_i x^i $.

Why does that make sense? Since we constrained $\lambda_\alpha (x)$ to be SOS, then it is always non-negative. Now consider the following cases:

- $ x \in \mathcal D$: this is the region where $g(x)\leq 0$, thus $ \lambda_\alpha (x) g(x) \leq 0 $. This means that $p(x) + \lambda_\alpha (x) g(x) \leq p(x)$. And if we are able to prove that the lefthand side is SOS, then we have a certificate that $p(x) \geq 0$ in $\mathcal D$. 

- $ x \notin \mathcal D$: In this case, $g(x) > 0$, and therefore $ \lambda_\alpha (x) g(x) > 0$. This means that $p(x) + \lambda_\alpha (x) g(x) > p(x)$. That is, we have added some positive terms to $p$ outside $\mathcal D$ to ensure it is also positive there (so that the oracle can answer $\forall x\in \mathbb R^n$).

> If $\mathcal D = \\{ x \mid g(x) = 0 \\}$, then we have even more tools, because the problem has gone from semi-algebraic geometry to algebraic geometry. In our case, we can drop the second constraint and just solve:
>
> $$
  \text{find } \alpha \text{ such that } p(x) + \lambda_\alpha^T (x) g(x) \text{ is SOS, and } \lambda_\alpha(x) \text{ can be arbitrary}.
$$

