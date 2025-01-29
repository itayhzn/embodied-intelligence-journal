---
layout: post
title:  Underactuated Robotics - Lecture 9
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