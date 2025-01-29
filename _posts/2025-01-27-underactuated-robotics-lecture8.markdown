---
layout: post
title:  Underactuated Robotics - Lecture 8
date:   2025-01-27 20:31:00 +0200
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

In the previous lectures, we talked about Lyapunov functions and made a connection to the HJB equation:

$$ 0 = \min_u \left[ \ell(x,u) + \frac{\partial J}{\partial x} f(x,u) \right] $$

$$ \Longrightarrow \quad \dot J (x) = -\ell(x,u^* ), \quad \dot V(x) \leq 0 $$

In discrete time, we can define:

$$ J^* (x) = \min_u \left[ \ell(x,u) + J^* (f(x,u)) \right] $$

$$ \Longrightarrow \quad \Delta J^* (x) = J^* (f(x,u)) - J^* (x) = - \ell(x,u), \quad \Delta V (x) \leq 0 $$

The reason that connection is so powerful was that although finding an optimal cost-to-go function is hard, finding Lyapunov functions is often easier. Why is that? Because there are many more Lyapunov functions (since the requirements from Lyapunov functions are a relaxed version of the requirements from cost-to-go functions).

Today we will do a global Lyapunov analysis for the pendulum. 

- Input: 
  - Pendulum dynamics
  - Parametric family of $ \text{trig}/\text{poly} $ functions for Lyapunov (i.e. polynomial in $ \cos \theta, \sin \theta, \dot \theta $, etc.)[^1]
- Output: 
  - Coefficients of the polynomial
  - Certificate that Lyapunov conditions satisfied $ \forall x $.

# Optimization Crash Course

Suppose we want find a solution to the optimization problem:

$$ \min_x f(x) \quad \text{subject to} \quad g(x) \leq 0 $$

We denote $x$ as the *vector of decision variables*, $f(x)$ as the *scalar objective*, and $g(x)$ as the *vector of constraints*. A solution to this problem is the point $x^* $ that minimizes $f(x)$ while satisfying the constraints ($ x^* $ is also called the *minimizer*).

An optimization problem is called *convex* if $f(x)$ is convex and the constraints $ \\{ g_i (x) \\} $ form a convex set. 

> Recall: a function $h(x)$ is convex if $h(\lambda x + (1-\lambda) y) \leq \lambda h(x) + (1-\lambda) h(y)$ for all $ x,y \in \text{dom}(h)$ and for all $ \lambda \in [0,1]$.

Convex optimization problems are nice because they have a unique minimizer, and we can find it efficiently.

Examples:
- Linear programming (LP): $ \min c^T x $ subject to $ Ax \leq b$.
- Quadratic programming (QP): $ \min x^T Q x + c^T x $ subject to $ Ax \leq b$.
- Second-order cone programming (SOCP): $ \min c^T x $ subject to $ \Vert A_i x + b_i \Vert_2 \leq c_i^T x + d_i $.
- Semidefinite programming (SDP): $ \min \text{trace}(C^T X) $ subject to $ \text{trace}(A_i^T X) = b_i, \quad X \succeq 0$. That is, linear objective with linear constraints and positive semidefinite constrainted.

# Computing Lyapunov Functions Using Optimization

Our first idea is to parameterize the Lyapunov candidate. Given our systm dynamics: $ \dot x = f(x) $, take some nonlinear basis functions $ \phi(x) $ and define a Lyapunov candidate: $ V(x) = \sum_i \alpha_i \phi_i(x) = \alpha^T \Phi(x) $.

Now, use convex optimization to search over $\alpha$ to satisfy the Lyapunov conditions.

#### The LP version

Sample points $ \\{ x_i \\} $ and find $\alpha$:
  - $ \min_\alpha \sum_i \Vert \dot V (x_i) - \mathbb{1} \Vert_1 $ subject to 
  - $ V(0)=0 $ and $ \forall i \;\; V(x_i) = \alpha^T \Phi(x) \geq \epsilon x_i^T x_i $.
  - $ \dot V(x) \leq 0 $ and $ \forall i \;\; \dot V(x_i) = \left.\frac{\partial V}{\partial x} \right \vert_{x=x_i} f(x_i) = \alpha^T \left.\frac{\partial \Phi}{\partial x} \right \vert_{x=x_i} f(x_i) \leq -\epsilon x_i^T x_i $.

The problems with this approach are: (1) sampling points in higher dimensions is discouraged, (2) it is unclear how to sample the basis functions, and (3) there is no certificate that the Lyapunov conditions are satisfied $ \forall x $.

#### How do we get a "$ \forall x $" certificate?

For $V$, is it fairly simple: We can write $V$ as $ V(x) = \sum_i \alpha_{ij} \phi_i (x) \phi_j (x) $ where $ \alpha_{ij} \geq 0$. This can be written as $ V(x) = \Phi^T(x) G \Phi(x) $ where $ G $ is a symmetric matrix s.t. $G_{ij} = G_{ji} = \alpha_{ij}$.

What about $ \dot V$? We know that $ \dot V(x) = \frac{\partial V}{\partial x} f(x) $. But if we can define some new function $F(x) = -\Phi^T (x) G_2 \Phi(x)$ that satisfies $ \dot V(x) = F(x) $ for all $ x $, then we get a certificate that $ \dot V(x) \leq 0 $ for all $ x $.

Now, if we can write $V$ and $\dot V$ as positive semidefinite constraints, then we can use semidefinite programming to solve the problem, which is convex! This means that optimization is guaranteed to find the global minimum.

#### Example: A linear system

Suppose we have $\dot x = Ax$. Is it stable? 

- We can use the Lyapunov function $ V(x) = x^T P x $ where $ P \succeq 0 $.
- Therefore, $\dot V(x) = 2x^T PAx = x^T P A x + x^T A^T P x $. 
- $ \dot V (x) < 0 $ if $ A^T P + PA \prec 0 $. This is known as *the Lyapunov condition for linear systems*.[^2]

Now, we can write a semidefinite program: 

$$\text{find} \quad \alpha \quad \text{s.t.} $$ 

$$ P = \begin{bmatrix} \alpha_{11} & \alpha_{12} & \cdots \\ \alpha_{21} & \alpha_{22} & \\ \vdots \end{bmatrix} \succeq 0 $$

$$ A^T P + PA \prec 0 $$

And how do we do that? Another parametrization of $Q$: 

$$\text{find} \quad \alpha, \beta \quad \text{s.t.} $$ 

$$ P = \begin{bmatrix} \alpha_{11} & \alpha_{12} & \cdots \\ \alpha_{21} & \alpha_{22} & \\ \vdots \end{bmatrix} \succeq 0 $$

$$ Q = \begin{bmatrix} \beta_{11} & \beta_{12} & \cdots \\ \beta_{21} & \beta_{22} & \\ \vdots \end{bmatrix} \succeq 0 $$

$$ A^T P + PA = -Q $$

#### Robust Stability Analysis

We have a system $ \dot x = Ax$ and we want to prove it is stable, but $A$ is uncertain (e.g. we only know that its entries are in some range). To express this uncertainty, we can write $ A $ as a linear combination of some other matrices $ \\{A_i\\} $: $ A = \sum_i \beta_i A_i $ where $ \beta_i \geq 0 $ and $ \sum_i \beta_i = 1 $.

Now, we can write a semidefinite program:

$$\text{find} \quad P \succ 0 \quad \text{s.t.} $$

$$ \forall A_i \quad A_i^T P + PA_i \prec 0 $$

And if we find such a $P$, then we have a certificate that the system is stable, since $ P \left( \sum_i \beta_i A_i \right) + \left( \sum_i \beta_i A_i \right)^T P \prec 0 $.


<br><br><br>

----

[^1]: More generally, we can use a neural network to represent the Lyapunov function, and people actually do Lyapunov-stable NN. However, the more complex the function, the harder it is to achieve a certificate that the Lyapunov conditions are satisfied $ \forall x $.

[^2] If the system is stable, you can pick any $ Q \succeq 0 $ and solve the *Lyapunov equation* $ A^T P + PA = -Q $ to find a Lyapunov function.