---
layout: post
title:  Underactuated Robotics - Lecture 4
date:   2025-01-14 20:16:00 +0200
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

In the previous lecture, we discussed control as an optimization problem. We saw an example of the Double Integrator and demonstrated the Dynamic Programming algorithm for control. The key idea was that a simple grid-based search algorithm can find an (almost)-optimal control policy (up to some caveats that arise to due the discretization).

We will now try to understand "$ \lim_{\Delta x, \Delta u, \Delta t \to 0} \text{Dynamic Programming}$".

# Recap: Dynamic Programming
- Discrete time $n$, state $s[n]$, action $a[n]$.
- Cost function $\ell(s[n],a[n])$, and transition/time function $s[n+1] = f(s[n],a[n])$.
- Additive cost given a set of actions $a[n]$ and states $s[n]$, i.e. cost of a trajectory is $ J[n] = \sum_{k=1}^{N-1} \ell(s[k],a[k])$. 
- Cost-to-go function using the Bellman Equation: $ \forall s,\; J^* (s) = \min _{a} \left[ \ell(s,a) + J (f(s,a)) \right] $.
- The optimal policy is given by $ \pi^* (s) = \arg \min _{a} \left[ \ell(s,a) + J^ \ast (f(s,a)) \right] $.

# Going Continuous

| | Discrete | Continuous |
|---|---|---|
| State | $ s $ | $ x $ |
| Action | $ a $ | $ u $ |
| Time | $ n $ | $ t $ |
| Transition | $ s[n+1] = f(s[n],a[n]) $ | $ \dot x (t) = f(x(t),u (t)) $ |
| Cost | $ \sum_n \ell_{\text{discrete}} (s[n],a[n]) $ | $ \int_0^\infty dt \ell_{\text{continuous}} (x,u) $  |
| Cost-to-go | Bellman equation: $ \forall s,\; J^* (s)= \min _a \left[ \ell _d(s,a) + J^* (f_d(s,a)) \right] $  | Hamilton-Jacobi-Bellman (HJB) equation: $ \forall x, \; 0 = \min _u \left[ \ell _c (x,u) + \frac{\partial J^* }{\partial x} (f_c(x,u)) \right] $ |
| Optimal strategy | like above, but with $ \arg\min_a $ | like above, but with $ \arg\min_u $ |

### Informal derivation of HJB from Bellman equation

First, approximate the continuous-time dynamics using a small time step $ dt $ ("Euler integration"):

$$ 
  x[t+dt] \approx x[t] + dt \cdot f_c(x[n],u[n]) \;\;\;\;;\;\;\;\;
  \ell_d(x,u) \approx dt\cdot \ell_c(x,u)
$$

Substutiting that in the discrete Bellman equation, we get:

$$ 
  J^* (x) = \min _u \left[ dt \ell_c(x,u) + J^* \left( x + dt \cdot f_c(x,u) \right) \right]
$$

We will now use the following approximation: $ J^* \left( x + dt \cdot f_c(x,u) \right) \approx J^* (x) + \frac{\partial J^* (x)}{\partial x} dtf_c(x,u) $, and get:

$$
  J^* (x) = \min _u \left[ dt \ell_c(x,u) + J^* (x) + \frac{\partial J^* (x)}{\partial x} dtf_c(x,u) \right]
$$

Subtracting $ J^* (x) $ from both sides and dividing by $ dt $, we get:

$$
  0 = \min _u \left[ \ell_c(x,u) + \frac{\partial J^* (x)}{\partial x} f_c(x,u) \right]
$$

**Intuition behind HJB**:
Using the chain rule, $ \frac{dJ^* }{dt} = \frac{\partial J^* }{\partial x} \frac{dx}{dt} $, and since $ \dot x = f_c(x,u) $, we get $ \frac{dJ^* }{dt} = \frac{\partial J^* }{\partial x} f_c(x,u) $. This effectively means that for the optimal $ u^* $, we get $ \frac{dJ^* }{dt} = - \ell_c(x,u^* ) $, i.e. the rate of change of the cost-to-go function is equal to the cost of the optimal control policy.

**HJB Sufficiency Theorem**:
If $ J^* $ is a solution to the HJB equation, then $ J^* $ is the optimal cost-to-go function.

# Example: Double Integrator with Quadratic Cost

Consider the double integrator system: $ \ddot q = u $. The cost function is given by $ \ell(x,u) = q^2 + \dot q^2 +  u^2 $. The goal is to minimize the cost-to-go function $ J^* (x) $. We will now prove that the optimal policy is $ \pi^* (x) = -q-\sqrt 3 \dot q $ and the optimal cost-to-go function is $ J^* (x) = \sqrt 3 q^2 + 2q \dot q + \sqrt 3 \dot q^2 $.

**Proof**:
First, recall that:
 - $ x = \begin{bmatrix} q \\ \dot q \end{bmatrix} $
 - $ \frac{\partial J^* }{\partial x} = \begin{bmatrix} \frac{\partial J^* }{\partial q} &  \frac{\partial J^* }{\partial \dot q} \end{bmatrix}  $
 - $ f_x (x,u) = \begin{bmatrix} \dot q \\ u \end{bmatrix} $

To show that $J^* $ and $\pi^* $ are optimal, we need to show that:

$$ 0 = \min _u \left[\ell(x,u) + \frac{\partial J^*}{\partial x} f_c(x,u) \right] $$

Substituting the values, we get:

$$ 0 = \min _u \left[ q^2 + \dot q^2 + u^2 + \begin{bmatrix} \frac{\partial J^* }{\partial q} &  \frac{\partial J^* }{\partial \dot q} \end{bmatrix} \begin{bmatrix} \dot q \\ u \end{bmatrix} \right] = \min_u \left[ q^2 + \dot q^2 + u^2 + \frac{\partial J^* }{\partial q} \dot q + \frac{\partial J^* }{\partial \dot q} u \right] $$

It is easy to see that $ \frac{\partial J^* }{\partial q} = 2\sqrt 3 q + 2 \dot q $ and $ \frac{\partial J^* }{\partial \dot q} = 2q + 2\sqrt 3 \dot q $.
We therefore get:

$$ 0 = \min_u \left[ q^2 + \dot q^2 + u^2 + 2\sqrt 3 q \dot q + 2\dot q^2 + 2qu + 2\sqrt 3 \dot q u \right] $$

Notice that this function is convex in $u$, and thus to find the optimal $ u^* $, we differentiate the above equation w.r.t. $ u $ and set it to 0:

$$ 0 = 2u + 2q + 2\sqrt 3 \dot q \implies u^* = - q -\sqrt 3 \dot q $$

It is interesting to see that $ J^* (x) = x^T \begin{bmatrix} \sqrt 3 & 1 \\ 1 & \sqrt 3 \end{bmatrix} x $.

# Linear Quadratic Regulator (LQR)

The example above is actually a special case of the **Linear Quadratic Regulator (LQR)** problem. The LQR problem is a special case of the HJB equation where the dynamics are linear $ \dot x = Ax + Bu $ and the cost function is quadratic $ \ell(x,u) = x^T Q x + u^T R u $ ($ Q\geq 0, R>0 $ are positive (semi-)definite). The LQR problem is a well-studied problem in control theory and has a closed-form solution.

In the infinite horizon case, the optimal cost-to-go function is a quadratic form: $ J^* (x) = x^T S x $ (for some $ S \geq 0$).
In this case, $ \frac{\partial J}{\partial x} = 2x^T S $, and trying to find the optimal control policy $ u^* $, we get:

$$ 0 = \min_u \left[ x^T Q x + u^T R u + 2x^T S (Ax + Bu) \right] $$

Deriving w.r.t. $ u $ and setting to 0, we get:

$$ 0 = 2Ru + 2B^T S x \implies u^* = -R^{-1} B^T S x $$

We ended up with a linear control policy! We usually denote it as $ u^* = -Kx $, where $ K = R^{-1} B^T S $ is the **feedback gain matrix**.

If we substitute $ u^* $ back into the HJB equation, we get the **Riccati equation**, which is optimality condition for the LQR problem:

$$ 0 = Q - SBR^{-1} B^T S + A^T S + SA $$
