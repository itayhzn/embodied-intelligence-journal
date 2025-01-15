---
layout: post
title:  Underactuated Robotics - Lecture 5
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

