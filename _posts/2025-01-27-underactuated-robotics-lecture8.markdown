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
  - Parametric family of $ \text{trig}/\text{poly} $ functions for Lyapunov (i.e. polynomial in $ \cos \theta, \sin \theta, \dot \theta $, etc.)
- Output: 
  - Coefficients of the polynomial
  - Certificate that Lyapunov conditions satisfied $ \forall x $.

