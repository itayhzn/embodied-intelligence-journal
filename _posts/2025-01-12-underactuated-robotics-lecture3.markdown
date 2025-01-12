---
layout: post
title:  Underactuated Robotics - Lecture 3
date:   2025-01-12 13:49:00 +0200
categories: underactuated-robotics lecture
---

> This is part of a series of posts on the course [MIT 6.8210: Underactuated Robotics](https://underactuated.csail.mit.edu/Spring2024/index.html) by [Prof. Russ Tedrake](https://locomotion.csail.mit.edu/russt.html). The goal is to document the key concepts and takeaways for future reference. These are <i>not</i> actual lecture notes, but a summary of the key points covered in the lecture and my thoughts on them. The official course material is available here: [lecture videos on Youtube](https://www.youtube.com/playlist?list=PLkx8KyIQkMfU5szP43GlE_S1QGSPQfL9s) \| [lecture notes](https://underactuated.csail.mit.edu)

In the previous lecture, we explored the dynamics of a simple pendulum. We derived the equations of motion and discussed the stability of the system. In this lecture, we will continue our discussion on the simple pendulum and explore how we can control it, that is, what happens when $ u $ stops being 0. How does that change the dynamics of the system? Or better yet, how should we change $u$ to make the pendulum go to a specific point?

In this lecture we will discuss Dynamic Programming. The agenda will be as follows:
 - We will talk about the philosophy of control as an optimization problem.
 - We will see an example of the Double Integrator.
 - We will discuss Numerical Dynamic Programming.
 - We will talk about the intuition behind value functions.
 - And finally, we will see how to solve the pendulum problem using Dynamic Programming.

 
