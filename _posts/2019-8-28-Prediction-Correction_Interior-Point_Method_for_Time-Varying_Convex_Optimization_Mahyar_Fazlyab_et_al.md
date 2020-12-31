---
layout: post
use_math: true
title: Understanding "Prediction-Correction Interior-Point Method for Time-Varying Convex Optimization"
---

_This post is based on [**Prediction-Correction Interior-Point Method for Time-Varying Convex Optimization**](https://arxiv.org/abs/1608.07544), a paper whose authors are **Mahyar Fazlyab**, **Santiago Paternain**, **Victor M. Preciado**, and **Alejandro Ribiero**. It was published in **IEEE Transactions on Automatic Control, Vol. 63, No. 7, July 2018**._

_This article can be seen as a complement to the above-mentioned paper, and some level of paraphrasing is to be expected. In this perspective, the paper's notation and equation numbering have been strictly followed so that one may read the paper alongside this post without confusion. Equations with lowercase Roman numeral numbering have been introduced in this article for explanatory purposes, and do not appear in the paper._

## 1 Motivation
The aim of this post is to build an understanding of the paper's main result, equation **(23)**:


$\dot{x}=-\nabla_{xx}^{-1}\phi\left[P\nabla_x\phi+\nabla_{xs}\phi\dot{s}+\nabla_{xc}\phi\dot{c}+\nabla_{xt}\phi\right]$


This equation, as well as others of its form, is referred to in the paper as a _dynamical system_. It is a differential equation whose solution converges—by design—to the optimal solution $x^*$.

## 2 Problem statement
For any time $t\ge 0$, the objective function $f_0$ and the constraints define an optimization problem whose optimal argument $x^*(t)$ is **(1)**:


$x^*(t) := \text{argmin}\, f_0(x,t)$

$\text{s.t.}; f_i(x,t)\le 0,\, i\in[p]$

$f_i^\prime(x,t)=0,\, i\in[q]$


The quantities $x(t)$ and $f_0(x,t)$ can respectively be seen as input and output. The aim is to find the optimal value $x^*(t)$ for which the output is minimized, subject to $p$ inequality constraint functions $f_i(x,t)$ and $q$ linear equality constraint functions $f_i^\prime(x,t)$.

## 3 Assumptions
For all $t\ge 0$, the following conditions are assumed:
1. **Convexity**: constraint functions are convex and the objective function is uniformly strongly convex, meaning there is a unique optimal value $x^*(t)$ at each time.

2. **Slater's condition**: the interior of the feasible region is nonempty, i.e. for each time there exists a solution under the given constraints.

3. **Full rank condition**: the system of equality constraints has infinitely many solutions at each time.

4. The optimization problem has **no exponentially growing function of $t$**.

## 4 Derivation
The following sections will build up to the differential equation (23) whose solution converges towards $x^*(t)$, as defined in (1). At the core of equation (23) are a prediction and correction term which can respectively be seen to _anticipate_ how the optimal solution evolves over time, and _steer_ towards the solution at each time.

Equality constraints may then be added through a method called Langrangian relaxation, while inequality constraints are incorporated through barrier functions. A slack term is added so that the optmization problem can be initialized outwith the feasible set.

### 4.1 Prediction
The unconstrained version of (1) is considered in both this and the following section [3.1, 3.2]; it is defined as **(2)**:


$x^*\left(t\right):= \text{argmin}\, f_0\left(x,t\right)$


The solution $x^*\left(t\right)$ in (2) satisfies the first order optimality condition, meaning the first derivative $\nabla_x f_0\left(x,t\right)$ is equal to zero. As this is true for all $t$ the second derivative must also be null, yielding **(4)**:


$0=\dot{\nabla}_x f_0(x^*(t),t)\Longleftrightarrow 0=\nabla_x$


$0=\dot{\nabla}_x f_0(x^*(t),t)\Longleftrightarrow 0=\nabla_{xx} f_0(x^*(t),t)\dot{x}^*(t)+\nabla_{xt}f_0(x^*(t),t)$


This follows from the chain rule, whereby **(i)**:


$\frac{d f(x(t),\,t)}{d t}=\frac{\partial f}{\partial x}\frac{d x}{d t}+\frac{\partial f}{\partial t}$


From (4) ensues the following expression for $x^*(t)$, **(5)**:


$\dot{x}^*\left(t\right)=-\nabla_{xx}^{-1}f_0\left(x^*\left(t\right),\, t\right)\nabla_{xt}f_0\left(x^*\left(t\right),\, t\right)$


By ensuring that the first order optimality condiion is fulfilled, this yields the prediction term given $x^*\left(t\right)$ known at some time $t$. However, it is not assumed that the optimization problem is initalized at an optimal solution, and the correction term is what will enable the system to close in on the optimal solution.

### 4.2 Correction
The correction term is derived from the Newton method in the limit of infinitesimal steps. Note that the Newton method is conventionally written as **(ii)**:


$x_{n+1}=x_n-\frac{f_0(x(n))}{f_0^\prime(x(n))}$


where $x$ converges onto a root of the function $f_0$. However, the method developed in this paper converges onto an extremum which corresponds to the root of the derivative $f_0^\prime$. Taking this into account and looking at the Newton iteration in discrete time yields **(iii)**:


$x_{n+1}=x-\gamma_n\frac{\nabla_x f_0(x(n),n)}{\nabla_{xx}f_0(x(n),n)}\Longleftrightarrow\frac{x_{n+1}-x_n}{\gamma_n}=-\nabla_{xx}^{-1}f_0(x(n),n)\nabla_x f_0(x(n),n)$


where $\gamma_n$ is a stepsize parameter. Note that as the objective function is assumed to be uniformly strongly convex, this will converge onto the minimum. In the limit where $n\rightarrow t$, $\gamma_n\rightarrow 0$, and $(x_{n+1}-x_n)\rightarrow 0$ this becomes **(3)**:


$\dot{x}(t)=-\nabla_{xx}^{-1}f_0(x(t),t)\nabla_x f_0(x(t),t)$


The solution to this differential equation converges onto a neighborhood around $x^*(t)$.

### 4.3 Equality constraints
Equality constraints are dealt with following Lagrangian relaxation; they are therefore omitted in the final result without loss of generality.

### 4.4 Inequality constraints
The following sections [3.4.1, 3.4.2] are based on a version of (1) with only inequality constraints **(15)**:


$x^*\left(t\right):= \text{argmin}\, f_0\left(x,t\right), \text{s.t.} f_i\left(x,t\right)\le 0,\, i\in\left[p\right]$

#### 4.4.1 Barrier term
Using barrier functions, the formulation in (15) can be rewritten as **(18)**:


$x^*\left(t\right):=\text{argmin}\, f_0\left(x,t\right)+\sum_{i=1}^p\mathbb{I}_-\left( f_i\left(x,t\right)\right)$


where $$\mathbb{I}_-$$ is defined such that $$\mathbb{I}_-(u)=0$$ if $u\le 0$ and $$\mathbb{I}_-(u)=\infty$$ if $u>0$. It is clear from its definition that $$\mathbb{I}_-$$ is a barrier term, given $x^*(t)$ is defined only when all the inequalities are respected. Furthermore, $$\mathbb{I}_-(u)$$ can be improved by approximating it with the smooth, closed convex function $-\frac{1}{c}\log(-u)$, where $c>0$ is an arbitrary barrier parameter. You may convince yourself by playng with [this graph](https://www.desmos.com/calculator/3oxnjvvtyw) This yields a smooth version of (18), which is **(19)**:


$x^*\left(t\right):=\text{argmin}\, f_0\left(x,t\right)-\frac{1}{c\left(t\right)}\sum_{i=1}^p\log\left(-f_i\left(x,t\right)\right)$


where $x$ is within the feasible set $\mathcal{D}(t)$ defined by all inequalities, and $c$ is a time-dependent positive barrier parameter. This barrier parameter ensures that inequality constraints are not violated, and therefore allows these to be included into the new objective function defined in (19).

#### 4.4.2 Slack term
In order to circumvent the requirement of $x$ falling within the feasible set from time $t=0$, a slack term $s(t)$ is added which satisfies $s(0)>\text{max}_{i\in[p]}\,f_i(x(0),0)$ so that $x(0)\in\hat{\mathcal{D}}(0)$, with $\hat{\mathcal{D}}(t)$ an open set containing $\mathcal{D}(t)$. In other words, the specification on $s(0)$ ensures that the inital condition lies within the enlarged set $\hat{\mathcal{D}}(0)$, by definition. The approximate optimal trajectory is therefore **(20)**:


$\hat{x}^*(t):=\text{argmin}\, f_0(x,t)-\frac{1}{c(t)}\sum_{i=1}^p\text{log}\left(s(t)-f_i(x,t)\right)$


The optimization can therefore be initialized for any $x$. However, in order for the approximation error to vanish, the barrier parameter $c(t)$ must asymptotically diverge to infinity, while the slack parameter $s(t)$ must asymptotically vanish. This signifies that, over time, $\hat{x}^*(t)$. will converge to $x^*(t)$.

### 4.5 Tying things together
Lagrangian relaxation and the barrier term respectively serve to incorporate equality and inequality constraints into the objective function, while the slack term ensures that the optimization can be initialized with any value. These terms therefore all work towards building the entire problem into a single objective function, while enabling the optimization to start from a random point. This objective function is defined as **(22)**:


$\Phi(x,c,s,t):=f_0(x,t)-\frac{1}{c}\sum_{i=1}^p\text{log}\left(s-f_i(x,t)\right)$


This updated objective function is fed into the prediction-correction differential equation to yield the following dynamical system **(23)**:


$\dot{x}=-\nabla_{xx}^{-1}\Phi\left[P\nabla_x\Phi+\nabla_{xs}\Phi\dot{s}+\nabla_{xc}\Phi\dot{c}+\nabla_{xt}\Phi\right]$


where $P\succeq\alpha I_n$ for some $\alpha> 0$. The first term in the brackets corresponds to the correction component as defined in (3), while the subsequent terms correspond to the the prediction, analogous to (5). $P$ can be interpreted as a weighting which influences the convergence rate of $x(t)$ towards $x^*(t)$ based on $\alpha$, though furher explanation is found in the paper.

All the terms of (23) have now been explained, and hopefully this article has helped build an intuitive sense of their role. _Please do not hesitate to leave your questions and comments in the section below, and I will do my best to respond._

##### Notation
Let $\mathbb{R}$, $\mathbb{R}_+$, and $\mathbb{R}_{++}$ be the set of real, nonnegative, and positive numbers. The set $\{1,...,n\}$ is denoted by $[n]$. $I_n$ denotes the $n$-dimensional identity matrix. $$\mathbb{S}^n$$ denotes the space of $n\times n$ symmetric matrices. The gradient of a function $$f(x,t):\mathbb{R}^n\times\mathbb{R}_+\rightarrow\mathbb{R}$$ with respect to $$x\in\mathbb{R}^n$$ is denoted by $\nabla_x f(x,t)$. The _partial_ derivatives of $\nabla_x f(x,t)$ with respect to $x$ and $t$ are denoted by $\nabla_{xx} f(x,t)$ and $\nabla_{xt} f(x,t)$, respectively.
The variable $$x\in\mathbb{R}^n$$ is considered, with $t\ge 0$ a continuous time index. A time-varying objective function is defined as $$f_0 :\mathbb{R}^n\times\mathbb{R}_+\rightarrow\mathbb{R}$$ taking values $f_0 (x,t)$; $p$ time-varying inequality constraint functions are defined as $$f_i :\mathbb{R}^n\times\mathbb{R}_+\rightarrow\mathbb{R}$$ taking values $f_i (x,t)$ for $i\in[p]$; and $q$ time-varying affine equality constraint functions are defined as $$f_i^\prime :\mathbb{R}^n\times\mathbb{R}_+\rightarrow\mathbb{R}$$ taking values $f_i^\prime = a_i(t)^T x-b_i(t)$ for $i\in[q]$. It is assumed that $f_i(x,t),\, i\in\{0\}\cup[p]$ and $f_i^\prime(x,t),\, i\in[q]$ are twice continuously differentiable with respect to $x$ and piecewise continuously differentiable with respect to $t$, for all $$(x,t)\in\mathbb{R}^n\times\mathbb{R}_+$$.
