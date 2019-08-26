---
layout: post
title: Prediction-Correction Interior-Point Method for Time-Varying Convex Optimization
---

_This post is based on the paper mentioned in the title, whose authors are **Mahyar Fazlyab**, **Santiago Paternain**, **Victor M. Preciado**, and **Alejandro Ribiero**. It was published in **IEEE Transactions on Automatic Control, Vol. 63, No. 7, July 2018**._

_Please note that the paper's notation will be followed as closesly as possible, down to the equation numbering. This is done so that one may read the paper alongside this post without any confusion._

The aim of this post is to explain the terms that come up in the paper's main equation, equation (23):
$$ \dot{x}=-\nabla_{xx}^{-1}\phi\[P\nabla_x\phi+\nabla_{xs}\phi\dot{s}+\nabla_{xc}\phi\dot{c}+\nabla_{xt}\phi\]$$
