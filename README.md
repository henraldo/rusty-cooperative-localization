# UAV-UGV Cooperative Localization with Rust

This project generates a user-configurable simulation of cooperative localization between a UAV and UGV, using either an Extended Kalman Filter (EKF), or an Unscented Kalman Filter (UKF).

Future releases will also provide the means to tune EKF or UKF performance and stability via Monte Carlo
Truth Model Testing (TMT) - where Monte Carlo trials are run to collect NEES and NIS statistics for assessing dynamic performance of the implemented filter.

Time history data generated from the simulation (along with filter implementation settings) are written to CSV files and placed in a user-defined subdirectory under `/project-root/simulation_output/` for post-processing and analysis.

# Simulation Overview

## State-Space System Definition
The combined nonlinear state-space system has the form:

$$\boldsymbol{\dot{x}}(t) = \boldsymbol{f}(x, u) + \boldsymbol{\Gamma} \boldsymbol{\tilde{w}}(t)$$
$$\boldsymbol{y}(t) = \boldsymbol{h}(x) + \boldsymbol{\tilde{v}}(t)$$

### UGV Model
The equations of motion used to define the UGV dynamics model are as follows:

$$\dot{\xi}_g = v_g \cos\theta_g + \tilde{w}_{x,g}$$

$$\dot{\eta}_g = v_g \sin\theta_g + \tilde{w}_{y,g}$$

$$\dot{\theta}_g = \frac{v_g}{L}\tan\phi_g + \tilde{w}_{\omega,g}$$

with control inputs

$$\boldsymbol{u}_g (t) = \begin{bmatrix}
    v_g \\
    \phi_g
\end{bmatrix}$$

and AWGN process noise

$$\boldsymbol{\tilde{w}}_g (t) = \begin{bmatrix}
    \tilde{w}_{x,g} \\
    \tilde{w}_{y,g} \\
    \tilde{w}_{\omega,g}
\end{bmatrix}$$

Here $L$ is the wheel base length of the UGV, and velocity and steering angle control inputs are limited to

$$v_{g, \max} = 3 \, \mathrm{m/s}$$

and

$$\phi_g \in \left[-\frac{5\pi}{12}, \frac{5\pi}{12}\right] \, \mathrm{rad}$$

### UAV Model
The equations of motion used to define the UAV dynamics model are as follows:

$$\dot{\xi}_a = v_a \cos\theta_a + \tilde{w}_{x,a}$$

$$\dot{\eta}_a = v_a \sin\theta_a + \tilde{w}_{y,a}$$

$$\dot{\theta}_a = \omega_a + \tilde{w}_{\omega,a}$$

with control inputs

$$\boldsymbol{u}_a (t) = \begin{bmatrix}
    v_a \\
    \omega_a
\end{bmatrix}$$

and AWGN process noise

$$\boldsymbol{\tilde{w}}_a (t) = \begin{bmatrix}
    \tilde{w}_{x,a} \\
    \tilde{w}_{y,a} \\
    \tilde{w}_{\omega,a}
\end{bmatrix}$$

The UAV velocity and steering rate inputs are limited to

$$v_a \in [10, 20] \, \mathrm{m/s}$$

and

$$\omega_a \in \left[-\frac{\pi}{6}, \frac{\pi}{6}\right] \, \mathrm{rad/s}$$

### Combined System Model
The combined system state, control input, and disturbance input vectors are defined as

$$\boldsymbol{x}(t) = \begin{bmatrix}
    \xi_g \\
    \eta_g \\
    \theta_g \\
    \xi_a \\
    \eta_a \\
    \theta_a
\end{bmatrix}$$

$$\boldsymbol{u}(t) = \begin{bmatrix}
    \boldsymbol{u}_g \\
    \boldsymbol{u}_a
\end{bmatrix}$$

$$\boldsymbol{\tilde{w}}(t) = \begin{bmatrix}
    \boldsymbol{\tilde{w}}_g \\
    \boldsymbol{\tilde{w}}_a
\end{bmatrix}$$

and the combined nonlinear state-space system is defined as

$$\boldsymbol{\dot{x}}(t) = \begin{bmatrix}
    u_1 \cos x_3 \\
    u_1 \sin x_3 \\
    \frac{u_1}{L} \tan u_2 \\
    u_3 \cos x_6 \\
    u_3 \sin x_6 \\
    u_4
\end{bmatrix} + \boldsymbol{\Gamma} \boldsymbol{\tilde{w}}(t)$$

Lastly, the combined system's sensing model is defined as

$$\boldsymbol{y}(t) = \begin{bmatrix}
    \tan^{-1}\left( \frac{x_5-x_2}{x_4-x_1} \right) - x_3 \\
    \sqrt{(x_1 - x_4)^{2} + (x_2 - x_5)^{2}} \\
    \tan^{-1}\left( \frac{x_2-x_5}{x_1-x_4} \right) - x_6 \\
    x_4 \\
    x_5
\end{bmatrix} + \boldsymbol{\tilde{v}}(t)$$

## Extended Kalman Filter Overview
The online nonlinear trajectory for the filter is updated at each time step, as part of the prediction
stage and the nonlinear dynamics model is propagated forward using BOOST's `odeint::runge_kutta_dopri5` ODE solver.
A new state estimate

 $$\boldsymbol{\hat{x}}^{-}_{k+1} = \boldsymbol{f}(\boldsymbol{\hat{x}}^{+}_k , \boldsymbol{u}_k , \boldsymbol{w}_k = 0)$$

is predicted by propagating the dynamics model and the new filter covariance $\boldsymbol{P}^{-}_{k+1}$ is predicted as

$$\boldsymbol{P}^{-}_{k+1} = \boldsymbol{\tilde{F}}_k \boldsymbol{P}^{+}_k \boldsymbol{\tilde{F}}^{T}_k + \boldsymbol{\tilde{\Omega}}_k \boldsymbol{Q} \boldsymbol{\tilde{\Omega}}^{T}_k$$

where $\boldsymbol{\tilde{F}}_k$ is the discrete-time mapping of the state transition matrix Jacobian
at time $t_k$. This is updated prior to the new covariance prediction as

$$\boldsymbol{\tilde{F}}_k \big|_{\hat{x}^{+}_k, u_k, w_k = 0} = \boldsymbol{I} + \Delta t \cdot \boldsymbol{\tilde{A}} \big|_{(\hat{x}^{+}_k, u(t_k),w(t_k)=0)}$$

where the state transition matrix Jacobian $\boldsymbol{\tilde{A}}$ is

$$\boldsymbol{\tilde{A}} = \begin{bmatrix}
    0 & 0 & -u_1 \sin x_3 & 0 & 0 & 0 \\
    0 & 0 & u_1 \cos x_3 & 0 & 0 & 0 \\
    0 & 0 & 0 & 0 & 0 & 0 \\
    0 & 0 & 0 & 0 & 0 & -u_3 \sin x_6 \\
    0 & 0 & 0 & 0 & 0 & u_3 \cos x_6 \\
    0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}_{(x_k,u_k)}$$

Next, after a new set of observations are received at measurement $k+1$, the filter's correction
stage corrects the predicted state estimate and covariance to render a final new state estimate

$$\boldsymbol{\hat{x}}^{+}_{k+1}$$

The correction stage begins by calculating an updated discrete-time observation matrix Jacobian

$$\boldsymbol{\tilde{H}}_{k+1} = \frac{\partial \boldsymbol{h}}{\partial \boldsymbol{x}} \big|_{\hat{x}^{-}_{k+1}}$$

which is computed to be

$$\boldsymbol{\tilde{H}}_{k+1} = \begin{bmatrix}
    \frac{x_5 - x_2}{\left( \frac{(x_5 - x_2)^2}{(x_4-x_1)^2} + 1 \right) (x_4 - x_1)^2} & -\frac{1}{(x_4-x_1)\left(\frac{(x_5-x_2)^2}{(x_4-x_1)^2} + 1\right)} & -1 & -\frac{x_5-x_2}{(x_4-x_1)^2\left(\frac{(x_5-x_2)^2}{(x_4-x_1)^2} + 1\right)} & \frac{1}{(x_4-x_1)\left(\frac{(x_5-x_2)^2}{(x_4-x_1)^2} + 1\right)} & 0 \\
    \frac{x_1 - x_4}{\sqrt{(x_1-x_4)^2 + (x_2-x_5)^2}} & \frac{x_2 - x_5}{\sqrt{(x_1-x_4)^2 + (x_2-x_5)^2}} & 0 & \frac{x_4 - x_1}{\sqrt{(x_1-x_4)^2 + (x_2-x_5)^2}} & \frac{x_5 - x_2}{\sqrt{(x_1-x_4)^2 + (x_2-x_5)^2}} & 0 \\
    -\frac{x_2-x_5}{(x_1-x_4)^2\left(\frac{(x_2-x_5)^2}{(x_1-x_4)^2} + 1\right)} & \frac{1}{(x_1-x_4)\left(\frac{(x_2-x_5)^2}{(x_1-x_4)^2} + 1\right)} & 0 & \frac{x_2 - x_5}{\left(\frac{(x_2 - x_5)^2}{(x_1-x_4)^2}+1\right) (x_1 - x_4)^2} & -\frac{1}{(x_1-x_4)\left(\frac{(x_2-x_5)^2}{(x_1-x_4)^2} + 1\right)} & -1 \\
    0 & 0 & 0 & 1 & 0 & 0 \\
    0 & 0 & 0 & 0 & 1 & 0
\end{bmatrix}_{\hat{x}^{-}_{k+1}}$$

and the observation estimate for measurement $k+1$ is

$$\boldsymbol{\hat{y}}_{k+1} = \boldsymbol{h}(\boldsymbol{\hat{x}}^{-}_{k+1})$$

The measurement and covariance innovations are then calculated as

$$\boldsymbol{\tilde{e}}_{y, k+1} = \boldsymbol{y}_{k+1} - \boldsymbol{\hat{y}}_{k+1}$$

$$\boldsymbol{S}_{k+1} = \boldsymbol{\tilde{H}}_{k+1} \boldsymbol{P}^{-}_{k+1} \boldsymbol{\tilde{H}}^{T}_{k+1} + \boldsymbol{R}$$

Finally, the Kalman Gain is updated, and the state estimate and filter covariance for measurement step $k+1$ are corrected as follows:

$$\boldsymbol{K}_{k+1} = \boldsymbol{P}^{-}_{k+1} \boldsymbol{\tilde{H}}_{k+1}^{T} \boldsymbol{S}_{k+1}^{-1}$$

$$\boldsymbol{\hat{x}}^{+}_{k+1} = \boldsymbol{\hat{x}}^{-}_{k+1} + \boldsymbol{K}_{k+1} \boldsymbol{\tilde{e}}_{y, k+1}$$

$$\boldsymbol{P}^{+}_{k+1} = (\boldsymbol{I} - \boldsymbol{K}_{k+1}\boldsymbol{\tilde{H}}_{k+1})\boldsymbol{P}^{-}_{k+1}$$

## Unscented Kalman Filter Overview
The UKF is a fully nonlinear filter; formulated by applying an Unscented Transform to the model dynamics,
since it is easier to directly approximate mean and covariance between pdf's than through the nonlinear state
transition functions. Instead of using a Taylor Series expansion to linearize the model dynamics, sigma points
are generated from the current state pdf

$$p(\boldsymbol{x}_k \mid \boldsymbol{y}_{1:k})$$

and propagated to the next time step through the nonlinear dynamics to approximate the first two moments of the pdf
at the future time step. Where $2n + 1$ sigma points are required to accurately sketch out

$$\boldsymbol{E}[\boldsymbol{x}_k] = \boldsymbol{\mu}_x \quad and \quad \mathrm{cov}(\boldsymbol{x}_k) = \boldsymbol{P}_{xx}$$

from

$$\boldsymbol{x}_k \sim p(\boldsymbol{x}_k \mid \boldsymbol{y}_{1:k})$$

Given that

$$\boldsymbol{P}_{xx} = \boldsymbol{S}^T \boldsymbol{S} \quad and \quad \boldsymbol{S} = \mathrm{chol}(\boldsymbol{P}_{xx})$$

sigma points are computed as:

$$\boldsymbol{\chi}^0 = \boldsymbol{\mu}_x$$

$$\boldsymbol{\chi}^i = \begin{cases}
    \boldsymbol{\mu}_x + (\sqrt{n + \lambda}) \cdot \boldsymbol{S}^{j,T}, & i = 1,\dots,n, \; j=1,\dots,n \\
    \boldsymbol{\mu}_x - (\sqrt{n + \lambda}) \cdot \boldsymbol{S}^{j,T}, & i = n+1,\dots,2n, \; j=1,\dots,n
\end{cases}$$

Where $n$ is the number of states in $\boldsymbol{x}_k$ and $\lambda$ is the scaling parameter defined by:

$$\lambda = \alpha^2 (n + \kappa) - n$$

The propagated sigma points can then be recombined and used to estimate the new mean and covariance.
The variable $\alpha$ controls the spread of the sigma points and as its magnitude is reduced, the estimate
performance approaches that of the EKF.

The UKF algorithm also incorporates a prediction step and measurement update/correction step.
However, a nonlinear transformation will be performed at both steps - approximating the statistics using
the generated sigma points. The prediction step is computed as follows, given initial values for $\boldsymbol{\hat{x}}^{+}_k$, $\boldsymbol{P}^{+}_k$, and $\boldsymbol{S}_k$:

$$\boldsymbol{\chi}^0_k = \boldsymbol{\hat{x}}^{+}_k$$

$$\boldsymbol{\chi}^i_k = \begin{cases}
    \boldsymbol{\hat{x}}^{+}_k + (\sqrt{n + \lambda}) \cdot \boldsymbol{S}^{j,T}_k, & i = 1,\dots,n, \; j=1,\dots,n \\
    \boldsymbol{\hat{x}}^{+}_k - (\sqrt{n + \lambda}) \cdot \boldsymbol{S}^{j,T}_k, & i = n+1,\dots,2n, \; j=1,\dots,n
\end{cases}$$

Then each of the sigma points at $t = t_k$ is propagated through the nonlinear dynamics function $\boldsymbol{f}$
(again using the BOOST `odeint::runge_kutta_dopri5` solver) to generate predicted points

$$\boldsymbol{\chi}^{-i}_{k+1} \quad at \quad t = t_{k+1}$$

The prediction step is concluded by recombining the resultant points to compute the predicted mean and covariance by:

$$\boldsymbol{\hat{x}}^-_{k+1} \approx \sum_{i=0}^{2n} \omega^i_m \cdot \boldsymbol{\chi}^{-i}_{k+1}$$

$$\boldsymbol{P}^-_{k+1} \approx \sum_{i=0}^{2n} \omega^i_c \cdot (\boldsymbol{\chi}^{-i}_{k+1} - \boldsymbol{\hat{x}}^-_{k+1})(\boldsymbol{\chi}^{-i}_{k+1} - \boldsymbol{\hat{x}}^-_{k+1})^T + \boldsymbol{Q}$$

where

$$\omega^0_m = \frac{\lambda}{n + \lambda}, \quad \omega^0_c = \frac{\lambda}{n + \lambda} + 1 - \alpha^2 + \beta, \quad \omega^i_m = \omega^i_c = \frac{1}{2(n + \lambda)}$$

The filter correction step has a form similar to the prediction step. Given

$$\boldsymbol{\hat{x}}^-_{k+1}, \quad \boldsymbol{P}^-_{k+1}, \quad and \quad observation \quad \boldsymbol{y}_{k+1}$$

the correction step begins by generating another set of sigma points at measurement $k+1$ for the span of measurement
estimates $\boldsymbol{\gamma}^i_{k+1}$ by passing the predicted state sigma points through the nonlinear sensor dynamics function $\boldsymbol{h}$.

The updated measurement mean and covariance are then computed by:

$$\boldsymbol{\hat{y}}_{k+1} \approx \sum_{i=0}^{2n} \omega^i_m \cdot \boldsymbol{\gamma}^i_{k+1}$$

$$\boldsymbol{P}_{yy,k+1} \approx \sum_{i=0}^{2n} \omega^i_c \cdot (\boldsymbol{\gamma}^i_{k+1} - \boldsymbol{\hat{y}}_{k+1})(\boldsymbol{\gamma}^i_{k+1} - \boldsymbol{\hat{y}}_{k+1})^T + \boldsymbol{R}$$

Next, the state-measurement cross-covariance and Kalman gain are calculated:

$$\boldsymbol{P}_{xy,k+1} \approx \sum_{i=0}^{2n} \omega^i_c \cdot (\boldsymbol{\chi}^{-i}_{k+1} - \boldsymbol{\hat{x}}^-_{k+1})(\boldsymbol{\gamma}^i_{k+1} - \boldsymbol{\hat{y}}_{k+1})^T$$

$$\boldsymbol{K}_{k+1} = \boldsymbol{P}_{xy,k+1} \boldsymbol{P}_{yy,k+1}^{-1}$$

The state and covariance are then updated as:

$$\boldsymbol{\hat{x}}^+_{k+1} = \boldsymbol{\hat{x}}^-_{k+1} + \boldsymbol{K}_{k+1} (\boldsymbol{y}_{k+1} - \boldsymbol{\hat{y}}_{k+1})$$

$$\boldsymbol{P}^+_{k+1} = \boldsymbol{P}^-_{k+1} - \boldsymbol{K}_{k+1} \boldsymbol{P}_{yy,k+1} \boldsymbol{K}_{k+1}^T$$

# Project Build
...update this section