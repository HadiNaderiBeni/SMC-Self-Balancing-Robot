# Sliding Mode Control of a Self-Balancing Robot

This repository contains the simulation and practical implementation of a Sliding Mode Controller (SMC) for a two-wheeled self-balancing robot using MATLAB and Simulink.

The objective of this project is to stabilize the robot around the upright equilibrium position using a robust nonlinear control method. Since a self-balancing robot is naturally unstable, the controller must react quickly to tilt deviations and disturbances in order to keep the robot balanced.

---

## Project Overview

A two-wheeled self-balancing robot behaves similarly to an inverted pendulum mounted on wheels. The upright position is unstable, so feedback control is required to prevent the robot from falling.

The robot state is defined as:

```text
x = [gamma, theta, gamma_dot, theta_dot]
