# Adaptive PID Controller with Error-Based Parameter Adjustment

## Overview

This project implements an Adaptive PID (Proportional-Integral-Derivative) Controller in Python, utilizing an Error-Based Parameter Adjustment method. This approach dynamically adjusts the controller's parameters (Kp, Ki, Kd) based on the current error, error history, and error trends. The controller is designed to automatically tune itself in real-time, making it suitable for systems with varying dynamics or those that are difficult to tune manually.

## Method

The Error-Based Parameter Adjustment method used in this controller works as follows:

1. **Proportional (Kp) Adjustment**: Adapts based on the error trend. Increases Kp when the error is steady (to improve responsiveness) and decreases it when the error is changing rapidly (to reduce overshooting).

2. **Integral (Ki) Adjustment**: Adapts based on the long-term error sum. Increases Ki when there's a persistent error (to eliminate steady-state error) and decreases it otherwise.

3. **Derivative (Kd) Adjustment**: Adapts based on the rate of error change. Increases Kd when there are rapid changes in error (to provide damping) and decreases it when the error change is small.

This method allows the controller to continuously optimize its behavior based on the system's response, potentially improving performance across a wide range of operating conditions.

## Features

- Real-time adaptation of PID parameters using Error-Based Parameter Adjustment
- Anti-windup mechanism with integral decay
- Error history tracking for improved adaptation decisions
- Configurable parameter ranges and adaptation rates
- Reset functionality to clear controller state

## Requirements

- Python 3.6+
- NumPy