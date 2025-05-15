# Backstepping Control of Drone

## Introduction
Drones are widely used for monitoring, rescue, supervision, and as aerial base stations. In the military, they reduce human losses and provide precise monitoring and strikes. Traditional PID controllers are commonly used but lack robustness for the nonlinear quadrotor system. Backstepping control, based on Lyapunov stability, is robust to parametric variations and ensures system stability.

---

## Quadrotor Dynamic Modeling

### Quadrotor Configuration
![quadrotor_config (1)](https://github.com/user-attachments/assets/5adbeafc-5d8f-46b1-a386-38cfcd622d14)

\date{}
\documentclass[a4paper,12pt]{article}
\usepackage{amsmath,amssymb}
\usepackage{graphicx}
\usepackage{hyperref}

\title{Backstepping Control of Drone}

\begin{document}
\maketitle

\section{Introduction}
Drones are widely used for monitoring, rescue, supervision, and as aerial base stations. In the military, they reduce human losses and provide precise monitoring and strikes. Traditional PID controllers are commonly used but lack robustness for the nonlinear quadrotor system. Backstepping control, based on Lyapunov stability, is robust to parametric variations and ensures system stability.

\section{Quadrotor Dynamic Modeling}

\subsection{Quadrotor Configuration}
We do not take into account the gyroscopic effect of the rotors in the problem
\begin{figure}[h!]
    \centering
    \includegraphics[width=0.7\textwidth]{quadrotor_config.png}
    \caption{Quadrotor configuration (cross-configuration with four rotors).}
    \label{fig:quadrotor}
\end{figure}

\subsection{Drone Dynamic Model}
The dynamic model is defined in terms of the position vector and force expressions:

\begin{align}
    \ddot{x} &= -\frac{T}{m} [\sin\phi \sin\psi + \cos\phi \cos\theta \cos\psi] \\
    \ddot{y} &= -\frac{T}{m} [\cos\phi \sin\theta \sin\psi - \sin\psi \cos\theta] \\
    \ddot{z} &= g - \frac{T}{m} [\cos\phi \cos\theta]
\end{align}

where $(\ddot{x}, \ddot{y}, \ddot{z})$ are the accelerations, $T$ is the total thrust, $m$ is the mass, and $g$ is gravity.

The moment equations in terms of roll ($\phi$), pitch ($\theta$), and yaw ($\psi$):

\begin{align}
    p' &= \frac{I_z - I_y}{I_x} qr  + \frac{1}{I_x} \tau_\phi \\
    q' &= \frac{I_x - I_z}{I_y} pr + \frac{1}{I_y} \tau_\theta \\
    r' &= \frac{I_y - I_x}{I_z} pq + \frac{1}{I_z} \tau_\psi
\end{align}

\begin{align}
    \phi' &= p + q \sin\phi \tan\theta + r \cos\phi \tan\theta \\
    \theta' &= q \cos\phi - r \sin\phi \\
    \psi' &= \frac{q \sin\phi + r \cos\phi}{\cos\theta}
\end{align}

where $p, q, r$ are the angular velocities, $I_x, I_y, I_z$ are the moments of inertia, $J_r$ is the rotor inertia, and $\tau_\phi, \tau_\theta, \tau_\psi$ are control torques.
\newpage
\subsection{State-Space Model}
The state-space representation is:

\begin{align}
    x_1' &= x_2 \\
    x_2' &= a_1 x_4 x_6 + b_1 U_2 \\
    x_3' &= x_4 \\
    x_4' &= a_4 x_2 x_6  + b_2 U_3 \\
    x_5' &= x_6 \\
    x_6' &= a_7 x_2 x_4 + b_3 U_4 \\
    x_7' &= x_8 \\
    x_8' &= \frac{\cos x_1 \cos x_2}{m} U_1 - g \\
    x_9' &= x_{10} \\
    x_{10}' &= \frac{U_y}{U_1} m \\
    x_{11}' &= x_{12} \\
    x_{12}' &= \frac{U_x}{U_1} m
\end{align}

Parameters:
\begin{align*}
    a_1 &= \frac{I_y - I_z}{I_x}, \quad a_3 = \frac{J_r}{I_x}, \quad a_4 = \frac{I_z - I_x}{I_y}, \\
    a_6 &= \frac{J_r}{I_y}, \quad a_7 = \frac{I_x - I_y}{I_z}, \\
    b_1 &= \frac{d}{I_x}, \quad b_2 = \frac{d}{I_y}, \quad b_3 = \frac{d}{I_z}
\end{align*}

\section{Backstepping Control of Drone}

\subsection{Principle}
Backstepping divides the system into subsystems in a cascade, designing control laws for each until a global law is generated. The approach uses Lyapunov functions to ensure stability at each step.

\subsection{Roll Angle Control ($\phi$)}
Define errors:
\begin{align*}
    \varepsilon_1 &= x_1^d - x_1 \\
    V_1 &= \frac{1}{2} \varepsilon_1^2 \\
    \dot{V}_1 &= \varepsilon_1 (\dot{x}_1^d - x_2)
\end{align*}
Choose $\dot{\varepsilon}_1 = -K_1 \varepsilon_1$, so $x_2^d = \dot{x}_1^d + K_1 \varepsilon_1$.

Second error:
\begin{align*}
    \varepsilon_2 &= x_2^d - x_2 \\
    V_2 &= V_1 + \frac{1}{2} \varepsilon_2^2 \\
    \dot{V}_2 &= -K_1 \varepsilon_1^2 + \varepsilon_2(\varepsilon_1 + x_2^d - (a_1 x_4 x_6 + b_1 U_2))
\end{align*}
Choose control law:
\begin{align}
    U_2 = \frac{1}{b_1} [\varepsilon_1 - K_1 x_2 - a_1 x_4 x_6 + K_2 \varepsilon_2]
\end{align}

\subsection{Pitch and Yaw Angle Control}
Analogous steps are followed for $\theta$ and $\psi$ angles, yielding similar control laws for $U_3$ and $U_4$

## Simulation Results
![results]()
