# Backstepping Control of Drone

## Introduction
Drones are widely used for monitoring, rescue, supervision, and as aerial base stations. In the military, they reduce human losses and provide precise monitoring and strikes. Traditional PID controllers are commonly used but lack robustness for the nonlinear quadrotor system. Backstepping control, based on Lyapunov stability, is robust to parametric variations and ensures system stability.

---

## Quadrotor Dynamic Modeling

### Quadrotor Configuration
![quadrotor_config (1)](https://github.com/user-attachments/assets/5adbeafc-5d8f-46b1-a386-38cfcd622d14)

## State space

UAV is quite a complicated system with 6 degrees of freedom: 3 for positioning and 3 for orientation

so we made our state space vector of 12 variables: all DoFs and their first derivatives

$$
\mathbf{X} = 
\begin{bmatrix}
\phi \\ 
p \\ 
\theta \\ 
q \\ 
\psi \\ 
r \\ 
z \\ 
\dot{z} \\ 
y \\ 
\dot{y} \\ 
x \\ 
\dot{x}
\end{bmatrix}
\quad
\text{(State Vector)}
$$

## Backstepping algorithm
We compute error of real ad desired state variables separatelly. In each timestep we compute main or temporal control action and get wariables for further simulation)


## Simulation Results
![results]()
