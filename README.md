# Backstepping Control of Drone

## Introduction
Drones are widely used for monitoring, rescue, supervision, and as aerial base stations. In the military, they reduce human losses and provide precise monitoring and strikes. Traditional PID controllers are commonly used but lack robustness for the nonlinear quadrotor system. Backstepping control, based on Lyapunov stability, is robust to parametric variations and ensures system stability.

---

## Quadrotor Dynamic Modeling

### Quadrotor Configuration
![quadrotor_config (1)](https://github.com/user-attachments/assets/5adbeafc-5d8f-46b1-a386-38cfcd622d14)

## Dynamics

More exact mathematical equations are in [file](theory.pdf)

## State space

UAV is quite a complicated system with 6 degrees of freedom: 3 for positioning and 3 for orientation (roll, pitch, yaw)

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

## Control actions

From dynamics we can get 4 control actions:

$$
\mathbf{U} = 
\begin{bmatrix}
U_1 \\ 
U_2 \\ 
U_3 \\ 
U_4
\end{bmatrix}
\quad
$$

where $U_1$ - thrust and $U_2$, $U_3$, $U_4$ are torques to control orientation angles

## Desired trajectory

To build the control we need reference trajectory to track it. This trajectory can be different but we need some exact variable to wake controller work:

$$
\mathbf{X}_{\text{des}} = 
\begin{bmatrix}
\psi_d \\ 
z_d \\ 
y_d \\ 
x_d
\end{bmatrix}
\quad
$$

so we need position trajectory and yaw (for example to make photoes of the puilding we can use spiral trajectory and make yaw agle that will make drone to 'see' the axis of spiral)

## Backstepping algorithm
We compute error of real ad desired state variables separatelly. In each timestep we compute main or temporal control action and get wariables for further simulation)


## Simulation Results
![results](result.jpg)
