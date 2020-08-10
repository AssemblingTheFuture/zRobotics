# uRobot 0.1
A powerful library for robotics analysis :robot:

- [uRobot 0.1](#urobot-01)
    - [Introduction](#introduction)
    - [Features](#features)
    - [Library Content](#library-content)
      - [MATLAB](#matlab)
      - [Python](#python)

### Introduction

**uRobot** is a simple 4 DoF antropomorphic robot that can be used to analyze and develop kinematics and control algorithms as shown in our course [Robotics: from Kinematics to Control](#). We hope this library will help you to start your journey in these amazing technologies!

![uRobot](images/uRobot.jpg "uRobot 0.1")

### Features

You can control how the robot behaves; also, you'll be able to see its end - effector displacement in a 3D animation. To achieve this, all the algorithms were developed to be simulated with [Peter Corke's Robotics Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/), however, **the programming logic used in these algorithms will allow you to adapt it to almost any embedded system!**

![uRobot](images/uRobot-animation.gif "uRobot 0.1")

Feel free to modify, adjust and extend our work to your necessities :smiley:; these libraries allows you to get a first approach to robot analysis, synthesis and control, however, we will be adding new interesting features, also, **you can request new features or create new ones!**

### Library Content

This library includes the following algorithms:

#### [MATLAB](/MATLAB)

  - **[Denavit - Hartenberg Parameters](/MATLAB/denavitHartenberg.m)**

    ```matlab
        % Returns uRobot's Denavit - Hartenberg parameters as a matrix
        DH = denavitHartenberg(q, L);
    ```
    where <img src="https://render.githubusercontent.com/render/math?math=q \in \mathbb{R}^{n \times 1}"> are the generalized coordinates (set in radians) of the system. Also, <img src="https://render.githubusercontent.com/render/math?math=L \in \mathbb{R}^{p \times 1}"> is the length (set in meters) of each rigid body in the kinematic chain

<Enter>

  - **Forward Kinematics**

    - [using *Homogeneous Transformation Matrices*](/MATLAB/forwardKinematicsDH.m)
    ```matlab
        % Returns robot's forward kinematics based on «DH» matrix
        H = forwardKinematicsDH(DH, m);
    ```
    - [using *Dual Quaternions*](/MATLAB/forwardKinematicsDQ.m)
    ```matlab
        % Returns robot's forward kinematics based on «DH» matrix
        H = forwardKinematicsDQ(DH, m);
    ```
<Enter>

  - **Inverse Kinematics (*Error Feedback*)**
    - [using *Homogeneous Transformation Matrices*](/MATLAB/inverseKinematics.m)
    ```matlab
        % Returns robot's inverse kinematics using HTM
        qi = inverseKinematics(q0, L, Hd, K, m);
    ```
    - [using *Dual Quaternions*](/MATLAB/inverseKinematicsDQ.m)
    ```matlab
        % Returns robot's inverse kinematics using Dual Quaternions
        qi = inverseKinematicsDQ(q0, L, Qd, KQ, m);
    ```
    For previous cases, <img src="https://render.githubusercontent.com/render/math?math=m \in \mathbb{R}, m \geq 1"> represents the number of reference frames of the system (including inertial one). Moreover, <img src="https://render.githubusercontent.com/render/math?math=q_0 \in \mathbb{R}^{n \times 1}"> are the initial conditions of the generalized coordinates; also, <img src="https://render.githubusercontent.com/render/math?math=H_d \in \mathbb{R}^{4 \times 4}"> and <img src="https://render.githubusercontent.com/render/math?math=Q_d \in \mathbb{R}^{8 \times 1}"> represent the desired frame's pose using an Homogeneous Transformation Matrix or a Dual Quaternion respectively. Last but not least, <img src="https://render.githubusercontent.com/render/math?math=K \in \mathbb{R}^{6 \times 6}"> and <img src="https://render.githubusercontent.com/render/math?math=K_Q \in \mathbb{R}^{8 \times 8}"> are the constant symmetric gain matrices that are used to solve inverse kinematics problem

<Enter>

  - **Differential Kinematics**
    - [End - effector Velocity](/MATLAB/endEffectorVelocityDQ.m)
    ```matlab
        % Returns instantaneous end - effector (angular and linear) velocity  in dual form
        W = endEffectorVelocityDQ(q, qd, DH, m);
    ```

    - [End - effector Acceleration](/MATLAB/endEffectorAccelerationDQ.m)
    ```matlab
        % Returns instantaneous end - effector (angular and linear) acceleration in dual form
        Wd = endEffectorAccelerationDQ(q, qd, qdd, DH, m);
    ```

    - [Joints' Velocities](/MATLAB/jointsVelocitiesDQ.m)
    ```matlab
        % Returns instantaneous joints' velocities «qd»
        qd = jointsVelocitiesDQ(q, W, DH, m);
    ```

    - [Joints' Accelerations](/MATLAB/jointsAccelerationsDQ.m)
    ```matlab
        % Returns instantaneous joints' accelerations «qdd»
        qdd = jointsAccelerationsDQ(q, qd, Wd, DH, m);
    ```
    Being <img src="https://render.githubusercontent.com/render/math?math=\dot{q}, \ddot{q} \in \mathbb{R}^{n \times 1}"> (``` qd, qdd ```) the instantaneous velocity and acceleration of generalized coordinates <img src="https://render.githubusercontent.com/render/math?math=q">. Also, <img src="https://render.githubusercontent.com/render/math?math=\hat{\omega}, \dot{\hat{\omega}} \in \mathbb{R}^{8 \times 1}"> (``` W, Wd ```) are the instantaneous end - effector velocity and acceleration in dual form. **For the case of the joint's acceleration computation, do not forget to include the gravitational acceleration (in dual form) in order to get a more accurate result**

#### Python
  - Under construction :nerd_face:
