# zRobotics 0.1 [![Run on Repl.it](https://repl.it/badge/github/zdynamics/zRobotics)](https://repl.it/github/zdynamics/zRobotics)

A powerful library for robotics analysis :robot:

- [zRobotics 0.1 ![Run on Repl.it](https://repl.it/github/zdynamics/zRobotics)](#zrobotics-01-)
    - [Introduction](#introduction)
    - [Features](#features)
    - [Library Content](#library-content)
      - [Python](#python)
      - [MATLAB](#matlab)

### Introduction

**uRobot** is a simple 4 DoF antropomorphic robot that can be used to analyze and develop kinematics and control algorithms as shown in our course [Robotics: from Kinematics to Control](https://www.udemy.com/course/robotica-de-la-cinematica-al-control/?referralCode=5FF404FC9C3C95DE6D11). We hope this library will help you to start your journey in these amazing technologies!

![uRobot](images/uRobot.jpg "uRobot 0.1")

### Features

You can control how the robot behaves; also, you'll be able to see its end - effector displacement in a 3D animation. To achieve this, all the algorithms were developed to be simulated with [Peter Corke's Robotics Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/), however, **the programming logic used in these algorithms will allow you to adapt it to almost any embedded system!**

![uRobot](images/uRobot-animation.gif "uRobot 0.1")

Feel free to modify, adjust and extend our work to your necessities :smiley:; these libraries allows you to get a first approach to robot analysis, synthesis and control, however, we will be adding new interesting features, also, **you can request new features or create new ones!**

### Library Content

This library includes the following algorithms:

#### [Python](/Python)

- **Geometric Properties**

    ```python
        # Sets robot's generalized coordinates (two - dimensional array) and links' lengths (list)
        q = np.random.rand(4, 1)
        L = [0.3, 0.4, 0.2]
    ```

    Where <img src="https://render.githubusercontent.com/render/math?math=q \in \mathbb{R}^{n \times 1}"> are set in radians. Also, <img src="https://render.githubusercontent.com/render/math?math=L \in \mathbb{R}^{p \times 1}"> is the length (set in meters) of each rigid body in the kinematic chain

<Enter>

- **[Robot Creation](/Python/Robot.py)**

    ```python
        # Returns uRobot's as an object
        uRobot = Robot.System(jointsPositions = q, linksLengths = L, name = 'uRobot')
    ```

  - **Robot's properties**

    ```python
        # Returns uRobot's properties
        uRobot.jointsPositions  # two - dimensional array
        uRobot.linksLengths     # list
        uRobot.name             # string
    ```

<Enter>

---

- **Denavit - Hartenberg Parameters**

    - ***Robot's configuration***: first of all, it is mandatory to modify [Denavit - Hartenberg Parameters' file](/Python/DenavitHartenberg.py) with your robot's information, as you would do it in a sheet of paper (**do not forget to include inertial frame**). For example:

    |<img src="https://render.githubusercontent.com/render/math?math=\theta_z">|<img src="https://render.githubusercontent.com/render/math?math=d_z">|<img src="https://render.githubusercontent.com/render/math?math=a_x">|<img src="https://render.githubusercontent.com/render/math?math=\alpha_x">|
    |:---:|:---:|:---:|:---:|
    | 0 | 0 | 0 | 0 |
    |<img src="https://render.githubusercontent.com/render/math?math=\theta_1">|<img src="https://render.githubusercontent.com/render/math?math=L_1">|0|<img src="https://render.githubusercontent.com/render/math?math=\frac{\pi}{2}">|
    |<img src="https://render.githubusercontent.com/render/math?math=\theta_2">|0|<img src="https://render.githubusercontent.com/render/math?math=L_2">|0|
    |<img src="https://render.githubusercontent.com/render/math?math=\theta_3">|0|0|<img src="https://render.githubusercontent.com/render/math?math=\frac{\pi}{2}">|
    |<img src="https://render.githubusercontent.com/render/math?math=\theta_4">|<img src="https://render.githubusercontent.com/render/math?math=L_3">|0|0|

    Therefore,

    ```python
        import numpy as np

        def matrix(robot):
            """
            Denavit - Hartenberg parameters for n - th rigid body
            theta: rotation on «z» axis
            d: translation on «z» axis
            a: translation on «x» axis
            alpha: rotation on «x» axis
            """
            return np.array([[0, 0, 0, 0],
                            [robot.jointsPositions[0, 0], robot.linksLengths[0], 0, np.pi / 2],
                            [robot.jointsPositions[1, 0], 0, robot.linksLengths[1], 0],
                            [robot.jointsPositions[2, 0], 0, 0, np.pi / 2],
                            [robot.jointsPositions[3, 0], robot.linksLengths[2], 0, 0]])
    ```

    - *Function call*
    ```python
        # Returns uRobot's Denavit - Hartenberg parameters as a matrix
        DH = dh.matrix(uRobot)
    ```
    Where <img src="https://render.githubusercontent.com/render/math?math=DH \in \mathbb{R}^{m \times 4}"> is the Denavit - Hartenberg matrix for the kinematic chain

<Enter>

---

- **[Forward Kinematics](/Python/Kinematics.py)**

  - Using *Homogeneous Transformation Matrices*
    ```python
        # Returns robot's forward kinematics for each individual frame (framesHTM) and for end - effector (fkHTM)
        framesHTM, fkHTM = k.forwardHTM(uRobot, m = 5)
    ```

  - Using *Dual Quaternions*
    ```python
        # Returns robot's forward kinematics for each individual frame (framesDQ) and for end - effector (fkDQ)
        framesDQ, fkDQ = k.forwardDQ(uRobot, m = 5)
    ```

<Enter>

---

- **[Inverse Kinematics (*Error Feedback*)](/Python/Kinematics.py)**
  - Using *Homogeneous Transformation Matrices*
    ```python
        # Returns robot's inverse kinematics using HTM
        qHTM = k.inverseHTM(uRobot, q0 = np.random.rand(4, 1), Hd = fkHTM, K = np.eye(6), m = 5)
    ```
  - using *Dual Quaternions*
    ```python
        # Returns robot's inverse kinematics using Dual Quaternions
        qDQ = k.inverseDQ(uRobot, q0 = np.random.rand(4, 1), Qd = fkDQ, K = np.eye(8), xi = xi, m = 5)
    ```
    For previous cases, <img src="https://render.githubusercontent.com/render/math?math=m \in \mathbb{R}, m \geq 1"> represents the number of reference frames of the system (including inertial one). Moreover, <img src="https://render.githubusercontent.com/render/math?math=q_0 \in \mathbb{R}^{n \times 1}"> are the initial conditions of the generalized coordinates; also, <img src="https://render.githubusercontent.com/render/math?math=H_d \in \mathbb{R}^{4 \times 4}"> and <img src="https://render.githubusercontent.com/render/math?math=Q_d \in \mathbb{R}^{8 \times 1}"> represent the desired frame's pose using an Homogeneous Transformation Matrix or a Dual Quaternion respectively. Last but not least, <img src="https://render.githubusercontent.com/render/math?math=K \in \mathbb{R}^{6 \times 6}"> and <img src="https://render.githubusercontent.com/render/math?math=K_Q \in \mathbb{R}^{8 \times 8}"> are the constant symmetric gain matrices that are used to solve inverse kinematics problem

<Enter>

**IMPORTANT NOTE:** Inverse kinematics algorithms returns a generalized coordinates vector <img src="https://render.githubusercontent.com/render/math?math=q \in \mathbb{R}^{n \times p}">, where <img src="https://render.githubusercontent.com/render/math?math=p \in \mathbb{R}, p \geq 1"> is the number of joints' positions that have to be reached

<Enter>

---

- **[Robot Animation](/Python/Plot.py)**

    ```python
        # Plot robot with new joints' positions (this also modifies them in the object)
        plot.animation(uRobot, q = qHTM, repeatAnimation = False, delayPerFrame = 1)
    ```
    
![uRobot](images/uRobotPython.gif "uRobot 0.1")
    
  - Under construction... :nerd_face:

<Enter>

---

#### [MATLAB](/MATLAB)

  - **[Denavit - Hartenberg Parameters](/MATLAB/denavitHartenberg.m)**

    ```matlab
        % Returns uRobot's Denavit - Hartenberg parameters as a matrix
        DH = denavitHartenberg(q, L);
    ```
    Where <img src="https://render.githubusercontent.com/render/math?math=q \in \mathbb{R}^{n \times 1}"> are the generalized coordinates (set in radians) of the system. Also, <img src="https://render.githubusercontent.com/render/math?math=L \in \mathbb{R}^{p \times 1}"> is the length (set in meters) of each rigid body in the kinematic chain

<Enter>

---

  - **Forward Kinematics**

    - [using *Homogeneous Transformation Matrices*](/MATLAB/forwardKinematicsDH.m)
    ```matlab
        % Returns robot's forward kinematics based on «DH» matrix
        H = forwardKinematicsDH(DH, m);
    ```
    - [using *Dual Quaternions*](/MATLAB/forwardKinematicsDQ.m)
    ```matlab
        % Returns robot's forward kinematics based on «DH» matrix
        DQ = forwardKinematicsDQ(DH, m);
    ```
<Enter>

---

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

---

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

<Enter>

---

  - **Dynamic System Solver and Simulation**
    - [Numerical Solver](/MATLAB/solver.m)
    ```matlab
        % Returns ODE solution using fourth order Runge - Kuta algorithm
        F = solver(f, F, h);
    ```

    - [Simulator](/MATLAB/dynamicSystem.m)
    ```matlab
        % Returns Dynamic System's simulation results
        [e, q, u, V] = dynamicSystem(q0, qd, K, t);
    ```

    Where <img src="https://render.githubusercontent.com/render/math?math=f, F \in \mathbb{R}^{n \times r}"> represent the differential equation and its **previous solution** respectively; on the other hand, <img src="https://render.githubusercontent.com/render/math?math=h \in \mathbb{R}, h > 0"> is the step that solver needs to compute ODE solution, so the return argument <img src="https://render.githubusercontent.com/render/math?math=F"> will be the **current solution**
    
    Furthermore, <img src="https://render.githubusercontent.com/render/math?math=\mathrm{K} \in \mathbb{R}^{n \times n}"> is a positive definite constant matrix used by the control function <img src="https://render.githubusercontent.com/render/math?math=\mathrm{v}\left(t\right) = \mathrm{K} \mathrm{e}\left(t\right), \mathrm{v} \in \mathbb{R}^{n \times n}">, where <img src="https://render.githubusercontent.com/render/math?math=\mathrm{e} \in \mathbb{R}^{n \times 1}"> represents the error between the desired joints position <img src="https://render.githubusercontent.com/render/math?math=q_{d}"> and the current one <img src="https://render.githubusercontent.com/render/math?math=q">. This control function leads the joints to the desire position, meanwhile <img src="https://render.githubusercontent.com/render/math?math=\mathrm{u} \in \mathbb{R}^{n \times 1}"> deals with the uncertainty that can be generated by disturbances, measurement noise, etc.
    
    **If you want to know more about these control functions and how you can implement them in your projects, check out our course [Control of Dynamic Systems](https://www.udemy.com/course/control-de-sistemas-dinamicos/?referralCode=74300CF3F21F98714329)** :wink:

<Enter>

---

- **Robot Creation and Animation (using [Peter Corke's Robotics Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/))**
    
    - Create robot's joints positions
    
    ```matlab
        % Returns uRobot's generalized coordinates vector
        q = [q1
             q2
             q3
             q4];
    ```

    - Create robot's links or rigid bodies length
    
    ```matlab
        % Returns uRobot's rigid bodies length
        L = [l1 l2 l3];
    ```

    - Create robot's links as **objects**
    
    ```matlab
        % Returns uRobot's rigid bodies as objects
        L1 = Link('d', L(1), 'a', 0, 'alpha', pi/2);
        L2 = Link('d', 0, 'a', L(2), 'alpha', 0);
        L3 = Link('d', 0, 'a', 0, 'alpha', pi/2);
        L4 = Link('d', L(3), 'a', 0, 'alpha', 0);
    ```

    - Create kinematic chain as an **object**
    
    ```matlab
        % Returns uRobot
        uRobot = SerialLink([L1, L2, L3, L4], 'name', 'uRobot');
    ```

    - Plot **uRobot**
    
    ```matlab
        % Returns uRobot plot
        uRobot.plot(q')
    ```

    **IMPORTANT NOTE:** Inverse kinematics algorithms returns a generalized coordinates vector <img src="https://render.githubusercontent.com/render/math?math=q \in \mathbb{R}^{n \times p}">, where <img src="https://render.githubusercontent.com/render/math?math=p \in \mathbb{R}, p \geq 1"> is the number of joints' positions that have to be reached. In order to use ```uRobot.plot( )```, we have to transpose <img src="https://render.githubusercontent.com/render/math?math=q">, otherwise, we won't be able to see the robot's animation

---

**We hope this can be useful for you. Thank you!**

<Enter>
    
![Z Dynamics](images/icon.png "The Future is ROBOTICS")
