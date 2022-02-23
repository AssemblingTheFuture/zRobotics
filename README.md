# zRobotics 0.1

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) [![License: CC BY-NC 4.0](https://img.shields.io/badge/License-CC%20BY--NC%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc/4.0/)

A powerful library for robotics analysis :mechanical_arm: :robot:

- Index :card_index_dividers:
    - [Introduction](#introduction-books) :books:
    - [Features](#features-sparkles) :sparkles:
    - [Future Work](#future-work-cloud) :cloud:
    - [Before Starting](#before-starting-warning) :warning:
    - [Library Content](#library-content-book) :book:
      - [How to import it?](#how-to-import-it-man_technologist) :man_technologist:
      - [(Brief) Libraries' Description](#brief-libraries-description-blue_book) :blue_book:
    - [Movements](#movements)
      - [Translation](#translation)
      - [Rotation](#rotation)
    - [Robot Creation and Setup](#robot-creation-and-setup-mechanical_leg) :mechanical_leg:
      - [Geometric Properties](#geometric-properties)
      - [Creation](#creation)
      - [Denavit - Hartenberg Parameters](#denavit---hartenberg-parameters)
      - [Centers of Mass](#centers-of-mass)
    - [Kinematics](#kinematics)
      - [Forward Kinematics](#forward-kinematics)
      - [Forward Kinematics to Center of Mass](#forward-kinematics-to-centers-of-mass)
      - [Axis - Angle Vector](#axis---angle-vector)
      - [Jacobian Matrix](#jacobian-matrix)
      - [Inverse Kinematics (Error Feedback)](#inverse-kinematics-error-feedback)

---

## Introduction :books:

You can use this simple library to analyze and develop kinematics and control algorithms for your robots as shown in our online trainigs:

- [Numerical Methods with Engineering Applications](https://bit.ly/NumericZ)
- [Control of Dynamical Systems](https://bit.ly/zControl) 
- [Robotics: from Kinematics to Control](https://bit.ly/RoboticZ)
 
We hope this library will help you to start your journey in these amazing discipline!

[*Return to top*](#zrobotics-01)

---

## Features :sparkles:

You can set your robot attributes and analyze its behavior; also, you'll be able to see its end - effector displacement in a 3D animation (*comming soon*). To achieve this, all the algorithms were developed using Homogeneous Transformation Matrices and Dual Quaternions algebra, however, **the logic used to develop them will allow you to adapt it to almost any embedded system!**

![Animation](img/uRobotPython.gif "Robot Animation")

Feel free to modify, adjust and extend our work to your necessities :smiley:; this library allows you to get a first approach to robot analysis, synthesis and control, however, we will be adding new interesting features, also, **you can request new features or create new ones!**

[*Return to top*](#zrobotics-01)

---

## Future Work :cloud: 

We are working, or will start working soon, on the following tasks for future releases:

- [ ] Velocity Analysis using Homogeneous Transformation Matrices
- [ ] Acceleration Analysis using Homogeneous Transformation Matrices
- [ ] Euler - Lagrange formulation using Homogeneous Transformation Matrices
- [ ] Newton - Euler Recursive Algorithm using Homogeneous Transformation Matrices

[*Return to top*](#zrobotics-01)

---

## Before Starting :warning:

This library can be used with [Python :snake: 3.8.10 - 64 bits](https://www.python.org/downloads/release/python-3810/) or superior. Please install the following libraries or make sure that they're already in your computer:

```bash
pip3 install numpy
pip3 install sympy
pip3 install matplotlib
```

**Please do not forget to set your Python Path** :wink:. Also, you can use ```pip``` if you only have Python installed:

```bash
pip install numpy
pip install sympy
pip install matplotlib
```

If this modules were installed correctly, you will be able to use our library :smiley:

[*Return to top*](#zrobotics-01)

---

## Library Content :book:

This library includes the following files:

```
zRobotics
├── img                 # Images for README.md file and related ones
├── MATLAB              # MATLAB files (DEPRECATED)
├── lib                 # Python source files
|   ├── Robot.py        # Robot constructor
|   ├── movements
|   |   ├── DQ.py       # Rotations and translations using Dual Quaternios
|   |   └── HTM.py      # Rotations and translations using Homogeneous Transformation Matrices
|   ├── kinematics
|   |   ├── DQ.py                   # Forward and Inverse Kinematics functions using Dual Quaternios
|   |   ├── HTM.py                  # Forward and Inverse Kinematics functions using Homogeneous Transformation Matrices
|   |   ├── DifferentialDQ.py       # Velocities and accelerations using Dual Quaternions (UNDER DEVELOPMENT)
|   |   └── DifferentialHTM.py      # Velocities and accelerations using Homogeneous Transformation Matrices (UNDER DEVELOPMENT)
|   ├── dynamics
|   |   └── Solver.py   # Numerical solvers for multiple purposes (UNDER DEVELOPMENT)
|   ├── plot
|   └── └── Plot.py     # For function plotting (UNDER DEVELOPMENT)
├── CODE_OF_CONDUCT.md
├── README.md           # Users' manual
├── LICENSE
└── main.py             # File to demonstrate functionalities
```

New functionalities can be added in future releases, so feel free to send an email to [**contact@zdynamics.org**](mailto:contact@zdynamics.org) requesting anything new :smiley:

[*Return to top*](#zrobotics-01)

---

### How to import it? :man_technologist:

Just open your terminal (depending on your operating system) inside ```zRobotics``` directory, then run Python and type ```from lib.NameOfLibrary.NameOfFile import *```, for example

```python
from lib.movements.HTM import *
```

So this will import the [```movements```](lib/movements/) library that uses [Homogeneous Transformations Matrices](lib/movements/HTM.py). If you have a Python project in another directory, just copy the ```lib``` folder and paste it into your project's directory:

```
your_project_root_directory
├── folder_1
|   ├──image.jpg
|   └── file0.c
├── folder_2
|   ├── test.ino
|   └── file1.py
├── lib                 # zRobotics Python source files
└── file2.py
```

Then, you will be able to use all our files from your root directory :wink:

[*Return to top*](#zrobotics-01)

---

### (Brief) Libraries' Description :blue_book:

1. [```lib.Robot```](/lib/Robot.py): to facilitate algorithms implementation and usage, this creates your robot as an object :robot:, so what you have to do is to create its Denavit - Hartenberg Parameters Matrix. :warning: **It only allows to create serial manipulators** :warning:, but some other robots will be added soon
2. [```lib.movements```](/lib/movements): it can be used to compute the translational and rotational movements representation using Homogeneous Transformation Matrices or Dual Quaternions
4. [```lib.kinematics```](/lib/kinematics): based on Dual Quaternions algebra and Homogeneous Transformation Matrices, it solves the most common problem of kinematics, from forward kinematics to differential one (**under construction :construction:**)
5. [ ```lib.dynamics```](/lib/dynamics): it has numerical integration and dynamical systems algorithms (**under construction :construction:**)
6. [```lib.plot```](/lib/plot): this allows to plot graphs of your robot's behavior


Please take a look at [main.py](main.py) to know more about this implementation. **Feel free to  [contact us](mailto:contact@zdynamics.org) if you have any comment, suggestion or question** :smile:

[*Return to top*](#zrobotics-01)

---

## [Movements](/lib/movements/HTM.py)

You can compute translational or rotational movements with Homogeneous Transformation Matrices and Dual Quaternions

### Translation

You can compute translational movements on each axis of euclidian space: (x, y, z)

```python
"""
  Translational movement on "Y" axis
"""

# Homogeneous Transformation Matrix library
from lib.movements.HTM import *

# Dual Quaternions library
from lib.movements.DQ import *

# Using Homogeneous Transformation Matrix
H = ty(y = 0.5)

# Using Dual Quaternions
Q = dqTy(y = 0.5)
```

So the outputs will be

```bash
# NumPy Array
>>> H
array([[1. , 0. , 0. , 0. ],
       [0. , 1. , 0. , 0.5],
       [0. , 0. , 1. , 0. ],
       [0. , 0. , 0. , 1. ]])

# NumPy Array
>>> Q
array([[1.  ],
       [0.  ],
       [0.  ],
       [0.  ],
       [0.  ],
       [0.  ],
       [0.25],
       [0.  ]])
```

In case you need a symbolic expression, it's possible to call the same functions with a ```symbolic``` parameter:

```python
"""
  Translational movement on "Z" axis
"""

# Homogeneous Transformation Matrix library
from lib.movements.HTM import *

# Dual Quaternions library
from lib.movements.DQ import *

# Using Homogeneous Transformation Matrix
H = tz(z = 0.5, symbolic = True)

# Using Dual Quaternions
Q = dqTz(z = 0.5, symbolic = True)
```

Then the outputs will be shown as follows:

```bash
# SymPy Matrix
>>> H
Matrix([[1, 0, 0,   0],
        [0, 1, 0,   0],
        [0, 0, 1, 0.5],
        [0, 0, 0,   1]])

# SymPy Matrix
>>> Q
Matrix([[   1],
        [   0],
        [   0],
        [   0],
        [   0],
        [   0],
        [   0],
        [0.25]])
```

The functions for translational movements (with their default values) are shown below:

```python
"""
  Translational movements
"""

# Homogeneous Transformation Matrix library
from lib.movements.HTM import *

# Using Homogeneous Transformation Matrix
Hx = tx(x = 0, symbolic = False)
Hy = ty(y = 0, symbolic = False)
Hz = tz(z = 0, symbolic = False)


# Dual Quaternions library
from lib.movements.DQ import *

# Using Dual Quaternions
Qx = dqTx(x = 0.5, symbolic = True)
Qy = dqTy(y = 0.5, symbolic = True)
Qz = dqTz(z = 0.5, symbolic = True)
```

[*Return to top*](#zrobotics-01)

---

## Rotation

You can compute rotational movements on each axis of euclidian space: (x, y, z)

```python
"""
  Rotational movement on "Y" axis
"""

# Homogeneous Transformation Matrix library
from lib.movements.HTM import *

# Dual Quaternions library
from lib.movements.DQ import *

# Using Homogeneous Transformation Matrix
H = ry(y = 0.5)

# Using Dual Quaternions
Q = dqRy(y = 0.5)
```

So the outputs will be

```bash
# NumPy Array
>>> H
array([[ 0.87758256,  0.        ,  0.47942554,  0.        ],
       [ 0.        ,  1.        ,  0.        ,  0.        ],
       [-0.47942554,  0.        ,  0.87758256,  0.        ],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])


# NumPy Array
>>> Q
array([[0.96891242],
       [0.        ],
       [0.24740396],
       [0.        ],
       [0.        ],
       [0.        ],
       [0.        ],
       [0.        ]])
```

In case you need a symbolic expression, it's possible to call the same functions with a ```symbolic``` parameter:

```python
"""
  Rotational movement on "Z" axis
"""

# Homogeneous Transformation Matrix library
from lib.movements.HTM import *

# Dual Quaternions library
from lib.movements.DQ import *

# Using Homogeneous Transformation Matrix
H = rz(z = 0.5, symbolic = True)

# Using Dual Quaternions
Q = dqRz(z = 0.5, symbolic = True)
```

Then the outputs will be shown as follows:

```bash
# SymPy Matrix
>>> H
Matrix([
       [0.877582561890373, -0.479425538604203, 0, 0],
       [0.479425538604203,  0.877582561890373, 0, 0],
       [                0,                  0, 1, 0],
       [                0,                  0, 0, 1]])

# SymPy Matrix
>>> Q
Matrix([[0.968912421710645],
        [                0],
        [                0],
        [0.247403959254523],
        [                0],
        [                0],
        [                0],
        [                0]])
```

The functions for rotational movements (with their default values) are shown below:

```python
"""
  Rotational movements
"""

# Homogeneous Transformation Matrix library
from lib.movements.HTM import *

# Using Homogeneous Transformation Matrix
Hx = rx(x = 0, symbolic = False)
Hy = ry(y = 0, symbolic = False)
Hz = rz(z = 0, symbolic = False)


# Dual Quaternions library
from lib.movements.DQ import *

# Using Dual Quaternions
Qx = dqRx(x = 0.5, symbolic = True)
Qy = dqRy(y = 0.5, symbolic = True)
Qz = dqRz(z = 0.5, symbolic = True)
```

[*Return to top*](#zrobotics-01)

---

## Robot Creation and Setup :mechanical_leg:

A robot can be created as an object, but upt to this day, this library only works with serial manipulators, but we will add other type of robots soon :wink: but before creating the robot, it is necessary to set some attributes

### Geometric Properties

```python
# NumPy library is necessary for joints positions
import numpy as np

# Sets robot's generalized coordinates
q = np.random.rand(4, 1)

# Set links' lengths
L = [0.3, 0.4, 0.2]

# Set centers of mass in each rigid body
Lcom = [0.15, 0.25, 0.2]
```

Where <img src="https://render.githubusercontent.com/render/math?math=q \in \mathbb{R}^{n \times 1}"> are set in radians. Also, <img src="https://render.githubusercontent.com/render/math?math=L, L_{com} \in \mathbb{R}^{p \times 1}"> is the length (set in meters) of each rigid body in the kinematic chain and its centers of mass

[*Return to top*](#zrobotics-01)

---

### [Creation](/lib/Robot.py)

It is really simple to create a robot object:

```python
"""
  Create a robot as an object
"""

# Robot's library
from lib.Robot import *

# Returns robot as an object
uRobot = Serial(jointsPositions = q, linksLengths = L, COMs = Lcom, name = 'uRobot')
```

After this, it is possible to access to its attributes as follows:

```bash
# NumPy Array
>>> uRobot.jointsPositions
array([[0.46727453],
       [0.54455365],
       [0.4350795 ],
       [0.28259213]])

# SymPy Matrix
>>> uRobot.qSymbolic
Matrix([[q1],
        [q2],
        [q3],
        [q4]])

# List
>>> uRobot.linksLengths
[0.3, 0.4, 0.2]

# SymPy Matrix
>>> uRobot.symbolicLinks
Matrix([[L1],
        [L2],
        [L3]])

# List
>>> uRobot.COMs
[0.3, 0.4, 0.2]

# SymPy Matrix
>>> uRobot.symbolicCOMs
Matrix([[Lcom1],
        [Lcom2],
        [Lcom3]])
```

You can add new attributes if they are necessary for your project :smiley:

[*Return to top*](#zrobotics-01)

---

### Denavit - Hartenberg Parameters

Any serial manipulator is built by a Denavit - Hartenberg Parameters Matrix as this

|<img src="https://render.githubusercontent.com/render/math?math=\theta_z">|<img src="https://render.githubusercontent.com/render/math?math=d_z">|<img src="https://render.githubusercontent.com/render/math?math=a_x">|<img src="https://render.githubusercontent.com/render/math?math=\alpha_x">|
|:---:|:---:|:---:|:---:|
| 0 | 0 | 0 | 0 |
|<img src="https://render.githubusercontent.com/render/math?math=q_1">|<img src="https://render.githubusercontent.com/render/math?math=L_1">|0|<img src="https://render.githubusercontent.com/render/math?math=\frac{\pi}{2}">|
|<img src="https://render.githubusercontent.com/render/math?math=q_2">|0|<img src="https://render.githubusercontent.com/render/math?math=L_2">|0|
|<img src="https://render.githubusercontent.com/render/math?math=q_3">|0|0|<img src="https://render.githubusercontent.com/render/math?math=\frac{\pi}{2}">|
|<img src="https://render.githubusercontent.com/render/math?math=q_4">|<img src="https://render.githubusercontent.com/render/math?math=L_3">|0|0|

so is mandatory to modify [Robot's file](/lib/Robot.py#L54) with your robot's information, as you would do it in a sheet of paper (**do not forget to include inertial frame**):

```python
def denavitHartenberg(self, symbolic = False):
  """Denavit - Hartenberg parameters for n - th rigid body

    theta: rotation on «z» axis
      d: translation on «z» axis
      a: translation on «x» axis
      alpha: rotation on «x» axis
  """ 

  self.dhParameters = Matrix([[0, 0, 0, 0],
                              [self.qSymbolic[0, 0], self.symbolicLinks[0], 0.0000000000000000000, np.pi / 2],
                              [self.qSymbolic[1, 0], 0.0000000000000000000, self.symbolicLinks[1], 0.0000000],
                              [self.qSymbolic[2, 0], 0.0000000000000000000, 0.0000000000000000000, np.pi / 2],
                              [self.qSymbolic[3, 0], self.symbolicLinks[2], 0.0000000000000000000, 0.0000000]]) if symbolic else np.array([[0, 0, 0, 0],
                                                                                                                                           [self.jointsPositions[0, 0], self.linksLengths[0], 0.000000000000000000, np.pi / 2],
                                                                                                                                           [self.jointsPositions[1, 0], 0.000000000000000000, self.linksLengths[1], 0.0000000],
                                                                                                                                           [self.jointsPositions[2, 0], 0.000000000000000000, 0.000000000000000000, np.pi / 2],
                                                                                                                                           [self.jointsPositions[3, 0], self.linksLengths[2], 0.000000000000000000, 0.0000000]])
```

It is not necessary to call this function before performing any kinematics task, this is because all the algorithms will access to these methods automatically

[*Return to top*](#zrobotics-01)

---

### Centers of Mass

Some calculations in robotics needs to be performed with respect to the Center of Mass, so it is mandatory to modify [Robot's file](/lib/Robot.py#L73) with your robot's information, as you would do it in a sheet of paper (**do not forget to include inertial frame**). For example:

|<img src="https://render.githubusercontent.com/render/math?math=\theta_z">|<img src="https://render.githubusercontent.com/render/math?math=d_z">|<img src="https://render.githubusercontent.com/render/math?math=a_x">|<img src="https://render.githubusercontent.com/render/math?math=\alpha_x">|
|:---:|:---:|:---:|:---:|
| 0 | 0 | 0 | 0 |
|<img src="https://render.githubusercontent.com/render/math?math=\theta_1">|<img src="https://render.githubusercontent.com/render/math?math=L_{com_{1}}">|0|<img src="https://render.githubusercontent.com/render/math?math=\frac{\pi}{2}">|
|<img src="https://render.githubusercontent.com/render/math?math=\theta_2">|0|<img src="https://render.githubusercontent.com/render/math?math=L_{com_{2}}">|0|
|<img src="https://render.githubusercontent.com/render/math?math=\theta_3">|0|0|<img src="https://render.githubusercontent.com/render/math?math=\frac{\pi}{2}">|
|<img src="https://render.githubusercontent.com/render/math?math=\theta_4">|<img src="https://render.githubusercontent.com/render/math?math=L_{com_{3}}">|0|0|

Therefore,

```python
def denavitHartenbergCOM(self, symbolic = False):
  """Denavit - Hartenberg parameters for n - th center of mass

    theta: rotation on «z» axis
    d: translation on «z» axis
    a: translation on «x» axis
    alpha: rotation on «x» axis
  """ 
    
  self.dhParametersCOM = Matrix([[0, 0, 0, 0],
                                 [self.qSymbolic[0, 0], self.symbolicCOMs[0], 0.000000000000000000, np.pi / 2],
                                 [self.qSymbolic[1, 0], 0.000000000000000000, self.symbolicCOMs[1], 0.0000000],
                                 [self.qSymbolic[2, 0], 0.000000000000000000, 0.000000000000000000, np.pi / 2],
                                 [self.qSymbolic[3, 0], self.symbolicCOMs[2], 0.000000000000000000, 0.0000000]]) if symbolic else np.array([[0, 0, 0, 0],
                                                                                                                                            [self.jointsPositions[0, 0], self.COMs[0], 0.0000000000, np.pi / 2],
                                                                                                                                            [self.jointsPositions[1, 0], 0.0000000000, self.COMs[1], 0.0000000],
                                                                                                                                            [self.jointsPositions[2, 0], 0.0000000000, 0.0000000000, np.pi / 2],
                                                                                                                                            [self.jointsPositions[3, 0], self.COMs[2], 0.0000000000, 0.0000000]])
```

As it happens with conventional Denavit - Hartenbger Parameters, it is not necessary to compute this matrix for centers of mass before executing any kinematics task. **For future releases, we will work on a simpler way to create this matrices** :wink:

[*Return to top*](#zrobotics-01)

---

## [Kinematics](/lib/kinematics/)

In order to know the position of each reference frame in the Denavit - Hartenberg Parameters matrix, we can use the following functions

### Forward Kinematics

```python
"""
  Calculate forward kinematics
"""

# Kinematics libraries
from lib.kinematics.HTM import *
from lib.kinematics.DQ import *

# Robot's forward kinematics using Homogeneous Transformation Matrices (numerical and symbolic)
fkHTM = forwardHTM(uRobot)
symbolicFKHTM = forwardHTM(uRobot, symbolic = True)

# Robot's forward kinematics using Dual Quaternions (numerical and symbolic)
fkDQ = forwardDQ(uRobot)
symbolicFKDQ = forwardDQ(uRobot, symbolic = True)
```

Where <img src="https://render.githubusercontent.com/render/math?math=fk_{HTM} \in \mathbb{R}^{4 \times 4}"> and <img src="https://render.githubusercontent.com/render/math?math=fk_{DQ} \in \mathbb{H}"> are lists that store the pose representation for each reference frame with Homogeneous Transformation Matrices or Dual Quaternions. You can get all the elements of the list, but also you can access to each specific pose representation by indexing it:

```bash
# NumPy Array
>>> fkHTM[-1]
array([[ 0.60345261,  0.29383926,  0.74128499,  0.45372242],
       [-0.0078605 , -0.92739162,  0.37400935,  0.22892198],
       [ 0.79736012, -0.23152379, -0.55732717,  0.39574911],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

# SymPy Matrix
>>> symbolicFKHTM[-1]
Matrix([[sin(q1)*sin(q4) + cos(q1)*cos(q4)*cos(q2 + q3),  sin(q1)*cos(q4) - sin(q4)*cos(q1)*cos(q2 + q3), sin(q2 + q3)*cos(q1), (L2*cos(q2) + L3*sin(q2 + q3))*cos(q1)],
        [sin(q1)*cos(q4)*cos(q2 + q3) - sin(q4)*cos(q1), -sin(q1)*sin(q4)*cos(q2 + q3) - cos(q1)*cos(q4), sin(q1)*sin(q2 + q3), (L2*cos(q2) + L3*sin(q2 + q3))*sin(q1)],
        [                          sin(q2 + q3)*cos(q4),                           -sin(q4)*sin(q2 + q3),        -cos(q2 + q3),      L1 + L2*sin(q2) - L3*cos(q2 + q3)],
        [                                             0,                                               0,                    0,                                      1]])

# NumPy Array
>>> fkDQ[-1]
array([[-0.17228887],
       [ 0.87865969],
       [ 0.0813679 ],
       [ 0.43778186],
       [-0.29527314],
       [-0.00507735],
       [ 0.05482832],
       [-0.11620462]])

# SymPy Matrix
>>> symbolicFKDQ[-1]
Matrix([[                                                   -sin(q1/2 + q4/2)*sin(q2/2 + q3/2)],
        [                                                    cos(q1/2 - q4/2)*cos(q2/2 + q3/2)],
        [                                                    sin(q1/2 - q4/2)*cos(q2/2 + q3/2)],
        [                                                    sin(q2/2 + q3/2)*cos(q1/2 + q4/2)],
        [-(L1*sin(q2/2 + q3/2) + L2*cos(q2/2 - q3/2) + L3*sin(q2/2 + q3/2))*cos(q1/2 + q4/2)/2],
        [(-L1*cos(q2/2 + q3/2) - L2*sin(q2/2 - q3/2) + L3*cos(q2/2 + q3/2))*sin(q1/2 - q4/2)/2],
        [ (L1*cos(q2/2 + q3/2) + L2*sin(q2/2 - q3/2) - L3*cos(q2/2 + q3/2))*cos(q1/2 - q4/2)/2],
        [-(L1*sin(q2/2 + q3/2) + L2*cos(q2/2 - q3/2) + L3*sin(q2/2 + q3/2))*sin(q1/2 + q4/2)/2]])
```

**IMPORTANT NOTE:** Please notice that symbolic computation is slower than numerical one, so use those commands only if you need to know the equations of motion of your system :wink:

[*Return to top*](#zrobotics-01)

---

### Forward Kinematics to Centers of Mass

```python
"""
  Calculate forward kinematics to each center of mass
"""

# Kinematics libraries
from lib.kinematics.HTM import *
from lib.kinematics.DQ import *

# Robot's forward kinematics to each center of mass using Homogeneous Transformation Matrices (numerical and symbolic)
fkCOMHTM = forwardCOMHTM(uRobot)
symbolicFKCOMHTM = forwardCOMHTM(uRobot, symbolic = True)
  
# Robot's forward kinematics to each center of mass using Dual Quaternions (numerical and symbolic)
fkCOMDQ = forwardCOMDQ(uRobot)
symbolicDQCOMHTM = forwardCOMDQ(uRobot, symbolic = True)
```

Where <img src="https://render.githubusercontent.com/render/math?math=fkCOM_{HTM} \in \mathbb{R}^{4 \times 4}"> and <img src="https://render.githubusercontent.com/render/math?math=fkCOM_{DQ} \in \mathbb{H}"> are lists that store the pose representation for each center of mass with Homogeneous Transformation Matrices or Dual Quaternions. You can get all the elements of the list, but also you can access to each specific pose representation by indexing it:

```bash
# NumPy Array
>>> fkCOMHTM[-1]
array([[ 6.03452611e-01,  2.93839264e-01,  7.41284988e-01,  4.53722422e-01],
       [-7.86050467e-03, -9.27391622e-01,  3.74009346e-01,  2.28921979e-01],
       [ 7.97360119e-01, -2.31523791e-01, -5.57327171e-01,  3.95749108e-01],
       [-1.99892919e-16, -6.93639930e-18,  1.57048749e-17,  1.00000000e+00]])

# SymPy Matrix
>>> symbolicFKCOMHTM[-1]
Matrix([[sin(q1)*sin(q4) + cos(q1)*cos(q4)*cos(q2 + q3),  sin(q1)*cos(q4) - sin(q4)*cos(q1)*cos(q2 + q3), sin(q2 + q3)*cos(q1), (L2*cos(q2) + Lcom3*sin(q2 + q3))*cos(q1)],
        [sin(q1)*cos(q4)*cos(q2 + q3) - sin(q4)*cos(q1), -sin(q1)*sin(q4)*cos(q2 + q3) - cos(q1)*cos(q4), sin(q1)*sin(q2 + q3), (L2*cos(q2) + Lcom3*sin(q2 + q3))*sin(q1)],
        [                          sin(q2 + q3)*cos(q4),                           -sin(q4)*sin(q2 + q3),        -cos(q2 + q3),      L1 + L2*sin(q2) - Lcom3*cos(q2 + q3)],
        [                                             0,                                               0,                    0,                                         1]])

# NumPy Array
>>> fkCOMDQ[-1]
array([[-0.17228887],
       [ 0.87865969],
       [ 0.0813679 ],
       [ 0.43778186],
       [-0.29527314],
       [-0.00507735],
       [ 0.05482832],
       [-0.11620462]])

# SymPy Matrix
>>> symbolicDQCOMHTM[-1]
Matrix([[                                                      -sin(q1/2 + q4/2)*sin(q2/2 + q3/2)],
        [                                                       cos(q1/2 - q4/2)*cos(q2/2 + q3/2)],
        [                                                       sin(q1/2 - q4/2)*cos(q2/2 + q3/2)],
        [                                                       sin(q2/2 + q3/2)*cos(q1/2 + q4/2)],
        [-(L1*sin(q2/2 + q3/2) + L2*cos(q2/2 - q3/2) + Lcom3*sin(q2/2 + q3/2))*cos(q1/2 + q4/2)/2],
        [(-L1*cos(q2/2 + q3/2) - L2*sin(q2/2 - q3/2) + Lcom3*cos(q2/2 + q3/2))*sin(q1/2 - q4/2)/2],
        [ (L1*cos(q2/2 + q3/2) + L2*sin(q2/2 - q3/2) - Lcom3*cos(q2/2 + q3/2))*cos(q1/2 - q4/2)/2],
        [-(L1*sin(q2/2 + q3/2) + L2*cos(q2/2 - q3/2) + Lcom3*sin(q2/2 + q3/2))*sin(q1/2 + q4/2)/2]])
```

In this case, <img src="https://render.githubusercontent.com/render/math?math=H_{com_{i}/0}^{0} \in \mathbb{R}^{4 \times 4}"> and <img src="https://render.githubusercontent.com/render/math?math=Q_{com_{i}/0}^{0} \in \mathbb{H}"> are defined as <img src="https://render.githubusercontent.com/render/math?math=H_{com_{i}/0}^{0} = H_{i/0}^{0} (H_{i/i-1}^{i - 1})^{-1} H_{com_{i}/i-1}^{i - 1}"> and <img src="https://render.githubusercontent.com/render/math?math=Q_{com_{i}/0}^{0} = Q_{i/0}^{0} (Q_{i/i-1}^{i - 1})^{*} Q_{com_{i}/i-1}^{i - 1}"> respectively


**IMPORTANT NOTE:** Please notice that symbolic computation is slower than numerical one, so use those commands only if you need to know the equations of motion of your system :wink:

[*Return to top*](#zrobotics-01)

---

### Axis - Angle Vector

A compact representation of an Homogeneous Transformation Matrix can be obtained by and Axis - Angle Vector. This is OPTIONAL, because each function can call it automatically if needed:

```python
# Kinematics libraries
from lib.kinematics.HTM import *

# Axis - Angle vector based on Homogeneous Transformation Matrix obtained by Forward Kinematics calculation
X = axisAngle(fkHTM[-1])
symbolicX = axisAngle(symbolicFKHTM[-1], symbolic = True)
```

So the output will be:

```bash
# NumPy Array
>>> X
array([[ 0.48684051],
       [ 0.07557617],
       [ 0.19911015],
       [-2.55307359],
       [ 1.13388496],
       [-0.6245545 ]])

# SymPy Matrix
>>> symbolicX
Matrix([[                                                                                                                                                (L2*cos(q2) + L3*sin(q2 + q3))*cos(q1)],
        [                                                                                                                                                (L2*cos(q2) + L3*sin(q2 + q3))*sin(q1)],
        [                                                                                                                                                     L1 + L2*sin(q2) - L3*cos(q2 + q3)],
        [-(sin(q1) + sin(q4))*sin(q2 + q3)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)],
        [ (cos(q1) - cos(q4))*sin(q2 + q3)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)],
        [  (cos(q2 + q3) - 1)*sin(q1 + q4)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)]])
```

**IMPORTANT NOTE:** Please notice that symbolic computation is slower than numerical one, so use those commands only if you need to know the equations of motion of your system :wink:

[*Return to top*](#zrobotics-01)

---

### Jacobian Matrix

This is OPTIONAL, because each function who needs a Jacobian Matrix can call and process it automatically :wink: but you can calculate its geometrical or analytical form:

```python
# Kinematics libraries
from lib.kinematics.HTM import *
from lib.kinematics.DQ import *

# Screw vectors stored in a matrix. This is MANDATORY for calculations using Dual Quaternions
xi = np.array([[0, 0, 0, 0],
               [0, 0, 0, 0],
               [0, 0, 0, 0],
               [1, 1, 1, 1],
               [0, 0, 0, 0],
               [0, 0, 0, 0],
               [0, 0, 0, 0],
               [0, 0, 0, 0]])
  
# Geometric Jacobian Matrix (OPTIONAL)
Jg = geometricJacobian(uRobot)
symbolicJg = geometricJacobian(uRobot, symbolic = True)
  
# Analytic Jacobian Matrix (OPTIONAL)
Ja = analyticJacobian(uRobot)
symbolicJa = analyticJacobian(uRobot, symbolic = True)
  
# Dual Jacobian Matrix (OPTIONAL)
Jdq = jacobianDQ(uRobot, xi)
symbolicJdq = jacobianDQ(uRobot, xi, symbolic = True)
```

Then the output will be:

```bash
# NumPy Array
>>> Jg
array([[-7.55761747e-02,  9.96957218e-02,  1.71533593e-01,  -1.73472348e-18],
       [ 4.86840507e-01,  1.54765702e-02,  2.66285418e-02,  -1.38777878e-17],
       [ 0.00000000e+00,  4.92671734e-01,  9.93335155e-02,  -2.60208521e-18],
       [ 0.00000000e+00,  1.53400671e-01,  1.53400671e-01,   4.90789056e-01],
       [ 0.00000000e+00, -9.88164072e-01, -9.88164072e-01,   7.61891398e-02],
       [ 1.00000000e+00,  6.12323400e-17,  6.12323400e-17,  -8.67940849e-01]])

# SymPy Matrix
>>> symbolicJg
Matrix([[-(L2*cos(q2) + L3*sin(q2 + q3))*sin(q1), (-L2*sin(q2) + L3*cos(q2 + q3))*cos(q1), L3*cos(q1)*cos(q2 + q3),                    0],
        [ (L2*cos(q2) + L3*sin(q2 + q3))*cos(q1), (-L2*sin(q2) + L3*cos(q2 + q3))*sin(q1), L3*sin(q1)*cos(q2 + q3),                    0],
        [                                      0,            L2*cos(q2) + L3*sin(q2 + q3),         L3*sin(q2 + q3),                    0],
        [                                      0,                                 sin(q1),                 sin(q1), sin(q2 + q3)*cos(q1)],
        [                                      0,                                -cos(q1),                -cos(q1), sin(q1)*sin(q2 + q3)],
        [                                      1,                                       0,                       0,        -cos(q2 + q3)]])

# NumPy Array
>>> Ja
array([[-0.07581958,  0.09945228,  0.17148449,  0.        ],
       [ 0.48680264,  0.01543878,  0.02662092,  0.        ],
       [ 0.        ,  0.4927221 ,  0.09942029,  0.        ],
       [-0.41114626,  0.7160252 ,  0.7160252 ,  0.72266962],
       [-1.34563675, -0.31800502, -0.31800502,  1.20728129],
       [ 0.23911895, -1.0821444 , -1.0821444 ,  0.23911895]])

# SymPy Matrix
>>> symbolicJa
Matrix([[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           -(L2*cos(q2) + L3*sin(q2 + q3))*sin(q1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                (-L2*sin(q2) + L3*cos(q2 + q3))*cos(q1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                L3*cos(q1)*cos(q2 + q3),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  0],
        [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            (L2*cos(q2) + L3*sin(q2 + q3))*cos(q1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                (-L2*sin(q2) + L3*cos(q2 + q3))*sin(q1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                L3*sin(q1)*cos(q2 + q3),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  0],
        [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           L2*cos(q2) + L3*sin(q2 + q3),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        L3*sin(q2 + q3),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  0],
        [        -sin(q2 + q3)*cos(q1)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2) - (-2*sin(q1 + q4)*cos(q2 + q3) + 2*sin(q1 + q4))*(sin(q1) + sin(q4))*(cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)*sin(q2 + q3)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/(2*(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)**(3/2)) + (-sin(q1 + q4)*cos(q2 + q3)/2 + sin(q1 + q4)/2)*(sin(q1) + sin(q4))*sin(q2 + q3)/(sqrt(1 - (cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)**2)*sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)), -(sin(q1) + sin(q4))*cos(q2 + q3)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2) - (-2*sin(q2 + q3)*cos(q1 + q4) + 2*sin(q2 + q3))*(sin(q1) + sin(q4))*(cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)*sin(q2 + q3)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/(2*(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)**(3/2)) + (-sin(q2 + q3)*cos(q1 + q4)/2 + sin(q2 + q3)/2)*(sin(q1) + sin(q4))*sin(q2 + q3)/(sqrt(1 - (cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)**2)*sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)), -(sin(q1) + sin(q4))*cos(q2 + q3)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2) - (-2*sin(q2 + q3)*cos(q1 + q4) + 2*sin(q2 + q3))*(sin(q1) + sin(q4))*(cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)*sin(q2 + q3)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/(2*(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)**(3/2)) + (-sin(q2 + q3)*cos(q1 + q4)/2 + sin(q2 + q3)/2)*(sin(q1) + sin(q4))*sin(q2 + q3)/(sqrt(1 - (cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)**2)*sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)),         -sin(q2 + q3)*cos(q4)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2) - (-2*sin(q1 + q4)*cos(q2 + q3) + 2*sin(q1 + q4))*(sin(q1) + sin(q4))*(cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)*sin(q2 + q3)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/(2*(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)**(3/2)) + (-sin(q1 + q4)*cos(q2 + q3)/2 + sin(q1 + q4)/2)*(sin(q1) + sin(q4))*sin(q2 + q3)/(sqrt(1 - (cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)**2)*sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2))],
        [        -sin(q1)*sin(q2 + q3)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2) + (-2*sin(q1 + q4)*cos(q2 + q3) + 2*sin(q1 + q4))*(cos(q1) - cos(q4))*(cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)*sin(q2 + q3)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/(2*(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)**(3/2)) - (-sin(q1 + q4)*cos(q2 + q3)/2 + sin(q1 + q4)/2)*(cos(q1) - cos(q4))*sin(q2 + q3)/(sqrt(1 - (cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)**2)*sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)),  (cos(q1) - cos(q4))*cos(q2 + q3)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2) + (-2*sin(q2 + q3)*cos(q1 + q4) + 2*sin(q2 + q3))*(cos(q1) - cos(q4))*(cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)*sin(q2 + q3)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/(2*(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)**(3/2)) - (-sin(q2 + q3)*cos(q1 + q4)/2 + sin(q2 + q3)/2)*(cos(q1) - cos(q4))*sin(q2 + q3)/(sqrt(1 - (cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)**2)*sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)),  (cos(q1) - cos(q4))*cos(q2 + q3)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2) + (-2*sin(q2 + q3)*cos(q1 + q4) + 2*sin(q2 + q3))*(cos(q1) - cos(q4))*(cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)*sin(q2 + q3)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/(2*(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)**(3/2)) - (-sin(q2 + q3)*cos(q1 + q4)/2 + sin(q2 + q3)/2)*(cos(q1) - cos(q4))*sin(q2 + q3)/(sqrt(1 - (cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)**2)*sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)),          sin(q4)*sin(q2 + q3)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2) + (-2*sin(q1 + q4)*cos(q2 + q3) + 2*sin(q1 + q4))*(cos(q1) - cos(q4))*(cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)*sin(q2 + q3)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/(2*(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)**(3/2)) - (-sin(q1 + q4)*cos(q2 + q3)/2 + sin(q1 + q4)/2)*(cos(q1) - cos(q4))*sin(q2 + q3)/(sqrt(1 - (cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)**2)*sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2))],
        [(cos(q2 + q3) - 1)*cos(q1 + q4)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2) + (-2*sin(q1 + q4)*cos(q2 + q3) + 2*sin(q1 + q4))*(cos(q2 + q3) - 1)*(cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)*sin(q1 + q4)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/(2*(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)**(3/2)) - (-sin(q1 + q4)*cos(q2 + q3)/2 + sin(q1 + q4)/2)*(cos(q2 + q3) - 1)*sin(q1 + q4)/(sqrt(1 - (cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)**2)*sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)),          -sin(q1 + q4)*sin(q2 + q3)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2) + (-2*sin(q2 + q3)*cos(q1 + q4) + 2*sin(q2 + q3))*(cos(q2 + q3) - 1)*(cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)*sin(q1 + q4)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/(2*(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)**(3/2)) - (-sin(q2 + q3)*cos(q1 + q4)/2 + sin(q2 + q3)/2)*(cos(q2 + q3) - 1)*sin(q1 + q4)/(sqrt(1 - (cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)**2)*sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)),          -sin(q1 + q4)*sin(q2 + q3)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2) + (-2*sin(q2 + q3)*cos(q1 + q4) + 2*sin(q2 + q3))*(cos(q2 + q3) - 1)*(cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)*sin(q1 + q4)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/(2*(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)**(3/2)) - (-sin(q2 + q3)*cos(q1 + q4)/2 + sin(q2 + q3)/2)*(cos(q2 + q3) - 1)*sin(q1 + q4)/(sqrt(1 - (cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)**2)*sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)), (cos(q2 + q3) - 1)*cos(q1 + q4)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2) + (-2*sin(q1 + q4)*cos(q2 + q3) + 2*sin(q1 + q4))*(cos(q2 + q3) - 1)*(cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)*sin(q1 + q4)*acos(cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)/(2*(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2)**(3/2)) - (-sin(q1 + q4)*cos(q2 + q3)/2 + sin(q1 + q4)/2)*(cos(q2 + q3) - 1)*sin(q1 + q4)/(sqrt(1 - (cos(q1 + q4)*cos(q2 + q3)/2 - cos(q1 + q4)/2 - cos(q2 + q3)/2 - 1/2)**2)*sqrt(4 - (cos(q1 + q4)*cos(q2 + q3) - cos(q1 + q4) - cos(q2 + q3) - 1)**2))]])

# NumPy Array
>>> Jdq
array([[-0.1080318 , -0.26155557, -0.26155557, -0.1080318 ],
       [ 0.19613282, -0.11742141, -0.11742141, -0.19613282],
       [ 0.44161581,  0.05214984,  0.05214984, -0.44161581],
       [-0.06954512,  0.40630197,  0.40630197, -0.06954512],
       [ 0.07135409, -0.10805327, -0.09509771,  0.07135409],
       [-0.01503999,  0.03786138, -0.04307636,  0.01503999],
       [ 0.00667964,  0.08524929, -0.09699143, -0.00667964],
       [-0.11084186, -0.06955894, -0.06121884, -0.11084186]])

# SymPy Matrix
>>> symbolicJdq
Matrix([[                                                 -sin(q2/2 + q3/2)*cos(q1/2 + q4/2)/2,                                                                                                                                                                                          -sin(q1/2 - q2/2 - q3/2 + q4/2)/4 - sin(q1/2 + q2/2 + q3/2 + q4/2)/4,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   -sin(q1/2 - q2/2 - q3/2 + q4/2)/4 - sin(q1/2 + q2/2 + q3/2 + q4/2)/4,                                                                                                                                                                                                 -sin(q2/2 + q3/2)*cos(q1/2 + q4/2)/2],
        [                                                 -sin(q1/2 - q4/2)*cos(q2/2 + q3/2)/2,                                                                                                                                                                                         -sin(-q1/2 + q2/2 + q3/2 + q4/2)/4 - sin(q1/2 + q2/2 + q3/2 - q4/2)/4,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  -sin(-q1/2 + q2/2 + q3/2 + q4/2)/4 - sin(q1/2 + q2/2 + q3/2 - q4/2)/4,                                                                                                                                                                                                  sin(q1/2 - q4/2)*cos(q2/2 + q3/2)/2],
        [                                                  cos(q1/2 - q4/2)*cos(q2/2 + q3/2)/2,                                                                                                                                                                                         -cos(-q1/2 + q2/2 + q3/2 + q4/2)/4 + cos(q1/2 + q2/2 + q3/2 - q4/2)/4,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  -cos(-q1/2 + q2/2 + q3/2 + q4/2)/4 + cos(q1/2 + q2/2 + q3/2 - q4/2)/4,                                                                                                                                                                                                 -cos(q1/2 - q4/2)*cos(q2/2 + q3/2)/2],
        [                                                 -sin(q1/2 + q4/2)*sin(q2/2 + q3/2)/2,                                                                                                                                                                                           cos(q1/2 - q2/2 - q3/2 + q4/2)/4 + cos(q1/2 + q2/2 + q3/2 + q4/2)/4,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    cos(q1/2 - q2/2 - q3/2 + q4/2)/4 + cos(q1/2 + q2/2 + q3/2 + q4/2)/4,                                                                                                                                                                                                 -sin(q1/2 + q4/2)*sin(q2/2 + q3/2)/2],
        [ (L1*sin(q2/2 + q3/2) + L2*cos(q2/2 - q3/2) + L3*sin(q2/2 + q3/2))*sin(q1/2 + q4/2)/4,  L1*sin(q1/2)*sin(q4/2)*cos(q2/2 + q3/2)/2 - L1*cos(q1/2 - q4/2)*cos(q2/2 + q3/2)/4 - L2*sin(q1/2)*sin(q4/2)*sin(q2/2 - q3/2)/2 + L2*sin(q2/2 - q3/2)*cos(q1/2 - q4/2)/4 + L3*sin(q1/2)*sin(q4/2)*cos(q2/2 + q3/2)/2 - L3*cos(q1/2 - q4/2)*cos(q2/2 + q3/2)/4,                                                                                                                                                                                                                                                                                                                                                                       L1*sin(q2/2)**3*sin(q3/2)*cos(q1/2 + q4/2)/4 + L1*sin(q2/2)**2*cos(q2/2)*cos(q3/2)*cos(q1/2 + q4/2)/4 - L1*sin(q2/2)*sin(q2/2 - q3/2)*cos(q2/2)*cos(q1/2 + q4/2)/4 - L1*cos(q2/2)*cos(q3/2)*cos(q1/2 + q4/2)/4 - L2*sin(q2/2)**3*cos(q3/2)*cos(q1/2 + q4/2)/4 - L2*sin(q2/2)**2*sin(q3/2)*cos(q2/2)*cos(q1/2 + q4/2)/4 - L2*sin(q2/2)*cos(q2/2)*cos(q1/2 + q4/2)*cos(q2/2 + q3/2)/4 + L2*sin(q3/2)*cos(q2/2)*cos(q1/2 + q4/2)/4 + L3*sin(q2/2)**3*sin(q3/2)*cos(q1/2 + q4/2)/4 + L3*sin(q2/2)**2*cos(q2/2)*cos(q3/2)*cos(q1/2 + q4/2)/4 - L3*sin(q2/2)*sin(q2/2 - q3/2)*cos(q2/2)*cos(q1/2 + q4/2)/4 - L3*cos(q2/2)*cos(q3/2)*cos(q1/2 + q4/2)/4,    L1*cos(q1/2 - q2/2 - q3/2 + q4/2)/8 - L1*cos(q1/2 + q2/2 + q3/2 + q4/2)/8 + L2*sin(q1/2 - q2/2 + q3/2 + q4/2)/8 + L2*sin(q1/2 + q2/2 - q3/2 + q4/2)/8 + L3*cos(q1/2 - q2/2 - q3/2 + q4/2)/8 - L3*cos(q1/2 + q2/2 + q3/2 + q4/2)/8],
        [(-L1*cos(q2/2 + q3/2) - L2*sin(q2/2 - q3/2) + L3*cos(q2/2 + q3/2))*cos(q1/2 - q4/2)/4,  L1*sin(q1/2)*sin(q2/2 + q3/2)*cos(q4/2)/2 - L1*sin(q1/2 + q4/2)*sin(q2/2 + q3/2)/4 - L2*sin(q1/2)*cos(q4/2)*cos(q2/2 - q3/2)/2 + L2*sin(q1/2 + q4/2)*cos(q2/2 - q3/2)/4 - L3*sin(q1/2)*sin(q2/2 + q3/2)*cos(q4/2)/2 + L3*sin(q1/2 + q4/2)*sin(q2/2 + q3/2)/4,  L1*sin(q1/2)**2*sin(q2/2)**2*sin(q1/2 - q4/2)*sin(q2/2 + q3/2)/4 + L1*sin(q1/2)**2*sin(q1/2 - q4/2)*sin(q2/2 + q3/2)*cos(q2/2)**2/4 + L1*sin(q2/2)**3*sin(q1/2 - q4/2)*cos(q1/2)**2*cos(q3/2)/4 + L1*sin(q2/2)*sin(q1/2 - q4/2)*cos(q1/2)**2*cos(q2/2)*cos(q2/2 - q3/2)/4 + L1*sin(q3/2)*sin(q1/2 - q4/2)*cos(q1/2)**2*cos(q2/2)**3/4 + L2*sin(q1/2)**2*sin(q2/2)**2*sin(q1/2 - q4/2)*cos(q2/2 - q3/2)/4 + L2*sin(q1/2)**2*sin(q1/2 - q4/2)*cos(q2/2)**2*cos(q2/2 - q3/2)/4 + L2*sin(q2/2)**3*sin(q3/2)*sin(q1/2 - q4/2)*cos(q1/2)**2/4 + L2*sin(q2/2)*sin(q1/2 - q4/2)*sin(q2/2 + q3/2)*cos(q1/2)**2*cos(q2/2)/4 + L2*sin(q1/2 - q4/2)*cos(q1/2)**2*cos(q2/2)**3*cos(q3/2)/4 - L3*sin(q1/2)**2*sin(q2/2)**3*sin(q1/2 - q4/2)*cos(q3/2)/4 - L3*sin(q1/2)**2*sin(q2/2)*sin(q1/2 - q4/2)*cos(q2/2)*cos(q2/2 - q3/2)/4 - L3*sin(q1/2)**2*sin(q3/2)*sin(q1/2 - q4/2)*cos(q2/2)**3/4 - L3*sin(q2/2)**2*sin(q1/2 - q4/2)*sin(q2/2 + q3/2)*cos(q1/2)**2/4 - L3*sin(q1/2 - q4/2)*sin(q2/2 + q3/2)*cos(q1/2)**2*cos(q2/2)**2/4,  L1*cos(-q1/2 + q2/2 + q3/2 + q4/2)/8 + L1*cos(q1/2 + q2/2 + q3/2 - q4/2)/8 - L2*sin(q1/2 - q2/2 + q3/2 - q4/2)/8 + L2*sin(q1/2 + q2/2 - q3/2 - q4/2)/8 - L3*cos(-q1/2 + q2/2 + q3/2 + q4/2)/8 - L3*cos(q1/2 + q2/2 + q3/2 - q4/2)/8],
        [(-L1*cos(q2/2 + q3/2) - L2*sin(q2/2 - q3/2) + L3*cos(q2/2 + q3/2))*sin(q1/2 - q4/2)/4, -L1*sin(q1/2)*sin(q4/2)*sin(q2/2 + q3/2)/2 - L1*sin(q2/2 + q3/2)*cos(q1/2 + q4/2)/4 + L2*sin(q1/2)*sin(q4/2)*cos(q2/2 - q3/2)/2 + L2*cos(q1/2 + q4/2)*cos(q2/2 - q3/2)/4 + L3*sin(q1/2)*sin(q4/2)*sin(q2/2 + q3/2)/2 + L3*sin(q2/2 + q3/2)*cos(q1/2 + q4/2)/4,                                                                                                                                                                                                                                                                                                                                                                      -L1*sin(q2/2)**3*cos(q3/2)*cos(q1/2 - q4/2)/4 + L1*sin(q2/2)**2*sin(q3/2)*cos(q2/2)*cos(q1/2 - q4/2)/4 - L1*sin(q2/2)*cos(q2/2)*cos(q1/2 - q4/2)*cos(q2/2 - q3/2)/4 - L1*sin(q3/2)*cos(q2/2)*cos(q1/2 - q4/2)/4 - L2*sin(q2/2)**3*sin(q3/2)*cos(q1/2 - q4/2)/4 + L2*sin(q2/2)**2*cos(q2/2)*cos(q3/2)*cos(q1/2 - q4/2)/4 - L2*sin(q2/2)*sin(q2/2 + q3/2)*cos(q2/2)*cos(q1/2 - q4/2)/4 - L2*cos(q2/2)*cos(q3/2)*cos(q1/2 - q4/2)/4 + L3*sin(q2/2)**3*cos(q3/2)*cos(q1/2 - q4/2)/4 - L3*sin(q2/2)**2*sin(q3/2)*cos(q2/2)*cos(q1/2 - q4/2)/4 + L3*sin(q2/2)*cos(q2/2)*cos(q1/2 - q4/2)*cos(q2/2 - q3/2)/4 + L3*sin(q3/2)*cos(q2/2)*cos(q1/2 - q4/2)/4, -L1*sin(-q1/2 + q2/2 + q3/2 + q4/2)/8 + L1*sin(q1/2 + q2/2 + q3/2 - q4/2)/8 + L2*cos(q1/2 - q2/2 + q3/2 - q4/2)/8 - L2*cos(q1/2 + q2/2 - q3/2 - q4/2)/8 + L3*sin(-q1/2 + q2/2 + q3/2 + q4/2)/8 - L3*sin(q1/2 + q2/2 + q3/2 - q4/2)/8],
        [-(L1*sin(q2/2 + q3/2) + L2*cos(q2/2 - q3/2) + L3*sin(q2/2 + q3/2))*cos(q1/2 + q4/2)/4, -L1*sin(q1/2)*cos(q4/2)*cos(q2/2 + q3/2)/2 + L1*sin(q1/2 - q4/2)*cos(q2/2 + q3/2)/4 + L2*sin(q1/2)*sin(q2/2 - q3/2)*cos(q4/2)/2 - L2*sin(q1/2 - q4/2)*sin(q2/2 - q3/2)/4 - L3*sin(q1/2)*cos(q4/2)*cos(q2/2 + q3/2)/2 + L3*sin(q1/2 - q4/2)*cos(q2/2 + q3/2)/4, -L1*sin(q1/2)**3*cos(q2/2)**2*cos(q4/2)*cos(q2/2 + q3/2)/4 - L1*sin(q1/2)**2*sin(q2/2)**2*sin(q1/2 + q4/2)*cos(q2/2 + q3/2)/4 - L1*sin(q1/2)*cos(q1/2)*cos(q2/2)**2*cos(q1/2 - q4/2)*cos(q2/2 + q3/2)/4 - L1*sin(q2/2)**2*sin(q1/2 + q4/2)*cos(q1/2)**2*cos(q2/2 + q3/2)/4 - L1*sin(q4/2)*cos(q1/2)**3*cos(q2/2)**2*cos(q2/2 + q3/2)/4 - L2*sin(q1/2)**3*sin(q2/2)**2*sin(q2/2 - q3/2)*cos(q4/2)/4 - L2*sin(q1/2)**2*sin(q1/2 + q4/2)*sin(q2/2 - q3/2)*cos(q2/2)**2/4 - L2*sin(q1/2)*sin(q2/2)**2*sin(q2/2 - q3/2)*cos(q1/2)*cos(q1/2 - q4/2)/4 - L2*sin(q2/2)**2*sin(q4/2)*sin(q2/2 - q3/2)*cos(q1/2)**3/4 - L2*sin(q1/2 + q4/2)*sin(q2/2 - q3/2)*cos(q1/2)**2*cos(q2/2)**2/4 - L3*sin(q1/2)**3*cos(q2/2)**2*cos(q4/2)*cos(q2/2 + q3/2)/4 - L3*sin(q1/2)**2*sin(q2/2)**2*sin(q1/2 + q4/2)*cos(q2/2 + q3/2)/4 - L3*sin(q1/2)*cos(q1/2)*cos(q2/2)**2*cos(q1/2 - q4/2)*cos(q2/2 + q3/2)/4 - L3*sin(q2/2)**2*sin(q1/2 + q4/2)*cos(q1/2)**2*cos(q2/2 + q3/2)/4 - L3*sin(q4/2)*cos(q1/2)**3*cos(q2/2)**2*cos(q2/2 + q3/2)/4,    L1*sin(q1/2 - q2/2 - q3/2 + q4/2)/8 - L1*sin(q1/2 + q2/2 + q3/2 + q4/2)/8 - L2*cos(q1/2 - q2/2 + q3/2 + q4/2)/8 - L2*cos(q1/2 + q2/2 - q3/2 + q4/2)/8 + L3*sin(q1/2 - q2/2 - q3/2 + q4/2)/8 - L3*sin(q1/2 + q2/2 + q3/2 + q4/2)/8]]) 
```

**IMPORTANT NOTE:** Please notice that symbolic computation is slower than numerical one, so use those commands only if you need to know the equations of motion of your system :wink:

[*Return to top*](#zrobotics-01)

---

#### Inverse Kinematics (*Error Feedback*)

Instead of calculating horrible and complex equations, we use a numeiical algorithm to calculate inverse kinematics. It is asymptotically stable only if the pose to be reached is inside the robot's workspace, i.e. the point to be reached is reachable

```python
# Kinematics libraries
from lib.kinematics.HTM import *
from lib.kinematics.DQ import *

# Calculate robot's Inverse Kinematics to a single point (using Homogeneous Transformation Matrices)
qHTM = inverseHTM(uRobot, q0 = np.random.rand(4, 1), Hd = fkHTM[-1], K = 50 * np.eye(6))
  
# Calculate robot's Inverse Kinematics to a single point (Dual Quaternions)
qDQ = inverseDQ(uRobot, q0 = np.random.rand(4, 1), Qd = fkDQ[-1], K = 50 * np.eye(8), xi = xi)
```

<img src="https://render.githubusercontent.com/render/math?math=q_0 \in \mathbb{R}^{n \times 1}"> are the initial conditions of the generalized coordinates; also, <img src="https://render.githubusercontent.com/render/math?math=H_d \in \mathbb{R}^{4 \times 4}"> and <img src="https://render.githubusercontent.com/render/math?math=Q_d \in \mathbb{H}"> represent the desired frame's pose using an Homogeneous Transformation Matrix or a Dual Quaternion respectively. Last but not least, <img src="https://render.githubusercontent.com/render/math?math=K \in \mathbb{R}^{6 \times 6}"> and <img src="https://render.githubusercontent.com/render/math?math=K_Q \in \mathbb{R}^{8 \times 8}"> are the constant symmetric gain matrices that are used to solve inverse kinematics problem

**IMPORTANT NOTE:** Inverse kinematics algorithms returns a generalized coordinates vector <img src="https://render.githubusercontent.com/render/math?math=q \in \mathbb{R}^{n \times p}">, where <img src="https://render.githubusercontent.com/render/math?math=p \in \mathbb{R}, p \geq 1"> is the number of joints' positions that have to be reached

[*Return to top*](#zrobotics-01)

---

**We hope this can be useful for you. Thank you!**

    
![Z Dynamics](img/icon.png "The Future is ROBOTICS")
