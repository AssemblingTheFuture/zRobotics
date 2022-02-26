# zRobotics 0.1

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) [![License: CC BY-NC 4.0](https://img.shields.io/badge/License-CC%20BY--NC%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc/4.0/)

A powerful library for robotics analysis :mechanical_arm: :robot:

- Index :card_index_dividers:
    - [Introduction](#introduction-books) :books:
    - [Features](#features-sparkles) :sparkles:
    - [Future Work](#future-work-cloud) :cloud:
    - [Before Starting](#before-starting-warning) :warning:
      - [Symbolic Computation](#symbolic-computation-warning) :warning:
    - [Library Content](#library-content-book) :book:
      - [How to import it?](#how-to-import-it-man_technologist) :man_technologist:
      - [(Brief) Libraries' Description](#brief-libraries-description-blue_book) :blue_book:
    - [Movements](#movements)
      - [Translation](#translation)
      - [Rotation](#rotation)
    - [Dual Quaternions Functionalities](#dual-quaternions-functionalities)
      - [Dual Quaternions Multiplication](#dual-quaternions-multiplication)
      - [Dual Quaternions Conjugate](#dual-quaternions-conjugate)
      - [Dual Quaternions to Euclidian Space](#dual-quaternions-to-euclidian-space)
    - [Robot Creation and Setup](#robot-creation-and-setup-mechanical_leg) :mechanical_leg:
      - [Attributes](#attributes)
      - [Creation](#creation)
      - [Denavit - Hartenberg Parameters](#denavit---hartenberg-parameters)
      - [Centers of Mass](#centers-of-mass)
    - [Kinematics](#kinematics)
      - [Forward Kinematics](#forward-kinematics)
      - [Forward Kinematics to Center of Mass](#forward-kinematics-to-centers-of-mass)
      - [Axis - Angle Vector](#axis---angle-vector)
      - [Jacobian Matrix](#jacobian-matrix)
      - [Inverse Kinematics (Error Feedback)](#inverse-kinematics-error-feedback)
    - [Differential Kinematics](#differential-kinematics)
      - [Inertial Rate of Change](#inertial-rate-of-change)
      - [Inertial Velocity](#inertial-velocity)
      - [Inertial Velocity for Centers of Mass](#inertial-velocity-for-centers-of-mass)
    - [Dynamics](#dynamics)
      - [Inertia Matrix](#inertia-matrix)
      - [Kinetic Energy](#kinetic-energy)
      - [Potential Energy](#potential-energy)
      - [Gravity Actuation](#gravity-actuation)
      - [Centrifugal and Coriolis Effects](#centrifugal-and-coriolis-effects)
      - [Robot Model](#robot-model)

---

## Introduction :books:

You can use this simple library to analyze and develop kinematics and control algorithms for your robots as shown in our online trainigs:

- [Numerical Methods with Engineering Applications](https://bit.ly/NumericZ)
- [Control of Dynamical Systems](https://bit.ly/zControl) 
- [Robotics: from Kinematics to Control](https://bit.ly/RoboticZ)
 
We hope this library will help you to start your journey in these amazing discipline! :heart:

[*Return to top*](#zrobotics-01)

---

## Features :sparkles:

You can set your robot attributes and analyze its behavior; also, you'll be able to see its end - effector displacement in a 3D animation (*comming soon*). To achieve this, all the algorithms were developed using Homogeneous Transformation Matrices and Dual Quaternions algebra, however, **the logic used to develop them will allow you to adapt it to almost any embedded system!**

Some interesting functionalities can be listed as follows:

- [x] [Forward Kinematics](#forward-kinematics)
  - [x] [Using Homogeneous Transformation Matrices](/lib/kinematics/HTM.py#11) (numerical and symbolical)
  - [x] [Using Dual Quaternions](/lib/kinematics/DQ.py#11) (numerical and symbolical)
- [x] [Numerical Inverse Kinematics](#inverse-kinematics-error-feedback)
  - [x] [Using Homogeneous Transformation Matrices](/lib/kinematics/HTM.py#280)
  - [x] [Using Dual Quaternions](/lib/kinematics/DQ.py#135)
- [ ] [Differential Kinematics](#differential-kinematics)
  - [x] [Using Homogeneous Transformation Matrices](/lib/kinematics/DifferentialHTM.py) (numerical and Symbolical)
  - [ ] Using Dual Quaternions (:warning: **UNDER DEVELOPMENT** :warning:)
- Robot Dynamics
  - Robot Differential Equation using Homogeneous Transformation Matrices (numerical and symbolical)

Feel free to modify, adjust and extend our work to your necessities :smiley:; this library allows you to get a first approach to robot analysis, synthesis and control, however, we will be adding new interesting features, also, **you can request new features or create new ones!**

[*Return to top*](#zrobotics-01)

---

## Future Work :cloud: 

We are working, or will start working soon, on the following tasks for future releases:

- [x] ~~Velocity Analysis using Homogeneous Transformation Matrices~~
- [x] ~~Euler - Lagrange formulation using Homogeneous Transformation Matrices~~
- [ ] Newton - Euler Recursive Algorithm using Homogeneous Transformation Matrices
- [ ] Acceleration Analysis using Homogeneous Transformation Matrices


[*Return to top*](#zrobotics-01)

---

## Before Starting :warning:

This library can be used with [Python :snake: 3.8.10 - 64 bits](https://www.python.org/downloads/release/python-3810/) or superior. Please install the following libraries or make sure that they're already in your computer:

```bash
pip3 install numpy
pip3 install sympy
pip3 install matplotlib
```

**Please do not forget to set your Python Path** :wink:. Also, you can use ```pip``` depending on your Python version and/or configuration:

```bash
pip install numpy
pip install sympy
pip install matplotlib
```

If these modules were installed correctly, you will be able to use our library :smiley:

[*Return to top*](#zrobotics-01)

---

### Symbolic Computation :warning:

You can get your robot's equations in a symbolical form, however, please consider that **this computation is slower than numerical one**, also **resulting equations won't be simplified as we do in paper**. If you need to simplify the obtained results, you can use ```trigsimp()``` function as follows:

```python

"""
  Example of symbolcal simplification
"""

# Homogeneous Transformation Matrix library
from lib.movements.HTM import *

# SymPy library
from sympy import *

# Symbolical Homogeneous Transformation Matrix (multiplication is done with * because is a symbolical matrix)
H = rz(z = "q1", symbolic = True) * tx(x = "L1", symbolic = True) * rz(z = "q2", symbolic = True) * tx(x = "L2", symbolic = True)

# Simplified Homogeneous Transformation Matrix
Hsimplified = trigsimp(H)

```

So the outputs will be

```bash
>>> H
Matrix([[-sin(q1)*sin(q2) + cos(q1)*cos(q2), -sin(q1)*cos(q2) - sin(q2)*cos(q1), 0, L1*cos(q1) + L2*(-sin(q1)*sin(q2) + cos(q1)*cos(q2))],
        [ sin(q1)*cos(q2) + sin(q2)*cos(q1), -sin(q1)*sin(q2) + cos(q1)*cos(q2), 0,  L1*sin(q1) + L2*(sin(q1)*cos(q2) + sin(q2)*cos(q1))],
        [                                 0,                                  0, 1,                                                    0],
        [                                 0,                                  0, 0,                                                    1]])

>>> Hsimplified
Matrix([[cos(q1 + q2), -sin(q1 + q2), 0, L1*cos(q1) + L2*cos(q1 + q2)],
        [sin(q1 + q2),  cos(q1 + q2), 0, L1*sin(q1) + L2*sin(q1 + q2)],
        [           0,             0, 1,                            0],
        [           0,             0, 0,                            1]])
```

**Please consider that the bigger the equation, the slower the simplification**. Also ```trigsimp()``` will find the best possible value, then sometimes **your simplification won't be the same as the computer's, but it doesn't mean equations are wrong, but there are multiple ways to simplify them**

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
|   |   ├── DynamicsHTM.py          # Dynamics equations using Homogeneous Transformation Matrices 
|   |   └── Solver.py               # Numerical solvers for multiple purposes (UNDER DEVELOPMENT)
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
6. [```lib.plot```](/lib/plot): this allows to plot graphs of your robot's behavior (**under construction :construction:**)


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
Qx = dqRx(x = 0.5, symbolic = False)
Qy = dqRy(y = 0.5, symbolic = False)
Qz = dqRz(z = 0.5, symbolic = False)
```

[*Return to top*](#zrobotics-01)

---

## [Dual Quaternions Functionalities](/lib/movements/DQ.py)

As shown in our [online training](https://bit.ly/RoboticZ), there are multiple operations and operators for dual quaternions. Our library needs some of their functionalities to work correctly, so they are listed below

### Dual Quaternions Multiplication

Let's consider the dual quaternions <img src="https://render.githubusercontent.com/render/math?math={\color{red}\hat{q}_{a}, \hat{q}_{b} \in \mathbb{H}}"> that have to be multiplicated as <img src="https://render.githubusercontent.com/render/math?math={\color{red} \hat{q}_{c} = \hat{q}_{a} \cdot \hat{q}_{b}}">, so this operation can be simplified with left and right operators <img src="https://render.githubusercontent.com/render/math?math={\color{red}\left[ \cdot \right]_{\mathrm{L}}, \left[ \cdot \right]_{\mathrm{R}} \in \mathbb{R}^{8 \times 8}}">:

<img src="https://render.githubusercontent.com/render/math?math={\color{red}\hat{q}_{c} = \left[ \hat{q}_{a}\right]_{\mathrm{L}} \cdot \hat{q}_{b} = \left[ \hat{q}_{b}\right]_{\mathrm{R}} \cdot \hat{q}_{a}}">

Then these operators can be calculated as follows:

```python
"""
  Dual Quaternions Multiplication
"""

# Dual Quaternions library
from lib.movements.DQ import *

# NumPy Library for Matrix multiplication
import numpy as np

# Dual Quaternion 
Qa = dqTy(y = 0.5)

# Dual Quaternion 
Qb = dqRz(z = 0.5)

# Multiplication with Left Operator
QcLeft = leftOperator(Q = Qa, symbolic = False).dot(Qb)

# Multiplication with Right Operator
QcRight = rightOperator(Q = Qb, symbolic = False).dot(Qa)

```

So the outputs will be

```bash
# NumPy Array
>>> QcLeft
array([[0.96891242],
       [0.        ],
       [0.        ],
       [0.24740396],
       [0.        ],
       [0.06185099],
       [0.24222811],
       [0.        ]])

# NumPy Array
>>> QcRight
array([[0.96891242],
       [0.        ],
       [0.        ],
       [0.24740396],
       [0.        ],
       [0.06185099],
       [0.24222811],
       [0.        ]])
```

You can also calculate its symbolic expressions by setting ```symbolic``` parameter to ```True```.

[*Return to top*](#zrobotics-01)

---

### Dual Quaternions Conjugate

The inverse form of an Homogeneous Transformation Matrix can be calculated easily with ```numpy``` library, however, the inverse form of a dual quaternions is as simple as conjugating its values:

```python
"""
  Dual Quaternions Conjugate
"""

# Dual Quaternions library
from lib.movements.DQ import *

# Dual Quaternion 
Qa = dqTy(y = 0.5)

# Conjugate form of a Dual Quaternion 
Qb = conjugateDQ(Q = Qa, symbolic = False)
```

So the outputs will be

```bash
# NumPy Array
>>> Qa
array([[1.  ],
       [0.  ],
       [0.  ],
       [0.  ],
       [0.  ],
       [0.  ],
       [0.25],
       [0.  ]])

>>> Qb
array([[ 1.  ],
       [-0.  ],
       [-0.  ],
       [-0.  ],
       [ 0.  ],
       [-0.  ],
       [-0.25],
       [-0.  ]])
```

You can also calculate its symbolic expression by setting ```symbolic``` parameter to ```True```.

[*Return to top*](#zrobotics-01)

---

### Dual Quaternions to Euclidian Space

Sometimes it will be necessary to transform a pose representation as dual quaternion <img src="https://render.githubusercontent.com/render/math?math={\color{red}\hat{q}_{i/0}^0 \in \mathbb{H}}"> into an Euclidian space one <img src="https://render.githubusercontent.com/render/math?math={\color{red}\vec{r}_{i/0}^{0} \in \mathbb{R}^{3 \times 1}}">:

```python
"""
  Dual Quaternions to R3
"""

# Dual Quaternions library
from lib.movements.DQ import *

# Dual Quaternion 
Q = dqTy(y = 0.5)

# Euclidian space representation
r = dqToR3(Q, symbolic = False)
```

So the outputs will be

```bash
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

>>> r
array([0. , 0. , 0.5, 0. ])
```

You can also calculate its symbolic expression by setting ```symbolic``` parameter to ```True```.

[*Return to top*](#zrobotics-01)

---

## Robot Creation and Setup :mechanical_leg:

A robot can be created as an object, but this library only works with serial manipulators (we will add other type of robots soon :wink:). Before creating your system, it is necessary to set some attributes.

### Attributes

These are generated randomly as an example, but you can use your own values for each attribute :smiley:

```python
# NumPy library is necessary for joints positions
import numpy as np

# Number of rigid bodies
m = 4
  
# Number of Generalized Coordinates
n = 4
  
# Generalized coordinates (created randomly)
q = np.random.randn(n, 1)
  
# Joints velocities (created randomly)
qd = np.random.randn(n, 1)
  
# Screw vectors (or axes of actuation) stored in a matrix. This is MANDATORY for calculations using Dual Quaternions
xi = np.array([[0, 0],
               [0, 0],
               [0, 0],
               [1, 1],
               [0, 0],
               [0, 0],
               [0, 0],
               [0, 0]])
  
# Derivative of previous screw vectors (or axes of actuation) stored in a matrix. This is MANDATORY for calculations using Dual Quaternions
xid = np.zeros((n, 1))
  
# Links (created randomly)
L = [np.random.rand() for i in range(m)]
  
# Center of Mass of each link (created randomly)
Lcom = [value / 2 for value in L]

# Mass of each link (created randomly)
m = [np.random.rand() for i in range(m)]
  
# Inertia of each link (created randomly)
Inertia = [np.random.rand(3, 3) for i in range(n)]
```

Where each attribute is described below:

- Joints positions and velocities (generalized coordinates): <img src="https://render.githubusercontent.com/render/math?math={\color{red} q, \dot{q} \in \mathbb{R}^{n \times 1}}"> (set in <img src="https://render.githubusercontent.com/render/math?math={\color{red} rad}"> and <img src="https://render.githubusercontent.com/render/math?math={\color{red} \frac{rad}{sec}}">) 
- Links Lengths and distance to Centers of Mass: <img src="https://render.githubusercontent.com/render/math?math={\color{red} L, L_{com} \in \mathbb{R}^{m \times 1}}"> (set in meters)
- Screw vectors for Dual Quaternions operations: <img src="https://render.githubusercontent.com/render/math?math={\color{red} \xi, \dot{\xi} \in \mathbb{H}}">
- Mass of each link: <img src="https://render.githubusercontent.com/render/math?math={\color{red} m \in \mathbb{R}, m > 0}"> (set in kilograms)
- Inertia tensor of each link: <img src="https://render.githubusercontent.com/render/math?math={\color{red} I \in \mathbb{R}^{3 \times 3}}"> (set in <img src="https://render.githubusercontent.com/render/math?math={\color{red} kg \cdot m^2}">)

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
uRobot = Serial(jointsPositions = q, jointsVelocities = qd, linksLengths = L, COMs = Lcom, mass = m, inertia = Inertia, name = 'uRobot', xi = xi, xid = xid)
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

The rest of attributes were ommitted, but you can check them in [Robot.py](/lib/Robot.py). You can also add new attributes if they are necessary for your project :smiley:

[*Return to top*](#zrobotics-01)

---

### Denavit - Hartenberg Parameters

Any serial manipulator is built by a Denavit - Hartenberg Parameters Matrix as this

|<img src="https://render.githubusercontent.com/render/math?math={\color{red}\theta_z}">|<img src="https://render.githubusercontent.com/render/math?math={\color{red}d_z}">|<img src="https://render.githubusercontent.com/render/math?math={\color{red}a_x}">|<img src="https://render.githubusercontent.com/render/math?math={\color{red}\alpha_x}">|
|:---:|:---:|:---:|:---:|
| 0 | 0 | 0 | 0 |
|<img src="https://render.githubusercontent.com/render/math?math={\color{red}q_1}">|<img src="https://render.githubusercontent.com/render/math?math={\color{red}L_1}">|0|<img src="https://render.githubusercontent.com/render/math?math={\color{red}\frac{\pi}{2}}">|
|<img src="https://render.githubusercontent.com/render/math?math={\color{red}q_2}">|0|<img src="https://render.githubusercontent.com/render/math?math={\color{red}L_2}">|0|
|<img src="https://render.githubusercontent.com/render/math?math={\color{red}q_3}">|0|0|<img src="https://render.githubusercontent.com/render/math?math={\color{red}\frac{\pi}{2}}">|
|<img src="https://render.githubusercontent.com/render/math?math={\color{red}q_4}">|<img src="https://render.githubusercontent.com/render/math?math={\color{red}L_3}">|0|0|

so is mandatory to modify [Robot.py](/lib/Robot.py#L74) with your robot's information, as you would do it in a sheet of paper (**do not forget to include inertial frame**):

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

It is not necessary to call this function before performing any kinematics task, this is because all the algorithms will access to these methods automatically. **For future releases, we will work on a simpler way to create this matrices** :wink:

[*Return to top*](#zrobotics-01)

---

### Centers of Mass

Some calculations in robotics needs to be performed with respect to the Center of Mass, so it is mandatory to modify [Robot's file](/lib/Robot.py#L73) with your robot's information, as you would do it in a sheet of paper (**do not forget to include inertial frame**). For example:

|<img src="https://render.githubusercontent.com/render/math?math={\color{red}\theta_z}">|<img src="https://render.githubusercontent.com/render/math?math={\color{red}d_z}">|<img src="https://render.githubusercontent.com/render/math?math={\color{red}a_x}">|<img src="https://render.githubusercontent.com/render/math?math={\color{red}\alpha_x}">|
|:---:|:---:|:---:|:---:|
| 0 | 0 | 0 | 0 |
|<img src="https://render.githubusercontent.com/render/math?math={\color{red}q_1}">|<img src="https://render.githubusercontent.com/render/math?math={\color{red}L_{com_{1}}}">|0|<img src="https://render.githubusercontent.com/render/math?math={\color{red}\frac{\pi}{2}}">|
|<img src="https://render.githubusercontent.com/render/math?math={\color{red}q_2}">|0|<img src="https://render.githubusercontent.com/render/math?math={\color{red}L_{com_{2}}}">|0|
|<img src="https://render.githubusercontent.com/render/math?math={\color{red}q_3}">|0|0|<img src="https://render.githubusercontent.com/render/math?math={\color{red}\frac{\pi}{2}}">|
|<img src="https://render.githubusercontent.com/render/math?math={\color{red}q_4}">|<img src="https://render.githubusercontent.com/render/math?math={\color{red}L_{com_{3}}}">|0|0|

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

Where <img src="https://render.githubusercontent.com/render/math?math={\color{red}f_{k_{HTM}} \in \mathbb{R}^{4 \times 4}}"> and <img src="https://render.githubusercontent.com/render/math?math={\color{red}f_{k_{DQ}} \in \mathbb{H}}"> are lists that store the pose representation for each reference frame with Homogeneous Transformation Matrices or Dual Quaternions. You can get all the elements of the list, but also you can access to each specific pose representation by indexing it:

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

**IMPORTANT NOTE:** Please notice that symbolic computation is slower than numerical one, so use those commands only if you need to know the equations of your system. Take a look at [Symbolic Computation](#symbolic-computation-warning) for more information :wink:

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

Where <img src="https://render.githubusercontent.com/render/math?math={\color{red}f_{com_{HTM}} \in \mathbb{R}^{4 \times 4}}"> and <img src="https://render.githubusercontent.com/render/math?math={\color{red}f_{com_{DQ}} \in \mathbb{H}}"> are lists that store the pose representation for each center of mass with Homogeneous Transformation Matrices or Dual Quaternions. You can get all the elements of the list, but also you can access to each specific pose representation by indexing it:

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

In this case, <img src="https://render.githubusercontent.com/render/math?math={\color{red}H_{com_{i}/0}^{0} \in \mathbb{R}^{4 \times 4}}"> and <img src="https://render.githubusercontent.com/render/math?math={\color{red}Q_{com_{i}/0}^{0} \in \mathbb{H}}"> are defined as <img src="https://render.githubusercontent.com/render/math?math={\color{red}H_{com_{i}/0}^{0} = H_{i/0}^{0} (H_{i/i-1}^{i - 1})^{-1} H_{com_{i}/i-1}^{i - 1}}"> and <img src="https://render.githubusercontent.com/render/math?math={\color{red}Q_{com_{i}/0}^{0} = Q_{i/0}^{0} (Q_{i/i-1}^{i - 1})^{*} Q_{com_{i}/i-1}^{i - 1}}"> respectively

**IMPORTANT NOTE:** Please notice that symbolic computation is slower than numerical one, so use those commands only if you need to know the equations of your system. Take a look at [Symbolic Computation](#symbolic-computation-warning) for more information :wink:

[*Return to top*](#zrobotics-01)

---

### Axis - Angle Vector

A compact representation of an Homogeneous Transformation Matrix can be obtained by and Axis - Angle Vector <img src="https://render.githubusercontent.com/render/math?math={\color{red} \mathrm{x} \in \mathbb{R}^{6 \times 1}}">. This is OPTIONAL, because each function can call it automatically if needed:

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

**IMPORTANT NOTE:** Please notice that symbolic computation is slower than numerical one, so use those commands only if you need to know the equations of your system. Take a look at [Symbolic Computation](#symbolic-computation-warning) for more information :wink:

[*Return to top*](#zrobotics-01)

---

### Jacobian Matrix

This is **OPTIONAL**, because each function who needs a Jacobian Matrix <img src="https://render.githubusercontent.com/render/math?math={\color{red}J \left( \mathrm{x} \right) \in \mathbb{R}^{6 \times n}}"> can call and process it automatically :wink: but you can calculate its geometrical or analytical form:

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

**IMPORTANT NOTE:** Please notice that symbolic computation is slower than numerical one, so use those commands only if you need to know the equations of your system. Take a look at [Symbolic Computation](#symbolic-computation-warning) for more information :wink:

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

<img src="https://render.githubusercontent.com/render/math?math={\color{red}q_0 \in \mathbb{R}^{n \times 1}}"> is the initial condition of the generalized coordinates; also, <img src="https://render.githubusercontent.com/render/math?math={\color{red}H_d \in \mathbb{R}^{4 \times 4}}"> and <img src="https://render.githubusercontent.com/render/math?math={\color{red}Q_d \in \mathbb{H}}"> represent the desired frame's pose using an Homogeneous Transformation Matrix or a Dual Quaternion respectively. Last but not least, <img src="https://render.githubusercontent.com/render/math?math={\color{red}K \in \mathbb{R}^{6 \times 6}}"> and <img src="https://render.githubusercontent.com/render/math?math={\color{red}K_Q \in \mathbb{R}^{8 \times 8}}"> are the constant gain matrices that are used to solve inverse kinematics problem

**IMPORTANT NOTE:** Inverse kinematics algorithms returns a generalized coordinates vector <img src="https://render.githubusercontent.com/render/math?math={\color{red}q \in \mathbb{R}^{n \times p}}">, where <img src="https://render.githubusercontent.com/render/math?math={\color{red}p \in \mathbb{R}, p \geq 1}"> is the number of joints positions that will be reached

[*Return to top*](#zrobotics-01)

---

## [Differential Kinematics](/lib/kinematics/)

This is the relation between motion (velocity) in joint space and motion (linear/angular velocity) in task space (e.g., Cartesian space), but it is known that time derivative of end-effector pose won't lead to its inertial velocity

### Inertial Rate of Change

The rate of change of the end-effector (is not the same as its velocity) can be calculated by deriving its equations that define its pose in an [Axis - Angle vector](#axis---angle-vector) <img src="https://render.githubusercontent.com/render/math?math={\color{red}\mathrm{x} \left( t \right) \in \mathbb{R}^{6 \times 1}}">, so this will return an [Analytic Jacobian Matrix](/lib/kinematics/HTM.py#225) <img src="https://render.githubusercontent.com/render/math?math={\color{red}J \left( \mathrm{x} \right) \in \mathbb{R}^{6 \times n}}"> that can be multiplied by the generalized coordinates vector:

<img src="https://render.githubusercontent.com/render/math?math={\color{red}\frac{\partial \mathrm{x}}{\partial q} = J \left ( \mathrm{x} \right) \implies \frac{\partial \mathrm{x}}{\partial q} = J \left ( \mathrm{x} \right) \frac{\partial t}{\partial t} \implies \frac{\partial \mathrm{x}}{\partial t} = J \left ( \mathrm{x} \right) \frac{\partial q}{\partial t} \implies \dot{\mathrm{x}} = J \left ( \mathrm{x} \right) \dot{q}}">

This calculation can be done as follows:

```python
"""
  Inertial Rate of Change
"""

# Differential Kinematics library
from lib.kinematics.DifferentialHTM import *

# NumPY
import numpy as np

# Analytic Jacobian Matrix (dq: step size of numerical derivative)
Ja = analyticJacobian(uRobot, dq = 0.001, symbolic = False)

# Inertial Rate of Change (calculated by analytic jacobian and joints velocities)
Xd1 = Ja.dot(uRobot.jointsVelocities)

# Inertial Rate of Change (calculated by library's module)
Xd2 = analyticStateSpace(uRobot, dq = 0.001, symbolic = False)
```

So the outputs will be

```bash
# NumPy Array
>>> Ja
array([[ 0.29672459, -0.04726979,  0.38999721,  0.        ],
       [-0.32124701, -0.04361767,  0.35986554,  0.        ],
       [ 0.        , -0.43734803, -0.10510702,  0.        ],
       [-0.11043124,  0.73688832,  0.73688832, -0.02344649],
       [ 1.49976911,  0.02134172,  0.02134172, -1.50364647],
       [ 0.10196457,  1.08967036,  1.08967036,  0.10196457]])

>>> Xd1
array([[ 0.20836402],
       [-0.11456081],
       [ 0.05803193],
       [-0.02882665],
       [ 4.45519401],
       [-0.24049479]])

>>> Xd2
array([[ 0.20836402],
       [-0.11456081],
       [ 0.05803193],
       [-0.02882665],
       [ 4.45519401],
       [-0.24049479]])
```

You can also calculate its symbolic expression by setting ```symbolic``` parameter to ```True```.

[*Return to top*](#zrobotics-01)

---

### Inertial Velocity

End-effector velocity can be calcualted with [Geometric Jacobian Matrix](/lib/kinematics/HTM.py#130), because this maps the effect of each joint directly to the end-effector, so linear and angular velocities can be calculated:

<img src="https://render.githubusercontent.com/render/math?math={\color{red}\mathrm{v} = \begin{bmatrix} v_{x} \\ v_{y} \\ v_{z} \\ \omega_{x} \\ \omega_{y} \\ \omega_{z} \end{bmatrix} = J \left ( q \right) \dot{q}}">

This can be calculated with the library as follows:

```python
"""
  Inertial Velocity
"""

# Differential Kinematics library
from lib.kinematics.DifferentialHTM import *

# NumPY
import numpy as np

# Geometric Jacobian Matrix
Jg = geometricJacobian(uRobot, symbolic = False)

# Inertial Velocity (calculated by geometric jacobian and joints velocities)
Xd1 = Jg.dot(uRobot.jointsVelocities)

# Inertial Velocity (calculated by library's module)
Xd2 = geometricStateSpace(uRobot, symbolic = False)
```

So the outputs will be

```bash
# NumPy Array
>>> Jg
array([[ 2.96563941e-01, -4.74304994e-02,  3.89958559e-01, -1.38777878e-17],
       [-3.21395346e-01, -4.37659599e-02,  3.59829875e-01, 4.16333634e-17],
       [ 0.00000000e+00, -4.37315835e-01, -1.05372345e-01, -1.73472348e-18],
       [ 0.00000000e+00,  6.78145901e-01,  6.78145901e-01, -1.43152138e-01],
       [ 0.00000000e+00, -7.34927300e-01, -7.34927300e-01, -1.32092026e-01],
       [ 1.00000000e+00,  6.12323400e-17,  6.12323400e-17, -9.80846146e-01]])

>>> Xd1
array([[ 0.20830251],
       [-0.11461763],
       [ 0.05799426],
       [ 0.32363338],
       [ 0.35274929],
       [ 2.91790934]])

>>> Xd2
array([[ 0.20830251],
       [-0.11461763],
       [ 0.05799426],
       [ 0.32363338],
       [ 0.35274929],
       [ 2.91790934]])
```

Please notice that angular velocities are not the same as the angular rate of change on [Inertial Rate of Change](#inertial-rate-of-change) results. You can also calculate its symbolic expression by setting ```symbolic``` parameter to ```True```.

[*Return to top*](#zrobotics-01)

---

### Inertial Velocity for Centers of Mass

**For dynamic modelling, it will be mandatory to know the velocity of each center of mass**. As stated in previous section, inertial velocities can be calcualted with [Geometric Jacobian Matrix](/lib/kinematics/HTM.py#130). In this case, it maps the effect of each joint directly to the each center of mass, so linear and angular velocities can be calculated:

<img src="https://render.githubusercontent.com/render/math?math={\color{red}\mathrm{v}_{com_i} = \begin{bmatrix} v_{x_{com_i}} \\ v_{y_{com_i}} \\ v_{z_{com_i}} \\ \omega_{x_{com_i}} \\ \omega_{y_{com_i}} \\ \omega_{z_{com_i}} \end{bmatrix} = J_{com_i} \left ( q \right) \dot{q}}">

This can be calculated with the library as follows:

```python
"""
  Inertial Velocity to a Center of Mass
"""

# Differential Kinematics library
from lib.kinematics.DifferentialHTM import *

# NumPY
import numpy as np

# Geometric Jacobian Matrix for a specific center of mass
JgCOM = geometricJacobianCOM(uRobot, COM = 2, symbolic = False)

# Inertial Velocity (calculated by geometric jacobian to center of mass and joints velocities)
XdCOM1 = JgCOM.dot(uRobot.jointsVelocities)

# Inertial Velocity (calculated by library's module)
XdCOM2 = geometricCOMStateSpace(uRobot, COM = 2, symbolic = False)
```

So the outputs will be

```bash
# NumPy Array
>>> JgCOM
array([[ 1.12553059e-01, -2.18694529e-01,  0.00000000e+00, 0.00000000e+00],
       [-1.21977167e-01, -2.01797917e-01,  0.00000000e+00, 0.00000000e+00],
       [ 0.00000000e+00, -1.65971745e-01,  0.00000000e+00, 0.00000000e+00],
       [ 0.00000000e+00,  6.78145901e-01,  0.00000000e+00, 0.00000000e+00],
       [ 0.00000000e+00, -7.34927300e-01,  0.00000000e+00, 0.00000000e+00],
       [ 1.00000000e+00,  6.12323400e-17,  0.00000000e+00, 0.00000000e+00]])

>>> XdCOM1
array([[ 0.09348339],
       [-0.03018717],
       [ 0.02690157],
       [-0.10991743],
       [ 0.11912085],
       [ 0.51563446]])

>>> XdCOM2
array([[ 0.09348339],
       [-0.03018717],
       [ 0.02690157],
       [-0.10991743],
       [ 0.11912085],
       [ 0.51563446]])
```

Please notice that jacobian matrix is zero in columns two to four because these joints (<img src="https://render.githubusercontent.com/render/math?math={\color{red}q_{3}, q_{4}}">) don't affect the center of mass number two because they are attached after it. You can also calculate its symbolic expression by setting ```symbolic``` parameter to ```True```.

[*Return to top*](#zrobotics-01)

---

## Dynamics

[*Return to top*](#zrobotics-01)

---

### Inertia Matrix

[*Return to top*](#zrobotics-01)

---

### Kinetic Energy

[*Return to top*](#zrobotics-01)

---

### Potential Energy

[*Return to top*](#zrobotics-01)

---

### Gravity Actuation

[*Return to top*](#zrobotics-01)

---

### Centrifugal and Coriolis Effects

[*Return to top*](#zrobotics-01)

---

### Robot Model

[*Return to top*](#zrobotics-01)

---

**We hope this can be useful for you. Thank you!**

    
![Z Dynamics](img/icon.png "The Future is ROBOTICS")
