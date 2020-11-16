import DenavitHartenberg as dh
import Kinematics as k
import Movements as mv
import numpy as np
import Robot

"""
  1. Creates robot's generalized coordinates (as two dimensional array) and links' lengths
"""
q = np.random.rand(4, 1)
L = [0.3, 0.4, 0.2]

"""
  2. Creates robot as an object
"""
uRobot = Robot.System(jointsPositions = q, linksLengths = L, name = 'uRobot')

"""
  3. Gets robots' Denavit - Hartenberg parameters
"""
DH = dh.matrix(uRobot)

"""
  4. Computes robot's forward kinematics (using Homogeneous Transformation Matrices or Dual Quaternions)
"""
framesHTM, fkHTM = k.forwardHTM(uRobot, m = 5)
framesDQ, fkDQ = k.forwardDQ(uRobot, m = 5)

"""
  5. Compute Axis - Angle vector using Homogeneous Transformation Matrices (if necessary; this is OPTIONAL)
""" 
X = mv.axisAngle(fkHTM)

"""
  6. Compute Inverse Kinematics
""" 

"""
  6.1 Set desired pose representation (mandatory)
""" 

"""
  6.2 Computes robot's Jacobian Matrix (using Homogeneous Transformation Matrices or Dual Quaternions)
"""
xi = np.array([[0, 0, 0, 0],
               [0, 0, 0, 0],
               [0, 0, 0, 0],
               [1, 1, 1, 1],
               [0, 0, 0, 0],
               [0, 0, 0, 0],
               [0, 0, 0, 0],
               [0, 0, 0, 0]])

Jhtm = k.jacobianHTM(uRobot, m = 5)

Jdq = k.jacobianDQ(uRobot, m = 5, xi = xi)

""" 
  6.3 Computes robot's Inverse Kinematics (using Homogeneous Transformation Matrices or Dual Quaternions)
"""
qHTM = k.inverseHTM(uRobot, q0 = np.random.rand(4, 1), Hd = fkHTM, K = np.eye(6), m = 5)

qDQ = k.inverseDQ(uRobot, q0 = np.random.rand(4, 1), Qd = fkDQ, K = np.eye(8), xi = xi, m = 5)

"""
  7. Plot robot (uncomment any of these)
"""

"""
  7.1 Plot robot with new joints' positions (this also modifies them in the object)
"""
# uRobot.plot(q = qHTM, repeatAnimation = True, delayPerFrame = 1)
# uRobot.plot(q = qDQ, repeatAnimation = True, delayPerFrame = 1)
