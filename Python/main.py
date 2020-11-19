import DenavitHartenberg as dh
import Kinematics as k
import Movements as mv
import numpy as np
import Plot as plot
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
  6.1 Set desired pose representation (mandatory). We will use «fkHTM» and «fkDQ» as desired pose representations
""" 

"""
  6.2 Computes robot's Jacobian Matrix (using Homogeneous Transformation Matrices or Dual Quaternions)
"""

# Screw vectors stored in a matrix
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
  7. Animate robot with joints' positions (this also modifies them in the object). Uncomment any of these:
"""
# plot.animation(uRobot, q = qHTM, repeatAnimation = False, delayPerFrame = 1)
# plot.animation(uRobot, q = qDQ, repeatAnimation = False, delayPerFrame = 1)

"""
  7.1 Plot any robot's behavior (joint's positions, end - effector, dynamics, etc)
"""
plot.graph(function = qHTM, title = "Joints' Positions (using HTM)", labels = r'$\theta_', complement = r'(k)$', xlabel = r'Samples $k$ [3 $\frac{ms}{sample}$]', ylabel = r'Amplitude [$rad$]', save = True, name = "qHTM", transparent = True)
plot.graph(function = qDQ, title = "Joints' Positions (using DQ)", labels = r'$\theta_', complement = r'(k)$', xlabel = r'Samples $k$ [3 $\frac{ms}{sample}$]', ylabel = r'Amplitude [$rad$]', save = True, name = "qDQ", transparent = True)

"""
  8. Differential Kinematics
"""

"""
  8.1 Computes robot's Inertial Velocity Jacobian Matrix (using Dual Quaternions, Homogeneous Transformation Matrix does not need a different Jacobian)
"""
Jvdq = k.jacobianVDQ(uRobot, n = 4, xi = xi)

"""
  8.2 Computes Instantaneous Inertial Velocity to m - th frame
"""
Vhtm = k.velocityHTM(uRobot, m = 5, qd = np.random.rand(4, 1))
Vdq = k.velocityDQ(uRobot, m = 5, n = 4, qd = np.random.rand(4, 1), xi = xi)

"""
  8.3 Computes Instantaneous Joints' Velocities given m - th frame one
"""
qdHTM = k.jointsVelocitiesHTM(uRobot, m = 5, Vhtm = Vhtm)
qdDQ = k.jointsVelocitiesDQ(uRobot, m = 5, n = 4, Vdq = Vdq, xi = xi)

print("Z")
# END