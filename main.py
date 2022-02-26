import numpy as np
from lib.dynamics.DynamicsHTM import *
from lib.kinematics.HTM import *
from lib.kinematics.DifferentialHTM import *
from lib.kinematics.DQ import *
from lib.Robot import *
from sympy import *

if __name__ == '__main__':
  
  """
    1. ROBOT ATTRIBUTES & CONSTRUCTION
  """
  
  # Number of rigid bodies
  m = 2
  
  # Number of Generalized Coordinates
  n = 2
  
  # Generalized coordinates
  q = np.random.randn(n, 1)
  
  # Joints velocities
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
  
  # Links
  L = [np.random.rand() for i in range(m)]
  
  # Center of Mass of each link
  Lcom = [value / 2 for value in L]

  # Mass of each link
  m = [np.random.rand() for i in range(m)]
  
  # Inertia of each link (with respect to the Center of Mass)
  Inertia = [np.random.rand(3, 3) for i in range(n)]

  # Robot initialization as an object
  uRobot = Serial(jointsPositions = q, jointsVelocities = qd, linksLengths = L, COMs = Lcom, mass = m, inertia = Inertia, name = 'uRobot', xi = xi, xid = xid)

  # Set robot's Denavit - Hartenberg Matrix (numerical and symbolic, OPTIONAL)
  uRobot.denavitHartenberg()
  # uRobot.denavitHartenberg(symbolic = True)
  
  # Set robot's Denavit - Hartenberg Matrix for Centers of Mass (numerical and symbolic, OPTIONAL)
  uRobot.denavitHartenbergCOM()
  # uRobot.denavitHartenbergCOM(symbolic = True)

  """
    2. KINEMATICS
  """

  # Robot's forward kinematics using Homogeneous Transformation Matrices (numerical and symbolic)
  fkHTM = forwardHTM(uRobot)
  # symbolicFKHTM = forwardHTM(uRobot, symbolic = True)
  
  # Robot's forward kinematics using Dual Quaternions (numerical and symbolic)
  fkDQ = forwardDQ(uRobot)
  # symbolicFKDQ = forwardDQ(uRobot, symbolic = True)
  
  # Robot's forward kinematics to each center of mass using Homogeneous Transformation Matrices (numerical and symbolic)
  fkCOMHTM = forwardCOMHTM(uRobot)
  # symbolicFKCOMHTM = forwardCOMHTM(uRobot, symbolic = True)
  
  # Robot's forward kinematics to each center of mass using Dual Quaternions (numerical and symbolic)
  fkCOMDQ = forwardCOMDQ(uRobot)
  # symbolicDQCOMHTM = forwardCOMDQ(uRobot, symbolic = True)

  # Axis - Angle vector based on Homogeneous Transformation Matrix obtained by Forward Kinematics calculation
  X = axisAngle(fkHTM[-1])
  # symbolicX = axisAngle(symbolicFKHTM[-1], symbolic = True)
  
  """
    3. JACOBIAN MATRICES
  """
  
  # Geometric Jacobian Matrix (OPTIONAL)
  Jg = geometricJacobian(uRobot)
  # symbolicJg = geometricJacobian(uRobot, symbolic = True)
  
  # Geometric Jacobian Matrix to any Center of Mass (OPTIONAL)
  JgCOM = geometricJacobianCOM(uRobot, COM = 2)
  # symbolicJgCOM = geometricJacobianCOM(uRobot, COM = 1, symbolic = True)
  
  # Analytic Jacobian Matrix (OPTIONAL)
  Ja = analyticJacobian(uRobot)
  # symbolicJa = analyticJacobian(uRobot, symbolic = True)
  
  # Dual Jacobian Matrix (OPTIONAL)
  Jdq = jacobianDQ(uRobot)
  # symbolicJdq = jacobianDQ(uRobot, symbolic = True)
  
  """
    3. INVERSE KINEMATICS
  """ 
  
  # Calculate robot's Inverse Kinematics to a single point (using Homogeneous Transformation Matrices)
  qHTM = inverseHTM(uRobot, q0 = np.random.rand(n, 1), Hd = fkHTM[-1], K = 50 * np.eye(6))
  
  # Calculate robot's Inverse Kinematics to a single point (Dual Quaternions)
  qDQ = inverseDQ(uRobot, q0 = np.random.rand(n, 1), Qd = fkDQ[-1], K = 50 * np.eye(8))
  
  """
    4. DIFFERENTIAL KINEMATICS (Velocities)
  """
  
  # End-effector inertial velocity (using geometric jacobian matrix) with Homogeneous Transformation Matrices
  geometricXd = geometricStateSpace(uRobot)
  # symbolicXd = geometricStateSpace(uRobot, symbolic = True)
  
  # End-effector inertial linear velocity + inertial rate of change of its rotations (using analytic jacobian matrix) with Homogeneous Transformation Matrices
  analyticXd = analyticStateSpace(uRobot)
  # symbolicXd = analyticStateSpace(uRobot, symbolic = True)
  
  # Velocity of each Center of Mass using Geometric Jacobian Matrix
  XdCOM = geometricCOMStateSpace(uRobot, COM = 1)
  # symbolicXdCOM = geometricCOMStateSpace(uRobot, COM = 1, symbolic = True)
  
  """
    5. DYNAMICS
  """
  
  # Inertia Matrix for Kinetic Energy equation: D(q)
  D = inertiaMatrix(uRobot)
  # symbolicD = inertiaMatrix(uRobot, symbolic = True)  
  
  # Test: D(q)
  # d11 = (m[0] * Lcom[0] ** 2) + (m[1] * (L[0] ** 2 + Lcom[1] ** 2 + (2 * L[0] * Lcom[1] * np.cos(q[1])))) + I[0][-1, -1] + I[1][-1, -1]
  # d12 = m[1] * (Lcom[1] ** 2 + (L[0] * Lcom[1] * np.cos(q[1]))) + I[1][-1, -1]
  # d22 = m[1] * Lcom[1] ** 2 + I[1][-1, -1]
  
  # Kinetic Energy of the robot in the Centers of Mass: 0.5 * q'(t)^T * D * q'(t) (OPTIONAL)
  K = kineticEnergyCOM(uRobot)
  # symbolicK = kineticEnergyCOM(uRobot, symbolic = True)
  
  # Potential Energy of the robot in the Centers of Mass: m * g^T * r (OPTIONAL)
  P = potentialEnergyCOM(uRobot, g = np.array([[0], [-9.80665], [0]]))
  # symbolicP = potentialEnergyCOM(uRobot, g = np.array([[0], [-9.80665], [0]]), symbolic = True)
  
  # Lagrange Equation: L = K - P
  # L = K - Matrix(P)
  # symbolicL = symbolicK - symbolicP
  
  # Centrifugal and Coriolis Matrix: C(q, q')
  C = centrifugalCoriolis(uRobot)
  # symbolicC = centrifugalCoriolis(uRobot, symbolic = True)
  
  # Test: C(q, q')
  # c = -m[1] * L[0] * Lcom[1] * np.sin(q[1])
  
  # Derivative of Potential Energy (with respect to "q" or joints positions): G(q)
  G = dPdq(uRobot, g = np.array([[0], [-9.80665], [0]]))
  # symbolicG = dPdq(uRobot, g = np.array([[0], [-9.80665], [0]]), symbolic = True)
  
  # Test: g(q)
  # g1 = (m[0] * (-9.80665) * Lcom[0] * np.cos(q[0])) + (m[1] * (-9.80665) * (L[0] * np.cos(q[0]) + Lcom[1] * np.cos(q[0] + q[1])))
  # g2 = m[1] * (-9.80665) * Lcom[1] * np.cos(q[0] + q[1])
  
  # Robot Dynamic Equation: D(q) * q''(t) + C(q, q') * q'(t) + G(q) = T
  T = (D * uRobot.qddSymbolic) + (C * uRobot.qdSymbolic) + G
  # symbolicT = (symbolicD * uRobot.qddSymbolic) + (symbolicC * uRobot.qdSymbolic) + symbolicG
  
  print("Z")
  # END