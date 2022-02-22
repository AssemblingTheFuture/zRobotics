import numpy as np
from lib.Robot import *
from lib.kinematics.HTM import *
from lib.kinematics.DifferentialHTM import *
from lib.kinematics.DQ import *
from sympy import *

if __name__ == '__main__':
  
  """
    1. ROBOT ATTRIBUTES & CONSTRUCTION
  """
  
  # Generalized coordinates
  q = np.random.rand(4, 1)
  
  # Links
  L = [0.3, 0.5, 0.4]
  
  # Centers of Mass of each link
  Lcom = [0.15, 0.25, 0.2]

  # Robot initialization as an object
  uRobot = Serial(jointsPositions = q, linksLengths = L, COMs = Lcom, name = 'uRobot')

  # Set robot's Denavit - Hartenberg Matrix (numerical and symbolic)
  uRobot.denavitHartenberg()
  # uRobot.denavitHartenberg(symbolic = True)
  
  # Set robot's Denavit - Hartenberg Matrix for Centers of Mass (numerical and symbolic)
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
  # symbolicJg = geometricJacobian(uRobot, symbolic = True)
  
  # Analytic Jacobian Matrix (OPTIONAL)
  Ja = analyticJacobian(uRobot)
  # symbolicJa = analyticJacobian(uRobot, symbolic = True)
  
  # Dual Jacobian Matrix (OPTIONAL)
  Jdq = jacobianDQ(uRobot, xi)
  # symbolicJdq = jacobianDQ(uRobot, xi, symbolic = True)
  
  """
    3. INVERSE KINEMATICS
  """ 
  
  # Calculate robot's Inverse Kinematics to a single point (using Homogeneous Transformation Matrices)
  qHTM = inverseHTM(uRobot, q0 = np.random.rand(4, 1), Hd = fkHTM[-1], K = 50 * np.eye(6))
  
  # Calculate robot's Inverse Kinematics to a single point (Dual Quaternions)
  qDQ = inverseDQ(uRobot, q0 = np.random.rand(4, 1), Qd = fkDQ[-1], K = 50 * np.eye(8), xi = xi)
  
  """
    4. DIFFERENTIAL KINEMATICS (Velocities)
  """
  
  # Joints velocities
  qd = np.random.rand(4, 1)
  
  # End-effector inertial velocity (using geometric jacobian matrix) with Homogeneous Transformation Matrices
  geometricXd = geometricStateSpace(uRobot, qd)
  # symbolicXd = geometricalStateSpace(uRobot, qd, symbolic = True)
  
  # End-effector inertial linear velocity + inertial rate of change of its rotations (using analytic jacobian matrix) with Homogeneous Transformation Matrices
  analyticXd = analyticStateSpace(uRobot, qd)
  # symbolicXd = analyticalStateSpace(uRobot, qd, symbolic = True)
  
  print("Z")
  # END