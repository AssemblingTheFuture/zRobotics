import numpy as np
from lib.dynamics.DynamicsHTM import *
from lib.kinematics.HTM import *
from lib.kinematics.DifferentialHTM import *
from lib.kinematics.DQ import *
from lib.kinematics.DifferentialDQ import *
from lib.Robot import *
from sympy import *

if __name__ == '__main__':
  
  """
    1. ROBOT ATTRIBUTES & CONSTRUCTION
  """
  
  # Number of rigid bodies
  rb = 3
  
  # Number of Generalized Coordinates
  n = 4
  
  # Generalized coordinates
  q = np.random.randn(n, 1)
  
  # Joints velocities
  qd = np.random.randn(n, 1)
  
  # Joints accelerations
  qdd = np.random.randn(n, 1)
  
  # Screw vectors (or axes of actuation) stored in a matrix. This is MANDATORY for calculations using Dual Quaternions
  xi = np.array([[0, 0, 0, 0],
                 [0, 0, 0, 0],
                 [0, 0, 0, 0],
                 [1, 1, 1, 1],
                 [0, 0, 0, 0],
                 [0, 0, 0, 0],
                 [0, 0, 0, 0],
                 [0, 0, 0, 0]])
  
  # Derivative of previous screw vectors (or axes of actuation) stored in a matrix. This is MANDATORY for calculations using Dual Quaternions
  xid = np.zeros((8, n))
  
  # Links
  L = [np.random.rand() for body in range(rb)]
  
  # Center of Mass of each link
  Lcom = [value / 2 for value in L]

  # Mass of each link
  m = [np.random.rand() for i in range(rb)]
  
  # Inertia of each link (with respect to the Center of Mass; it has to be a SYMMETRIC matrix)
  Inertia = np.random.rand(3, 3)
  Inertia = [0.5 * (Inertia + Inertia.T) for i in range(rb)]

  # Robot initialization as an object
  uRobot = Serial(jointsPositions = q, jointsVelocities = qd, jointsAccelerations = qdd, linksLengths = L, COMs = Lcom, mass = m, inertia = Inertia, name = 'uRobot', xi = xi, xid = xid)

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
  # symbolicCOMDQ = forwardCOMDQ(uRobot, symbolic = True)

  # Axis - Angle vector based on Homogeneous Transformation Matrix obtained by Forward Kinematics calculation
  X = axisAngle(fkHTM[-1])
  # symbolicX = axisAngle(symbolicFKHTM[-1], symbolic = True)
  
  """
    3. JACOBIAN MATRICES (USING HOMOGENEOUS TRANSFORMATION MATRICES)
  """
  
  # Geometric Jacobian Matrix (OPTIONAL)
  Jg = geometricJacobian(uRobot)
  # symbolicJg = geometricJacobian(uRobot, symbolic = True)
  
  # Derivative of Geometric Jacobian Matrix (OPTIONAL)
  dJg = geometricJacobianDerivative(uRobot)
  # symbolicdJg = geometricJacobianDerivative(uRobot, symbolic = True)
   
  # Geometric Jacobian Matrix to any Center of Mass (OPTIONAL)
  JgCOM = geometricJacobianCOM(uRobot, COM = 2)
  # symbolicJgCOM = geometricJacobianCOM(uRobot, COM = 2, symbolic = True)
  
  # Derivative of Geometric Jacobian Matrix to any acenter of mass (OPTIONAL)
  dJgCOM = geometricJacobianDerivativeCOM(uRobot, COM = 2)
  # symbolicdJgCOM = geometricJacobianDerivativeCOMz(uRobot, COM = 2, symbolic = True)
  
  # Analytic Jacobian Matrix (OPTIONAL)
  Ja = analyticJacobian(uRobot)
  # symbolicJa = analyticJacobian(uRobot, symbolic = True)
  
  # Analytic Jacobian Matrix to a Center of Mass(OPTIONAL)
  JaCOM = analyticJacobianCOM(uRobot, COM = 2)
  # symbolicJa = analyticJacobianCOM(uRobot, COM = 1, symbolic = True)
  
  """
    3.1 JACOBIAN MATRICES (USING DUAL QUATERNIONS)
  """
  
  # Dual Jacobian Matrix (OPTIONAL)
  Jdq = jacobianDQ(uRobot)
  # symbolicJdq = jacobianDQ(uRobot, symbolic = True)
  
  # Dual Inertial Velocity Jacobian Matrix (OPTIONAL)
  Jvdq = jacobianVelocityDQ(uRobot)
  # symbolicJvdq = jacobianVelocityDQ(uRobot, symbolic = True)
  
  # Geometric Jacobian Matrix to any Center of Mass (OPTIONAL)
  JvdqCOM = jacobianVelocityDQCOM(uRobot, COM = 2)
  # symbolicJvdqCOM = jacobianVelocityDQCOM(uRobot, COM = 2, symbolic = True)
  
  """
    4. INVERSE KINEMATICS
  """ 
  
  # Calculate robot's Inverse Kinematics to a single point (using Homogeneous Transformation Matrices)
  qHTM = inverseHTM(uRobot, q0 = np.random.rand(n, 1), Hd = fkHTM[-1], K = 50 * np.eye(6))
  
  # Calculate robot's Inverse Kinematics to a single point (Dual Quaternions)
  qDQ = inverseDQ(uRobot, q0 = np.random.rand(n, 1), Qd = fkDQ[-1], K = 50 * np.eye(8))
  
  """
    5. DIFFERENTIAL KINEMATICS (Velocities and Accelerations using Homogeneous Transformation Matrices)
  """
  
  # End-effector inertial velocity (using geometric jacobian matrix) with Homogeneous Transformation Matrices
  geometricXd = geometricStateSpace(uRobot)
  # symbolicGeometricXd = geometricStateSpace(uRobot, symbolic = True)
  
  # End-effector inertial linear velocity + inertial rate of change of its rotations (using analytic jacobian matrix) with Homogeneous Transformation Matrices
  analyticXd = analyticStateSpace(uRobot)
  # symbolicXd = analyticStateSpace(uRobot, symbolic = True)
  
  # Inertial velocity propagation to each reference frame 
  V = velocityPropagation(uRobot, v0 = np.zeros((3, 1)), w0 = np.zeros((3, 1)), qd = qd)
  # symbolicV = velocityPropagation(uRobot, v0 = zeros(3, 1), w0 = zeros(3, 1), qd = uRobot.qdSymbolic, symbolic = True)
    
  # End-effector inertial acceleration (using geometric jacobian matrix and its derivative) with Homogeneous Transformation Matrices
  geometricXdd = geometricDerivativeStateSpace(uRobot)
  # symbolicGeometricXdd = geometricDerivativeStateSpace(uRobot, symbolic = True)
  
  # Inertial angular acceleration propagation to each reference frame
  dV = accelerationPropagation(uRobot, dv0 = np.zeros((3, 1)), dw0 = np.zeros((3, 1)), V = V, qd = qd, qdd = qdd)
  # symbolicdV = accelerationPropagation(uRobot, dv0 = zeros(3, 1), dw0 = zeros(3, 1), V = symbolicV, qd = uRobot.qdSymbolic, qdd = uRobot.qddSymbolic, symbolic = True)
  
  """
    5.1 DIFFERENTIAL KINEMATICS TO EACH CENTER OF MASS (Velocities and Accelerations using Homogeneous Transformation Matrices)
  """
  
  # Inertial Velocity of each Center of Mass using Geometric Jacobian Matrix
  geometricXdCOM = geometricCOMStateSpace(uRobot, COM = 2)
  # symbolicGeometricXdCOM = geometricCOMStateSpace(uRobot, COM = 2, symbolic = True)
  
  # Inertial Velocity of each Center of Mass using Analytic Jacobian Matrix
  analyticXdCOM = analyticCOMStateSpace(uRobot, COM = 2)
  # symbolicAnalyticXdCOM = analyticCOMStateSpace(uRobot, COM = 2, symbolic = True)
  
  # Inertial velocity propagation to each center of mass
  Vcom = velocityPropagationCOM(uRobot, vCOM0 = np.zeros((3, 1)), wCOM0 = np.zeros((3, 1)), V = V, qd = qd)
  # symbolicVcom = velocityPropagationCOM(uRobot, vCOM0 = zeros(3, 1), wCOM0 = zeros(3, 1), V = symbolicV, qd = uRobot.qdSymbolic, symbolic = True)
  
  # Inertial Acceleration of each Center of Mass using Geometric Jacobian Matrix and its derivative
  geometricXddCOM = geometricCOMDerivativeStateSpace(uRobot, COM = 2)
  # symbolicGeometricXddCOM = geometricCOMDerivativeStateSpace(uRobot, qd = uRobot.qdSymbolic, COM = 2, symbolic = True)
  
  # Inertial angular acceleration propagation to each reference frame
  dVcom = accelerationPropagationCOM(uRobot, dvCOM0 = np.zeros((3, 1)), dwCOM0 = np.zeros((3, 1)), Vcom = Vcom, dV = dV, qd = qd, qdd = qdd)
  # symbolicdVcom = accelerationPropagationCOM(uRobot, dvCOM0 = zeros(3, 1), dwCOM0 = np.zeros(3, 1), Vcom = symbolicVcom, dV = symbolicdV, qd = uRobot.qd, qdd = uRobot.qddSymbolic, symbolic = True)
  
  """
    5.2 DIFFERENTIAL KINEMATICS (Velocities and Accelerations using Dual Quaternions)
  """

  # End-effector inertial velocity (using dual inertial velocity jacobian matrix) with Dual Quaternions
  dqXd = dqStateSpace(uRobot)
  # symbolicdqXd = dqStateSpace(uRobot, symbolic = True)

  # Inertial velocity propagation using Dual Quaternions
  Wdq = dqVelocityPropagation(uRobot, w0 = np.zeros((8, 1)), qd = qd)
  # symbolicWdq = dqVelocityPropagation(uRobot, w0 = zeros(8, 1), qd = uRobot.qdSymbolic, symbolic = True)
  
  # Inertial acceleration propagation using Dual Quaternions
  dWdq = dqAccelerationPropagation(uRobot, dw0 = np.zeros((8, 1)), Wdq = Wdq, qd = qd, qdd = qdd)
  # symbolicdWdq = dqAccelerationPropagation(uRobot, dw0 = zeros(8, 1), Wdq = symbolicWdq, qd = uRobot.qdSymbolic, qdd = uRobot.qddSymbolic, symbolic = True)
  
  """
    5.1 DIFFERENTIAL KINEMATICS TO EACH CENTER OF MASS (Velocities and Accelerations using Dual Quaternions)
  """
  
  # Inertial Velocity of each Center of Mass using Dual Inertial Velocity Jacobian Matrix
  dqXdCOM = dqStateSpaceCOM(uRobot, COM = 2)
  # symbolicdqXdCOM = dqStateSpaceCOM(uRobot, COM = 2, symbolic = True)
  
  # Inertial velocity propagation to each center of mass using Dual Quaternions
  WdqCOM = dqVelocityPropagationCOM(uRobot, WdqCOM0 = np.zeros((8, 1)), Wdq = Wdq, qd = qd)
  # symbolicWdqCOM = dqVelocityPropagationCOM(uRobot, WdqCOM0 = zeros(8, 1), Wdq = symbolicWdq, qd = uRobot.qdSymbolic, symbolic = True)
  
  # Inertial acceleration propagation to each center of mass using Dual Quaternions
  dWdqCOM = dqAccelerationPropagationCOM(uRobot, dWdqCOM0 = np.zeros((8, 1)), Wdq = Wdq, WdqCOM = WdqCOM, dWdq = dWdq, qd = qd, qdd = qdd)
  # symbolicdWdqCOM = dqAccelerationPropagationCOM(uRobot, dWdqCOM0 = zeros(8, 1), Wdq = symbolicWdq, WdqCOM = symbolicWdqCOM, dWdq = symbolicdWdq, qd = uRobot.qdSymbolic, qdd = uRobot.qddSymbolic, symbolic = True)
  
  """
    6. DYNAMICS
  """
  
  # Inertia Matrix for Kinetic Energy equation: D(q)
  D = inertiaMatrixCOM(uRobot)
  # symbolicD = inertiaMatrixCOM(uRobot, symbolic = True)
  
  # Kinetic Energy of the robot in the Centers of Mass: 0.5 * q'(t)^T * D * q'(t) (OPTIONAL)
  K = kineticEnergyCOM(uRobot)
  # symbolicK = kineticEnergyCOM(uRobot, symbolic = True)
  
  # Potential Energy of the robot in the Centers of Mass: m * g^T * r (OPTIONAL)
  P = potentialEnergyCOM(uRobot)
  # symbolicP = potentialEnergyCOM(uRobot, symbolic = True)
  
  # Centrifugal and Coriolis Matrix: C(q, q')
  C = centrifugalCoriolisCOM(uRobot)
  # symbolicC = centrifugalCoriolisCOM(uRobot, symbolic = True)
  
  # Derivative of Potential Energy (with respect to "q" or joints positions): G(q)
  G = gravitationalCOM(uRobot, g = np.array([[0], [0], [-9.80665]]))
  # symbolicG = dPdqCOM(uRobot, g = Matrix([[0], [0], ['-g']]), symbolic = True)
  
  # Robot Dynamic Equation: D(q) * q''(t) + C(q, q') * q'(t) + G(q) = T
  T = (D * uRobot.qddSymbolic) + (C.dot(uRobot.jointsVelocities)) + G
  # symbolicT = (symbolicD * uRobot.qddSymbolic) + (symbolicC * uRobot.qdSymbolic) + symbolicG
  
  print("Z")
  # END