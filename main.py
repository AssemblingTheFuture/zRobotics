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
    3. JACOBIAN MATRICES
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
  
  # Analytic Jacobian Matrix (OPTIONAL)
  Ja = analyticJacobian(uRobot)
  # symbolicJa = analyticJacobian(uRobot, symbolic = True)
  
  # Analytic Jacobian Matrix to a Center of Mass(OPTIONAL)
  JaCOM = analyticJacobianCOM(uRobot, COM = 2)
  # symbolicJa = analyticJacobianCOM(uRobot, COM = 1, symbolic = True)
  
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
    4. DIFFERENTIAL KINEMATICS (Velocities and Accelerations using Homogeneous Transformation Matrices)
  """
  
  # End-effector inertial velocity (using geometric jacobian matrix) with Homogeneous Transformation Matrices
  geometricXd = geometricStateSpace(uRobot)
  # symbolicGeometricXd = geometricStateSpace(uRobot, symbolic = True)
  
  # End-effector inertial linear velocity + inertial rate of change of its rotations (using analytic jacobian matrix) with Homogeneous Transformation Matrices
  analyticXd = analyticStateSpace(uRobot)
  # symbolicXd = analyticStateSpace(uRobot, symbolic = True)
  
  # Inertial angular velocity propagation to each reference frame 
  W = angularVelocityPropagation(uRobot, w0 = np.zeros((3, 1)), qd = qd)
  # symbolicW = angularVelocityPropagation(uRobot, w0 = zeros(3, 1), qd = uRobot.qdSymbolic, symbolic = True)
  
  # Inertial linear velocity propagation to each reference frame
  V = linearVelocityPropagation(uRobot, v0 = np.zeros((3, 1)), W = W)
  # symbolicV = linearVelocityPropagation(uRobot, v0 = zeros(3, 1), W = symbolicW, symbolic = True)
  
  # End-effector inertial acceleration (using geometric jacobian matrix and its derivative) with Homogeneous Transformation Matrices
  geometricXdd = geometricDerivativeStateSpace(uRobot)
  # symbolicGeometricXdd = geometricDerivativeStateSpace(uRobot, symbolic = True)
  
  # Inertial angular acceleration propagation to each reference frame
  dW = angularAccelerationPropagation(uRobot, dw0 = np.zeros((3, 1)), W = W, qd = qd, qdd = qdd)
  # symbolicdW = angularAccelerationPropagation(uRobot, dw0 = zeros(3, 1), W = symbolicW, qd = uRobot.qdSymbolic, qdd = uRobot.qddSymbolic, symbolic = True)
  
  # Inertial linear acceleration propagation to each reference frame attached to joints
  dV = linearAccelerationPropagation(uRobot, dv0 = np.array([[0], [0], [-9.80665]]), W = W, dW = dW)
  # symbolicdV = linearAccelerationPropagation(uRobot, dv0 = Matrix([[0], [0], [-9.80665]]), W = symbolicW, dW = symbolicdW, symbolic = True)
  
  # Inertial Velocity of each Center of Mass using Geometric Jacobian Matrix
  geometricXdCOM = geometricCOMStateSpace(uRobot, COM = 2)
  # symbolicGeometricXdCOM = geometricCOMStateSpace(uRobot, COM = 2, symbolic = True)
  
  # Inertial Velocity of each Center of Mass using Analytic Jacobian Matrix
  analyticXdCOM = analyticCOMStateSpace(uRobot, COM = 2)
  # symbolicAnalyticXdCOM = analyticCOMStateSpace(uRobot, COM = 2, symbolic = True)
  
  # Inertial angular velocity propagation to each center of mass
  Wcom = angularVelocityPropagationCOM(uRobot, wCOM0 = np.zeros((3, 1)), W = W, qd = qd)
  # symbolicWcom = angularVelocityPropagationCOM(uRobot, wCOM0 = zeros(3, 1), W = symbolicW, qd = uRobot.qdSymbolic, symbolic = True)
  
  # Inertial linear velocity propagation to each center of mass
  Vcom = linearVelocityPropagationCOM(uRobot, vCOM0 = np.zeros((3, 1)), Wcom  = Wcom, V = V)
  # symbolicVcom = linearVelocityPropagationCOM(uRobot, vCOM0 = zeros(3, 1), Wcom  = symbolicWcom, V = symbolicV, symbolic = True)
  
  # Inertial Acceleration of each Center of Mass using Geometric Jacobian Matrix and its derivative
  geometricXddCOM = geometricCOMDerivativeStateSpace(uRobot, COM = 2)
  # symbolicGeometricXddCOM = geometricCOMDerivativeStateSpace(uRobot, COM = 1, symbolic = True)
  
  # Inertial angular acceleration propagation to each reference frame
  dWcom = angularAccelerationPropagationCOM(uRobot, dwCOM0 = np.zeros((3, 1)), Wcom = Wcom, dW = dW, qd = qd, qdd = qdd)
  # symbolicdWcom = angularAccelerationPropagationCOM(uRobot, dwCOM0 = zeros(3, 1), Wcom = symbolicWcom, dW = symbolicdW, qd = uRobot.qdSymbolic, qdd = uRobot.qddSymbolic, symbolic = True)
  
  # Inertial linear acceleration propagation to each center of mass
  dVcom = linearAccelerationPropagationCOM(uRobot, dvCOM0 = np.array([[0], [0], [-9.80665]]), Wcom = Wcom, dWcom = dWcom, dV = dV)
  # symbolicdVcom = linearAccelerationPropagationCOM(uRobot, dvCOM0 = Matrix([[0], [0], [-9.80665]]), Wcom = symbolicWcom, dWcom = symbolicdWcom, dV = symbolicdV, symbolic = True)
  
  """
    4.1 DIFFERENTIAL KINEMATICS (Velocities and Accelerations using Dual Quaternions)
  """

  # Inertial velocity propagation using Dual Quaternions
  Wdq = dqVelocityPropagation(uRobot, w0 = np.zeros((8, 1)), qd = qd)
  # symbolicWdq = dqVelocityPropagation(uRobot, w0 = zeros(8, 1), qd = uRobot.qdSymbolic, symbolic = True)
  
  # Inertial acceleration propagation using Dual Quaternions
  dWdq = dqAccelerationPropagation(uRobot, dw0 = np.array([[0], [0], [0], [0], [0], [0], [0], [-9.80665]]), Wdq = Wdq, qd = qd, qdd = qdd)
  # symbolicdWdq = dqAccelerationPropagation(uRobot, dw0 = Matrix([[0], [0], [0], [0], [0], [0], [0], [-9.80665]]), Wdq = symbolicWdq, qd = uRobot.qdSymbolic, qdd = uRobot.qddSymbolic, symbolic = True)
  
  # Inertial velocity propagation to each center of mass using Dual Quaternions
  WdqCOM = dqVelocityPropagationCOM(uRobot, WdqCOM0 = np.zeros((8, 1)), Wdq = Wdq, qd = qd)
  # symbolicWdqCOM = dqVelocityPropagationCOM(uRobot, WdqCOM0 = zeros(8, 1), Wdq = symbolicWdq, qd = uRobot.qdSymbolic, symbolic = True)
  
  # Inertial acceleration propagation to each center of mass using Dual Quaternions
  dWdqCOM = dqAccelerationPropagationCOM(uRobot, dWdqCOM0 = np.array([[0], [0], [0], [0], [0], [0], [0], [-9.80665]]), Wdq = Wdq, WdqCOM = WdqCOM, dWdq = dWdq, qd = qd, qdd = qdd)
  # symbolicdWdqCOM = dqAccelerationPropagationCOM(uRobot, dWdqCOM0 = Matrix([[0], [0], [0], [0], [0], [0], [0], [-9.80665]]), Wdq = symbolicWdq, WdqCOM = symbolicWdqCOM, dWdq = symbolicdWdq, qd = uRobot.qdSymbolic, qdd = uRobot.qddSymbolic, symbolic = True)
  
  """
    5. DYNAMICS
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
  G = dPdqCOM(uRobot)
  # symbolicG = dPdqCOM(uRobot, symbolic = True)
  
  # Robot Dynamic Equation: D(q) * q''(t) + C(q, q') * q'(t) + G(q) = T
  T = (D * uRobot.qddSymbolic) + (C.dot(uRobot.jointsVelocities)) + G
  # symbolicT = (symbolicD * uRobot.qddSymbolic) + (symbolicC * uRobot.qdSymbolic) + symbolicG
  
  # Force - Torque propagation (Newton - Euler Recursive Algorithm)
  Wrench = newtonEuler(uRobot, dv0 = np.array([[0], [0], [-9.80665]]), f0 = np.array([[0], [0], [-7]]), T0 = np.array([[0], [0], [-7]]))
  # symbolicWrench = newtonEuler(uRobot, w0 = np.zeros((3, 1)), dw0 = np.zeros((3, 1)), dv0 = np.array([[0], [0], [-9.80665]]), f0 = np.array([[0], [0], [-7]]), T0 = np.array([[0], [0], [-7]]), symbolic = True)
  
  print("Z")
  # END