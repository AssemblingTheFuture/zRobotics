import DenavitHartenberg as dh
import DualQuaternions as dq
import Dynamics as dy
import Movements as mv
import numpy as np
from sympy import *

"""
  Kinematics Section
"""

def forwardHTM(robot, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes forward kinematics to m - th rigid body given joints positions in radians. Robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): _description_
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    framesHTM (list): Homogeneous Transformation Matrices with frames' poses
  """

  # Initial conditions
  framesHTM = []

  # Get Denavit - Hartenberg Matrix
  DH = dh.symbolicMatrix(robot) if symbolic else dh.matrix(robot)

  # Create Homogeneous Transformation Matrix for Inertial Frame
  fkHTM = eye(4) if symbolic else np.identity(4)

  # Iteration through all the rows in Denavit - Hartenberg Matrix
  for frame in DH:
    
    # Operates matrices symbolically: Rz * Tz * Tx * Rx
    fkHTM = nsimplify(simplify(fkHTM * mv.symbolicRz(frame[0]) * mv.symbolicTz(frame[1]) * mv.symbolicTx(frame[2]) * mv.symbolicRx(frame[3])), tolerance=1e-10, rational = False) if symbolic else fkHTM.dot(mv.rz(frame[0]).dot(mv.tz(frame[1])).dot(mv.tx(frame[2])).dot(mv.rx(frame[3])))

    # Append each calculated Homogeneous Transformation Matrix
    framesHTM.append(fkHTM)

  return framesHTM

def forwardCOMHTM(robot, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes forward kinematics to m - th center of mass, given joints positions in radians. Robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): _description_
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    framesHTM (list): Homogeneous Transformation Matrices with COMs' poses
  """
    
  # Calculate forward kinematics
  framesHTM = forwardHTM(robot, symbolic = symbolic)

  # Initial conditions
  framesCOMHTM = [eye(4) if symbolic else np.identity(4)]

  # Get symbolic Denavit - Hartenberg COM Matrix
  comDH = dh.symbolicCentersOfMass(robot) if symbolic else dh.centersOfMass(robot)
  
  # Iterator
  i = 1
    
  # Iteration through all the rows in Denavit - Hartenberg Matrix
  for frame in comDH[1:, :]:
          
    # If calculation is symbolic
    if symbolic:
            
      # Center of Mass Homogeneous Transformation Matrix (symbolic)
      COM = mv.symbolicRz(frame[0]) * mv.symbolicTz(frame[1]) * mv.symbolicTx(frame[2]) * mv.symbolicRx(frame[3])
        
      # Rigid body's Homogeneous Transformation Matrix (symbolic)
      B = framesHTM[i - 1].inv() * framesHTM[i]

      # Forward kinematics to Center of Mass (symbolic)
      fkCOMHTM = nsimplify(simplify(framesHTM[i] * B.inv() * COM), tolerance=1e-10, rational = False)
      
    # Else, calculation is numerical
    else:
            
      # Center of Mass Homogeneous Transformation Matrix
      COM = mv.rz(frame[0]).dot(mv.tz(frame[1])).dot(mv.tx(frame[2])).dot(mv.rx(frame[3]))
       
      # Rigid body's Homogeneous Transformation Matrix
      B = np.linalg.pinv(framesHTM[i - 1]).dot(framesHTM[i])

      # Forward kinematics to Center of Mass
      fkCOMHTM = framesHTM[i].dot(np.linalg.inv(B)).dot(COM)
        
    framesCOMHTM.append(fkCOMHTM)
    i += 1
    
  return framesCOMHTM

def forwardDQ(robot, symbolic = False):
  """Using Dual Quaternions, this function computes forward kinematics to m - th rigid body given joints positions in radians. Robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): _description_
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    framesDQ (list): Dual Quaternions with frames' poses
  """
  
  # Initial conditions
  framesDQ = []

  # Get Denavit - Hartenberg Matrix
  DH = dh.symbolicMatrix(robot) if symbolic else dh.matrix(robot)

  # Create Dual Quaternion for Inertial Frame
  fkDQ = Matrix([1, 0, 0, 0, 0, 0, 0, 0]) if symbolic else np.array([[1, 0, 0, 0, 0, 0, 0, 0]]).T
  
  # Iteration through all the rows in Denavit - Hartenberg Matrix
  for frame in DH:
    
    # Operates dual quaternions symbolically: Rz * Tz * Tx * Rx
    fkDQ = nsimplify(simplify(dq.symbolicLeftOperator(fkDQ) * dq.symbolicRightOperator(dq.symbolicRx(frame[3])) * dq.symbolicLeftOperator(dq.symbolicRz(frame[0])) * dq.symbolicRightOperator(dq.symbolicTx(frame[2])) * dq.symbolicTz(frame[1])), tolerance=1e-10, rational = False) if symbolic else dq.leftOperator(fkDQ).dot(dq.rightOperator(dq.Rx(frame[3]))).dot(dq.leftOperator(dq.Rz(frame[0]))).dot(dq.rightOperator(dq.Tx(frame[2]))).dot(dq.Tz(frame[1]))

    # Append each calculated Homogeneous Transformation Matrix
    framesDQ.append(fkDQ)
    
  return framesDQ

def forwardCOMDQ(robot, symbolic = False):
  """Using Dual Quaternions, this function computes forward kinematics to m - th center of mass, given joints positions in radians. Robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): _description_
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    framesHTM (list): Dual Quaternions with COMs' poses
  """
  
  # Calculate forward kinematics
  framesDQ = forwardDQ(robot, symbolic = symbolic)

  # Initial conditions
  framesCOMDQ = Matrix([1, 0, 0, 0, 0, 0, 0, 0]) if symbolic else [np.array([[1], [0], [0], [0], [0], [0], [0], [0]])]

  # Get Denavit - Hartenberg COM Matrix
  comDH = dh.symbolicCentersOfMass(robot) if symbolic else dh.centersOfMass(robot)

  # Iterator
  i = 1
    
  # Iteration through all the rows in Denavit - Hartenberg Matrix
  for frame in comDH[1:, :]:
          
    # If calculation is symbolic
    if symbolic:
      
      # Center of Mass Homogeneous Transformation Matrix
      COM = dq.symbolicLeftOperator(dq.symbolicRz(frame[0])) * dq.symbolicRightOperator(dq.symbolicRx(frame[3])) * dq.symbolicRightOperator(dq.symbolicTx(frame[2])) * dq.symbolicTz(frame[1])

      # Rigid body's Dual Quaternion
      B = dq.symbolicLeftOperator(dq.symbolicConjugate(framesDQ[i - 1])) * framesDQ[i]

      # Forward kinematics to Center of Mass
      fkCOMDQ = nsimplify(simplify(dq.symbolicLeftOperator(framesDQ[i]) * dq.symbolicRightOperator(COM) * dq.symbolicConjugate(B)), tolerance=1e-10, rational = False)

    # Else, calculation is numerical  
    else:
      
      # Center of Mass Dual Quaternion
      COM = dq.leftOperator(dq.Rz(frame[0])).dot(dq.rightOperator(dq.Rx(frame[3]))).dot(dq.rightOperator(dq.Tx(frame[2]))).dot(dq.Tz(frame[1]))
        
      # Rigid body's Dual Quaternion
      B = dq.leftOperator(dq.conjugate(framesDQ[i - 1])).dot(framesDQ[i])
        
      # Forward kinematics to Center of Mass
      fkCOMDQ = dq.leftOperator(framesDQ[i]).dot(dq.rightOperator(COM)).dot(dq.conjugate(B))

    framesCOMDQ.append(fkCOMDQ)
    i += 1
      
  return framesCOMDQ

def geometricJacobian(robot, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes Jacobian matrix to m - th rigid body given joints positions in radians. Robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): _description_
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    J (np.array): Inertial Geometric Jacobian Matrix
    J (SymPy Matrix): Inertial Geometric Jacobian Matrix
  """

  # Get number of joints (generalized coordinates)
  n = robot.jointsPositions.shape[0]
  
  # Calculate forward kinematics
  fkHTM = forwardHTM(robot, symbolic = symbolic)
  
  # Initializes jacobian matrix with zeros
  J = zeros(6, n) if symbolic else np.zeros((6, n))
  
  # Iterates through all colums (generalized coordinates)
  for j in range(n):
        
    # Get axis of rotation of current joint
    z = fkHTM[j][0: 3, 2]
      
    # Calculate distance between end - effector and current joint
    r = fkHTM[-1][0: 3, 3] - fkHTM[j][0: 3, 3]
    
    # Calculate axes of actuation of end - effector caused by current joint
    J[0: 3, j] = nsimplify(z.cross(r), tolerance=1e-10, rational = False) if symbolic else np.cross(z, r)
    
    # Set axis of actuation
    J[3: 6, j] = z
    
  return J

def jacobianDQ(robot, xi, symbolic = False):
  """Using Dual Quaternions, this function computes Jacobian matrix to m - th rigid body given joints positions in radians. Robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): _description_
    xi (np.array): axes of rotation of each joint
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    J (np.array): Inertial Jacobian Matrix
    J (SymPy Matrix): Inertial Jacobian Matrix
  """

  # Get number of joints (generalized coordinates)
  n = robot.jointsPositions.shape[0]
  
  # Calculate forward kinematics
  fkDQ = forwardDQ(robot, symbolic = symbolic)
  
  # Initializes jacobian matrix with zeros
  J = zeros(8, n) if symbolic else np.zeros((8, n))
  
  # Iterates through all generalized coordinates
  for j in range(n):
        
    # Calculate Jacobian Matrix
      J[:, j] = 0.5 * dq.symbolicLeftOperator(fkDQ[j]) * dq.symbolicRightOperator(fkDQ[-1]) * dq.symbolicRightOperator(dq.symbolicConjugate(fkDQ[j])) * xi[:, j] if symbolic else 0.5 * dq.leftOperator(fkDQ[j]).dot(dq.rightOperator(fkDQ[-1])).dot(dq.rightOperator(dq.conjugate(fkDQ[j]))).dot(xi[:, j])
  
  return J


def dotJacobianDQ(robot, m, W, xi, xid):
    """
      Using Dual Quaternions, this function computes Jacobian matrix to m - th rigid body given joints positions in radians. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      m: int
      xi: np.array (two - dimensional)
    """
    n = robot.jointsPositions.shape[0]
    framesDQ, fkDQ = forwardDQ(robot, m)
    Jd = np.zeros((8, n))
    for j in range(n):
        a = 0.5 * dq.leftOperator(framesDQ[j]).dot(dq.rightOperator(
            dq.conjugate(framesDQ[j]))).dot(dq.rightOperator(fkDQ)).dot(xid[:, j])
        b = 0.5 * dq.dualCrossOperator(W[:, j].reshape((8, 1))).dot(dq.leftOperator(framesDQ[j])).dot(
            dq.rightOperator(dq.conjugate(framesDQ[j]))).dot(dq.rightOperator(fkDQ)).dot(xi[:, j])
        c = 0.25 * dq.leftOperator(framesDQ[j]).dot(dq.rightOperator(dq.conjugate(framesDQ[j]))).dot(
            dq.rightOperator(fkDQ)).dot(dq.rightOperator(W[:, -1].reshape((8, 1)))).dot(xi[:, j])
        Jd[:, j] = a + b + c
    return Jd


def inverseHTM(robot, q0, Hd, K):
  """Using Homogeneous Transformation Matrices, this function computes Inverse Kinematics to m - th rigid body given joints positions in radians. Robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): _description_
    q0 (np.array): initial conditions of joints
    Hd (np.array): desired pose represented with an Homogeneous Transformation Matrix or Axis - Angle vector
    K (np.array): gain matrix to guarantee stability

  Returns:
    q (np.array): joints positions to reach desired pose
  """
  
  # Get the shape of Hd
  r, s = Hd.shape
  
  # Convert desired pose to Axis - Angle Vector if this is an Homogeneous Transformation Matrix
  Xd = mv.axisAngle(Hd) if r == 4 else Hd
  
  # Extract rotation matrix from desired pose and convert it to Homogeneous Transformation Matrix
  Rd = np.append(np.append(Hd[0 : 3, 0 : 3], np.zeros((3, 1)), axis = 1), np.array([[0, 0, 0, 1]]), axis = 0)
  
  # Set initial conditions
  q = q0.reshape(q0.shape)
  
  # Iterator
  j = 0
  
  # While iterator is less than 15000 or absolute value of max
  while j < 15000:
    
    # Set calculated positions on robot
    robot.jointsPositions = q[:, -1].reshape(q0.shape)
    
    # Calculate forward kinematics
    fkH = forwardHTM(robot)
    
    # Set Axis - Angle vector
    X = mv.axisAngle(fkH[-1])
    
    # Extract rotation matrix from current pose and convert it to Homogeneous Transformation Matrix
    R = np.append(np.append(fkH[-1][0 : 3, 0 : 3], np.zeros((3, 1)), axis = 1), np.array([[0, 0, 0, 1]]), axis = 0)
    
    # Calculate error
    e = np.append(Xd[0 : 3, :] - X[0 : 3, :], mv.axisAngle(Rd.dot(R.T))[3 : 6], axis = 0)
    
    # If error is less equal to 0.001
    if np.abs(np.max(e)) <= 0.001:
      
      # Solution found!
      print("\nSolution found using Homogeneous Transformation Matrices!")
      
      # Stop loop
      break
    
    # Calculate Geometric Jacobian Matrix (Inertial)
    J = geometricJacobian(robot)
    
    # Calculate new joints positions
    q = np.append(q, dy.solver(f = (np.linalg.pinv(J)).dot(K).dot(e), F = q[:, -1].reshape(q0.shape)), axis = 1)
    
  return q

def inverseDQ(robot, q0, Qd, K, xi):
  """Using Dual Quaternions, this function computes Inverse Kinematics to m - th rigid body given joints positions in radians. Robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): _description_
    q0 (np.array): initial conditions of joints
    Qd (np.array): desired pose represented with a Dual Quaternion
    K (np.array): gain matrix to guarantee stability
    xi (np.array): axes of actuation of each joint

  Returns:
    q (np.array): joints positions to reach desired pose
  """
  
  # Set initial conditions
  q = q0.reshape(q0.shape)
  
  # Iterator
  j = 0
  
  while j < 15000:
    
    # Set calculated positions on robot
    robot.jointsPositions = q[:, -1].reshape(q0.shape)
    
    # Calculate forward kinematics
    Q = forwardDQ(robot)
    
    # Calculate error
    e = Qd - Q[-1]
    
    # If error is less equal to 0.001
    if np.abs(np.max(e)) <= 0.001:
      
      # Solution found!
      print("\nSolution found using Dual Quaternions!")
      
      # Stop loop
      break
    
    # Calculate inertial jacobian matrix
    J = jacobianDQ(robot, xi)
    
    # Calculate new joints positions
    q = np.append(q, dy.solver(f = (np.linalg.pinv(J)).dot(K).dot(e), F = q[:, -1].reshape(q0.shape)), axis = 1)
    
  return q

"""
  Differential Kinematics Section
"""


def velocityHTM(robot, m, qd):
    """
      Using Homogeneous Transformation Matrices, this function computes Instantaneous Inertial Velocity to m - th frame, given joints velocities in radians/second. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      m: int
      n: int
      qd: np.array (two - dimensional)
    """
    Jhtm = geometricJacobian(robot, m)
    Vhtm = Jhtm.dot(qd)
    return Vhtm


def jointsVelocitiesHTM(robot, m, Vhtm):
    """
      Using Homogeneous Transformation Matrices, this function computes Instantaneous Joints' Velocities to n - th joint, given m - th frame velocity in meters/second. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      m: int
      n: int
      Vhtm: np.array (two - dimensional)
    """
    Jhtm = geometricJacobian(robot, m)
    qd = np.linalg.pinv(Jhtm).dot(Vhtm)
    return qd


def jacobianVDQ(robot, m, n, xi):
    """
      Using Dual Quaternions, this function computes Inertial Velocity Jacobian matrix to n - th joint, given number of frames actuated by joints and joints positions in radians. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      m: int
      n: int
      xi: np.array (two - dimensional)
    """
    framesDQ, fkDQ = forwardDQ(robot, m)
    J = np.zeros((8, n))
    for j in range(n):
        J[:, j] = dq.leftOperator(
            framesDQ[j + 1]).dot(dq.rightOperator(dq.conjugate(framesDQ[j + 1]))).dot(xi[:, j])
    return J


def jacobianADQ(robot, m, n, W0, qd, xi, xid):
    """
      Using Dual Quaternions, this function computes Inertial Acceleration Jacobian matrix to n - th joint, given joints positions in radians. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      m: int
      n: int
      W0: np.array (two - dimensional)
      qd: np.array (two - dimensional)
      xi: np.array (two - dimensional)
    """
    framesQ, fkQ = forwardDQ(robot, m)
    K = np.zeros((8, n))
    W = relativeVelocityDQ(robot, m=m, n = n, W0 = W0, qd = qd, xi = xi)
    for j in range(n):
        K[:, j] = dq.leftOperator(framesQ[j + 1]).dot(dq.rightOperator(dq.conjugate(
            framesQ[j + 1]))).dot((dq.dualCrossOperator(W[:, j + 1]).dot(xi[:, j])) + xid[:, j])
    return K


def velocityDQ(robot, m, n, qd, xi):
    """
      Using Dual Quaternions, this function computes Instantaneous Inertial Velocity to m - th frame, given joints velocities in radians/second. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      m: int
      n: int
      qd: np.array (two - dimensional)
      xi: np.array (two - dimensional)
    """
    framesDQ, fkDQ = forwardDQ(robot, m)
    r = dq.toR3(fkDQ)
    M = np.append(np.append(np.eye(4), np.zeros((4, 4)), axis=1), np.append(-dq.crossOperatorExtension(r), np.eye(4), axis = 1), axis = 0)
    Jvdq = jacobianVDQ(robot, m, n, xi)
    Vdq = M.dot(Jvdq).dot(qd)
    return Vdq


def relativeVelocityDQ(robot, m, n, W0, qd, xi):
    """
      Using Dual Quaternions, this function computes Instantaneous Relative Inertial Velocity to m - th frame, given joints velocities in radians/second. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      n: int
      W0: np.array (two - dimensional)
      qd: np.array (two - dimensional)
      xi: np.array (two - dimensional)
    """
    framesQ, fkQ = forwardDQ(robot, m)
    W = np.append(W0, np.zeros((8, n)), axis=1)
    for j in range(n):
        Q = dq.leftOperator(framesQ[j + 1]).dot(dq.conjugate(framesQ[j]))
        W[:, j + 1] = (dq.leftOperator(dq.conjugate(Q)).dot(dq.rightOperator(Q)).dot(W[:, j])) + \
        (dq.leftOperator(dq.conjugate(Q)).dot(dq.rightOperator(Q)).dot(xi[:, j] * qd[j, :]))
    return W


def jointsVelocitiesDQ(robot, m, n, Vdq, xi):
    """
      Using Dual Quaternions, this function computes Instantaneous Joints' Velocities to n - th joint, given m - th frame velocity in meters/second. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      m: int
      n: int
      Vdq: np.array (two - dimensional)
      xi: np.array (two - dimensional)
    """
    framesDQ, fkDQ = forwardDQ(robot, m)
    r = dq.toR3(fkDQ)
    M = np.append(np.append(np.eye(4), np.zeros((4, 4)), axis=1), np.append(dq.crossOperatorExtension(r), np.eye(4), axis = 1), axis = 0)
    Jvdq = jacobianVDQ(robot, m, n, xi)
    qd = np.linalg.pinv(Jvdq).dot(M).dot(Vdq)
    return qd


def velocityPropagationDQ(robot, m, COMs, W0, qd, xi):
    """
      Using Dual Quaternions, this function computes Instantaneous Relative Inertial Velocity to p - th Center of Mass, given joints velocities in radians/second. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      m: int
      COMs: int
      W0: np.array (two - dimensional)
      qd: np.array (two - dimensional)
      xi: np.array (two - dimensional)
    """
    framesCOMDQ, fkCOMDQ = forwardCOMDQ(robot, m=m)
    framesDQ, fkDQ = forwardDQ(robot, m=m)
    Wcom = np.append(W0, np.zeros((8, COMs)), axis=1)
    for j in range(COMs):
        fkQcomi_i = dq.leftOperator(dq.conjugate(
            framesDQ[j])).dot(framesCOMDQ[j + 1])
        fkQcomi_com = dq.leftOperator(dq.conjugate(
            framesCOMDQ[j])).dot(framesCOMDQ[j + 1])
        Wcom[:, j + 1] = (dq.leftOperator(dq.conjugate(fkQcomi_com)).dot(dq.rightOperator(fkQcomi_com)).dot(Wcom[:, j])) + \
        (dq.leftOperator(dq.conjugate(fkQcomi_i)).dot(dq.rightOperator(fkQcomi_i)).dot(xi[:, j] * qd[j, :]))
    return Wcom


def accelerationDQ(robot, m, n, W0, qd, qdd, xi, xid):
    """
      Using Dual Quaternions, this function computes Instantaneous Inertial Acceleration to m - th frame, given joints velocities in radians/second. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      m: int
      n: int
      W0: np.array (two - dimensional)
      qd: np.array (two - dimensional)
      qdd: np.array (two - dimensional)
      xi: np.array (two - dimensional)
      xid: np.array (two - dimensional)
    """

    # Forward Kinematics
    framesDQ, fkDQ = forwardDQ(robot, m)
    r = dq.toR3(fkDQ)

    # Jacobian Matrices
    J = jacobianVDQ(robot, m=m, n = n, xi = xi)
    K = jacobianADQ(robot, m=m, n = n, W0 = W0, qd = qd, xi = xi, xid = xid)

    # Velocity in Dual form
    V = velocityDQ(robot, m=m, n = n, qd = qd, xi = xi)

    # Coriollis
    C = np.append(np.zeros((4, 1)), dq.crossOperatorExtension(V[0: 4, :]).dot(dq.crossOperatorExtension(V[0: 4, :]).dot(r))).reshape((8, 1))

    # Position matrix
    M = np.append(np.append(np.eye(4), np.zeros((4, 4)), axis=1), np.append(-dq.crossOperatorExtension(r), np.eye(4), axis = 1), axis = 0)

    # Acceleration in Dual form
    Adq = M.dot((J.dot(qdd)) + (K.dot(qd)) + C)
    return Adq


def jointsAccelerationsDQ(robot, m, n, W0, qd, Adq, xi, xid):
    """
      Using Dual Quaternions, this function computes Instantaneous Joints' Accelerations to n - th joint, given end - frame acceleration in Dual form. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      m: int
      n: int
      W0: np.array (two - dimensional)
      qd: np.array (two - dimensional)
      Adq: np.array (two - dimensional)
      xi: np.array (two - dimensional)
      xid: np.array (two - dimensional)
    """

    # Forward Kinematics
    framesDQ, fkDQ = forwardDQ(robot, m)
    r = dq.toR3(fkDQ)

    # Jacobian Matrices
    J = jacobianVDQ(robot, m=m, n = n, xi = xi)
    K = jacobianADQ(robot, m=m, n = n, W0 = W0, qd = qd, xi = xi, xid = xid)

    # Velocity in Dual form
    V = velocityDQ(robot, m=m, n = n, qd = qd, xi = xi)

    # Coriollis
    C = np.append(np.zeros((4, 1)), dq.crossOperatorExtension(V[0: 4, :]).dot(dq.crossOperatorExtension(V[0: 4, :]).dot(r))).reshape((8, 1))

    # Position matrix
    M = np.append(np.append(np.eye(4), np.zeros((4, 4)), axis=1), np.append(dq.crossOperatorExtension(r), np.eye(4), axis = 1), axis = 0)

    # Acceleration in Dual form
    qdd = np.linalg.pinv(J).dot((M.dot(Adq)) - (K.dot(qd)) - C)
    return qdd


def accelerationPropagationDQ(robot, m, COMs, W0, A0, qd, qdd, xi, xid):
    """
      Using Dual Quaternions, this function computes Instantaneous Relative Inertial Acceleration to p - th Center of Mass, given joints velocities in radians/second and joints accelerations in radians/second^2. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      m: int
      COMs: int
      W0: np.array (two - dimensional)
      A0: np.array (two - dimensional)
      qd: np.array (two - dimensional)
      qdd: np.array (two - dimensional)
      xi: np.array (two - dimensional)
      xid: np.array (two - dimensional)
    """
    framesCOMDQ, fkCOMDQ = forwardCOMDQ(robot, m=m)
    framesDQ, fkDQ = forwardDQ(robot, m=m)
    Acom = np.append(A0, np.zeros((8, COMs)), axis=1)
    Wcom = velocityPropagationDQ(robot, m=m, COMs = COMs, W0 = W0, qd = qd, xi = xi)
    for j in range(COMs):
        fkQi_comi = dq.leftOperator(dq.conjugate(
            framesCOMDQ[j + 1])).dot(framesDQ[j + 1])
        fkQcomi_i = dq.leftOperator(dq.conjugate(
            framesDQ[j])).dot(framesCOMDQ[j + 1])
        fkQcomi_com = dq.leftOperator(dq.conjugate(
            framesCOMDQ[j])).dot(framesCOMDQ[j + 1])
        a = dq.leftOperator(fkQi_comi).dot(dq.rightOperator(
            dq.conjugate(fkQi_comi))).dot(xi[:, j] * qd[j, :])
        b = dq.dualCrossOperator(Wcom[:, j]).dot(a)
        c = Acom[:, j] + b
        d = dq.leftOperator(dq.conjugate(fkQcomi_com)).dot(
            dq.rightOperator(fkQcomi_com)).dot(c)
        e = (xid[:, j] * qd[j, :]) + (xi[:, j] * qdd[j, :])
        f = dq.leftOperator(dq.conjugate(fkQcomi_i)).dot(
            dq.rightOperator(fkQcomi_i)).dot(e)
        Acom[:, j + 1] = d + f
    return Acom
