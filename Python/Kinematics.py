import DenavitHartenberg as dh
import DualQuaternions as dq
import Dynamics as dy
import Movements as mv
import numpy as np

"""
  Kinematics Section
"""

def forwardHTM(robot, m = 0):
    """
      Using Homogeneous Transformation Matrices, this function computes forward kinematics to m - th rigid body given joints positions in radians. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      m: int
    """
    
    # Initial conditions
    framesHTM = []
    
    # Gets Denavit - Hartenberg Matrix
    DH = dh.matrix(robot)
    
    # Computes forward kinematics, from inertial frame to m - th one
    fkHTM = np.identity(4)
    i = 0
    for frame in DH:
      if i > m:
        break
      else:
        fkHTM = fkHTM.dot(mv.rz(frame[0]).dot(mv.tz(frame[1])).dot(mv.tx(frame[2])).dot(mv.rx(frame[3])))
        framesHTM.append(fkHTM)
        i += 1
    return framesHTM, fkHTM

def forwardCOMHTM(robot, m = 0):
  """
    Using Homogeneous Transformation Matrices, this function computes forward kinematics to m - th center of mass, given joints positions in radians. Robot's kinematic parameters have to be set before using this function
    robot: object (robot.jointsPositions, robot.linksLengths)
    m: int
  """
  framesHTM, fkHTM = forwardHTM(robot, m = m)
    
  # Initial conditions
  framesCOMHTM = [np.identity(4)]
    
  # Gets Denavit - Hartenberg Matrix
  comDH = dh.centersOfMass(robot)
  
  i = 1
  for frame in comDH[1 : , :]:
    if i > m:
      break
    else:
      # Center of Mass Homogeneous Transformation Matrix
      COM = mv.rz(frame[0]).dot(mv.tz(frame[1])).dot(mv.tx(frame[2])).dot(mv.rx(frame[3]))
      
      # Rigid body's Homogeneous Transformation Matrix
      B = np.linalg.pinv(framesHTM[i - 1]).dot(framesHTM[i])
      
      # Forward kinematics to Center of Mass
      fkCOMHTM = framesHTM[i].dot(np.linalg.inv(B)).dot(COM)
      framesCOMHTM.append(fkCOMHTM)
      i += 1
  return framesCOMHTM, fkCOMHTM

def forwardDQ(robot, m = 0):
    """
      Using Dual Quaternions, this function computes forward kinematics to m - th rigid body given joints positions in radians. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      m: int
    """
    
    # Initial conditions
    framesDQ = []
    
    # Gets Denavit - Hartenberg Matrix
    DH = dh.matrix(robot)
    
    # Computes forward kinematics, from inertial frame to m - th one
    fkDQ = np.array([[1, 0, 0, 0, 0, 0, 0, 0]]).T
    i = 0
    for frame in DH:
      if i > m:
        break
      else:
        fkDQ = dq.leftOperator(fkDQ).dot(dq.rightOperator(dq.Rx(frame[3]))).dot(dq.leftOperator(dq.Rz(frame[0]))).dot(dq.rightOperator(dq.Tx(frame[2]))).dot(dq.Tz(frame[1]))
        framesDQ.append(fkDQ)
        i += 1
    return framesDQ, fkDQ

def forwardCOMDQ(robot, m = 0):
  """
    Using Dual Quaternions, this function computes forward kinematics to m - th center of mass, given joints positions in radians. Robot's kinematic parameters have to be set before using this function
    robot: object (robot.jointsPositions, robot.linksLengths)
    m: int
  """
  framesDQ, fkDQ = forwardDQ(robot, m = m)
    
  # Initial conditions
  framesCOMDQ = [np.array([[1], [0], [0], [0], [0], [0], [0], [0]])]
    
  # Gets Denavit - Hartenberg Matrix
  comDH = dh.centersOfMass(robot)
  
  i = 1
  for frame in comDH[1 : , :]:
    if i > m:
      break
    else:
      # Center of Mass Homogeneous Transformation Matrix
      COM = dq.leftOperator(dq.Rz(frame[0])).dot(dq.rightOperator(dq.Rx(frame[3]))).dot(dq.rightOperator(dq.Tx(frame[2]))).dot(dq.Tz(frame[1]))
      
      # Rigid body's Dual Quaternion
      B = dq.leftOperator(dq.conjugate(framesDQ[i - 1])).dot(framesDQ[i])
      
      # Forward kinematics to Center of Mass
      fkCOMDQ = dq.leftOperator(framesDQ[i]).dot(dq.rightOperator(COM)).dot(dq.conjugate(B))
      framesCOMDQ.append(fkCOMDQ)
      i += 1
  return framesCOMDQ, fkCOMDQ

def jacobianHTM(robot, m):
    """
      Using Homogeneous Transformation Matrices, this function computes Jacobian matrix to m - th rigid body given joints positions in radians. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      m: int
    """
    n = robot.jointsPositions.shape[0]
    framesHTM, fkHTM = forwardHTM(robot, m)
    J = np.zeros((6, n))
    for j in range(n):
      framesH, fkH = forwardHTM(robot, j)
      z = fkH[0 : 3, 2]
      r = fkHTM[0 : 3, 3] - fkH[0 : 3, 3]
      J[0 : 3, j] = np.cross(z, r)
      J[3 : 6, j] = z
    return J

def inverseHTM(robot, q0, Hd, K, m):
    """
      Using Homogeneous Transformation Matrices, this function computes Inverse Kinematics to m - th rigid body given joints positions in radians. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      q0: np.array (two - dimensional)
      Hd: np.array (two - dimensional)
      K: np.array (two - dimensional)
      m: int
    """
    Xd = mv.axisAngle(Hd)
    q = q0.reshape(q0.shape)
    for j in range(1, 15000):
      robot.jointsPositions = q[:, j - 1].reshape(q0.shape)
      framesH, fkH = forwardHTM(robot, m)
      X = mv.axisAngle(fkH)
      e = Xd - X
      if all(value <= 0.001 for value in abs(e)):
        break
      J = jacobianHTM(robot, m)
      q = np.append(q, dy.solver(f = (np.linalg.pinv(J)).dot(K).dot(e), F = q[:, j - 1].reshape(q0.shape), dt = 3/1000), axis = 1)
    return q

def jacobianDQ(robot, m, xi):
    """
      Using Dual Quaternions, this function computes Jacobian matrix to m - th rigid body given joints positions in radians. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      m: int
      xi: np.array (two - dimensional)
    """
    n = robot.jointsPositions.shape[0]
    framesDQ, fkDQ = forwardDQ(robot, m)
    J = np.zeros((8, n))
    for j in range(n):
      framesQ, fkQ = forwardDQ(robot, j)
      J[:, j] = 0.5 * dq.leftOperator(fkQ).dot(dq.rightOperator(fkDQ)).dot(dq.rightOperator(dq.conjugate(fkQ))).dot(xi[:, j])
    return J
  
def inverseDQ(robot, q0, Qd, K, xi, m):
    """
      Using Dual Quaternions, this function computes Inverse Kinematics to m - th rigid body given joints positions in radians. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      q0: np.array (two - dimensional)
      Qd: np.array (two - dimensional)
      K: np.array (two - dimensional)
      xi: np.array (two - dimensional)
      m: int
    """
    i = 0
    q = q0.reshape(q0.shape)
    for j in range(1, 15000):
      robot.jointsPositions = q[:, j - 1].reshape(q0.shape)
      framesDQ, Q = forwardDQ(robot, m)
      e = Qd - Q
      if all(value <= 0.001 for value in abs(e)):
        break
      J = jacobianDQ(robot, m, xi)
      q = np.append(q, dy.solver(f = (np.linalg.pinv(J)).dot(K).dot(e), F = q[:, j - 1].reshape(q0.shape), dt = 3/1000), axis = 1)
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
  Jhtm = jacobianHTM(robot, m)
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
  Jhtm = jacobianHTM(robot, m)
  qd = np.linalg.pinv(Jhtm).dot(Vhtm)
  return qd

def jacobianVDQ(robot, n, xi):
  """
    Using Dual Quaternions, this function computes Inertial Velocity Jacobian matrix to n - th joint, given joints positions in radians. Robot's kinematic parameters have to be set before using this function
    robot: object (robot.jointsPositions, robot.linksLengths)
    n: int
    xi: np.array (two - dimensional)
  """
  J = np.zeros((8, n))
  for j in range(n):
    framesQ, fkQ = forwardDQ(robot, j + 1)
    J[:, j] = dq.leftOperator(fkQ).dot(dq.rightOperator(dq.conjugate(fkQ))).dot(xi[:, j])
  return J

def jacobianADQ(robot, n, W0, qd, xi, xid):
  """
    Using Dual Quaternions, this function computes Inertial Acceleration Jacobian matrix to n - th joint, given joints positions in radians. Robot's kinematic parameters have to be set before using this function
    robot: object (robot.jointsPositions, robot.linksLengths)
    n: int
    W0: np.array (two - dimensional)
    qd: np.array (two - dimensional)
    xi: np.array (two - dimensional)
  """
  K = np.zeros((8, n))
  W = relativeVelocityDQ(robot, n = n, W0 = W0, qd = qd, xi = xi)
  for j in range(n):
    framesQ, fkQ = forwardDQ(robot, j + 1)
    K[:, j] = dq.leftOperator(fkQ).dot(dq.rightOperator(dq.conjugate(fkQ))).dot((dq.dualCrossOperator(W[:, j + 1]).dot(xi[:, j])) + xid[:, j])
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
  M = np.append(np.append(np.eye(4), np.zeros((4, 4)), axis = 1), np.append(-dq.crossOperatorExtension(r), np.eye(4), axis = 1), axis = 0)
  Jvdq = jacobianVDQ(robot, n, xi)
  Vdq = M.dot(Jvdq).dot(qd)
  return Vdq

def relativeVelocityDQ(robot, n, W0, qd, xi):
  """
    Using Dual Quaternions, this function computes Instantaneous Relative Inertial Velocity to m - th frame, given joints velocities in radians/second. Robot's kinematic parameters have to be set before using this function
    robot: object (robot.jointsPositions, robot.linksLengths)
    n: int
    W0: np.array (two - dimensional)
    qd: np.array (two - dimensional)
    xi: np.array (two - dimensional)
  """
  W = np.append(W0, np.zeros((8, n)), axis = 1)
  for j in range(n):
    framesQi, fkQi = forwardDQ(robot, j + 1)
    framesQ, fkQ = forwardDQ(robot, j)
    Q = dq.leftOperator(fkQi).dot(dq.conjugate(fkQ))
    W[:, j + 1] = (dq.leftOperator(dq.conjugate(Q)).dot(dq.rightOperator(Q)).dot(W[:, j])) + (dq.leftOperator(dq.conjugate(Q)).dot(dq.rightOperator(Q)).dot(xi[:, j] * qd[j, :]))
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
  M = np.append(np.append(np.eye(4), np.zeros((4, 4)), axis = 1), np.append(dq.crossOperatorExtension(r), np.eye(4), axis = 1), axis = 0)
  Jvdq = jacobianVDQ(robot, n, xi)
  qd = np.linalg.pinv(Jvdq).dot(M).dot(Vdq)
  return qd

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
  J = jacobianVDQ(robot, n = n, xi = xi)
  K = jacobianADQ(robot, n = n, W0 = W0, qd = qd, xi = xi, xid = xid)
  
  # Velocity in Dual form
  V = velocityDQ(robot, m = m, n = n, qd = qd, xi = xi)
  
  # Coriollis
  C = np.append(np.zeros((4, 1)), dq.crossOperatorExtension(V[0 : 4, :]).dot(dq.crossOperatorExtension(V[0 : 4, :]).dot(r))).reshape((8, 1))
  
  # Position matrix
  M = np.append(np.append(np.eye(4), np.zeros((4, 4)), axis = 1), np.append(-dq.crossOperatorExtension(r), np.eye(4), axis = 1), axis = 0)
  
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
  J = jacobianVDQ(robot, n = n, xi = xi)
  K = jacobianADQ(robot, n = n, W0 = W0, qd = qd, xi = xi, xid = xid)
  
  # Velocity in Dual form
  V = velocityDQ(robot, m = m, n = n, qd = qd, xi = xi)
  
  # Coriollis
  C = np.append(np.zeros((4, 1)), dq.crossOperatorExtension(V[0 : 4, :]).dot(dq.crossOperatorExtension(V[0 : 4, :]).dot(r))).reshape((8, 1))
  
  # Position matrix
  M = np.append(np.append(np.eye(4), np.zeros((4, 4)), axis = 1), np.append(dq.crossOperatorExtension(r), np.eye(4), axis = 1), axis = 0)
  
  # Acceleration in Dual form
  qdd = np.linalg.pinv(J).dot((M.dot(Adq)) - (K.dot(qd)) - C)
  return qdd