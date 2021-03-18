import DenavitHartenberg as dh
import DualQuaternions as dq
import Dynamics as dy
import Movements as mv
import multiprocessing
import numpy as np
from sympy import *

"""
  Kinematics Section
"""

def forwardHTM(robot, m = 0, symbolic = False):
    """
      Using Homogeneous Transformation Matrices, this function computes forward kinematics to m - th rigid body given joints positions in radians. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      m: int
    """
    
    # Initial conditions
    framesHTM = []
    
    # Gets Denavit - Hartenberg Matrix
    if not symbolic:
      if not robot.dhParameters:
        DH = dh.matrix(robot)
      else:
        DH = np.array(robot.dhParameters([float(q) for q in robot.jointsPositions], [float(L) for L in robot.linksLengths]))
    else:
        DH = robot.symbolicDHParameters
    
    # Computes forward kinematics, from inertial frame to m - th one
    fkHTM = np.identity(4) if not symbolic else eye(4)
    i = 0
    for frame in DH:
      if i > m:
        break
      else:
        if not symbolic:
          fkHTM = fkHTM.dot(mv.rz(frame[0]).dot(mv.tz(frame[1])).dot(mv.tx(frame[2])).dot(mv.rx(frame[3])))
        else:
          fkHTM = nsimplify(simplify(fkHTM * mv.symbolicRz(frame[0]) * mv.symbolicTz(frame[1]) * mv.symbolicTx(frame[2]) * mv.symbolicRx(frame[3])), tolerance = 1e-10, rational = False)
        framesHTM.append(fkHTM)
        i += 1
    return framesHTM, fkHTM

def forwardCOMHTM(robot, m = 0, symbolic = False):
  """
    Using Homogeneous Transformation Matrices, this function computes forward kinematics to m - th center of mass, given joints positions in radians. Robot's kinematic parameters have to be set before using this function
    robot: object (robot.jointsPositions, robot.linksLengths)
    m: int
  """
  framesHTM, fkHTM = forwardHTM(robot, m = m) if not symbolic else forwardHTM(robot, m = m, symbolic = symbolic)
    
  # Initial conditions
  framesCOMHTM = [np.identity(4) if not symbolic else eye(4)]
    
  # Gets Denavit - Hartenberg Matrix
  if not symbolic:
    if not robot.dhParametersCOM:
      comDH = dh.centersOfMass(robot)
    else:
      comDH = np.array(robot.dhParameters([float(q) for q in robot.jointsPositions], [float(Lcom) for Lcom in robot.centersOfMass]))
  else:
    comDH = robot.symbolicDHParametersCOM
  
  i = 1
  for frame in comDH[1 : , :]:
    if i > m:
      break
    else:
      if not symbolic:
        # Center of Mass Homogeneous Transformation Matrix
        COM = mv.rz(frame[0]).dot(mv.tz(frame[1])).dot(mv.tx(frame[2])).dot(mv.rx(frame[3]))
        
        # Rigid body's Homogeneous Transformation Matrix
        B = np.linalg.pinv(framesHTM[i - 1]).dot(framesHTM[i])
        
        # Forward kinematics to Center of Mass
        fkCOMHTM = framesHTM[i].dot(np.linalg.inv(B)).dot(COM)
      else:
        # Center of Mass Homogeneous Transformation Matrix (symbolic)
        COM = mv.symbolicRz(frame[0]) * mv.symbolicTz(frame[1]) * mv.symbolicTx(frame[2]) * mv.symbolicRx(frame[3])
        
        # Rigid body's Homogeneous Transformation Matrix (symbolic)
        B = framesHTM[i - 1].inv() * framesHTM[i]
        
        # Forward kinematics to Center of Mass (symbolic)
        fkCOMHTM = nsimplify(simplify(framesHTM[i] * B.inv() * COM), tolerance = 1e-10, rational = False)
      framesCOMHTM.append(fkCOMHTM)
      i += 1
  return framesCOMHTM, fkCOMHTM

def forwardDQ(robot, m = 0, symbolic = False):
    """
      Using Dual Quaternions, this function computes forward kinematics to m - th rigid body given joints positions in radians. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      m: int
    """
    
    # Initial conditions
    framesDQ = []
    
    # Gets Denavit - Hartenberg Matrix
    if not symbolic:
      if not robot.dhParameters:
          DH = dh.matrix(robot)
      else:
        DH = np.array(robot.dhParameters([float(q) for q in robot.jointsPositions], [float(L) for L in robot.linksLengths]))
    else:
      DH = robot.symbolicDHParameters
    
    # Computes forward kinematics, from inertial frame to m - th one
    fkDQ = np.array([[1, 0, 0, 0, 0, 0, 0, 0]]).T if not symbolic else Matrix([1, 0, 0, 0, 0, 0, 0, 0])
    i = 0
    for frame in DH:
      if i > m:
        break
      else:
        if not symbolic:
          fkDQ = dq.leftOperator(fkDQ).dot(dq.rightOperator(dq.Rx(frame[3]))).dot(dq.leftOperator(dq.Rz(frame[0]))).dot(dq.rightOperator(dq.Tx(frame[2]))).dot(dq.Tz(frame[1]))
        else:
          fkDQ = nsimplify(simplify(dq.symbolicLeftOperator(fkDQ) * dq.symbolicRightOperator(dq.symbolicRx(frame[3])) * dq.symbolicLeftOperator(dq.symbolicRz(frame[0])) * dq.symbolicRightOperator(dq.symbolicTx(frame[2])) * dq.symbolicTz(frame[1])), tolerance = 1e-10, rational = False)
        framesDQ.append(fkDQ)
        i += 1
    return framesDQ, fkDQ

def forwardCOMDQ(robot, m = 0, symbolic = False):
      
  """
    Using Dual Quaternions, this function computes forward kinematics to m - th center of mass, given joints positions in radians. Robot's kinematic parameters have to be set before using this function
    robot: object (robot.jointsPositions, robot.linksLengths)
    m: int
  """
  framesDQ, fkDQ = forwardDQ(robot, m = m, symbolic = symbolic)
    
  # Initial conditions
  framesCOMDQ = [np.array([[1], [0], [0], [0], [0], [0], [0], [0]]) if not symbolic else Matrix([1, 0, 0, 0, 0, 0, 0, 0])]
    
  # Gets Denavit - Hartenberg Matrix
  if not symbolic:
    if not robot.dhParametersCOM:
      comDH = dh.centersOfMass(robot)
    else:
      comDH = np.array(robot.dhParameters([float(q) for q in robot.jointsPositions], [float(Lcom) for Lcom in robot.centersOfMass]))
  else:
    comDH = robot.symbolicDHParametersCOM
  
  i = 1
  for frame in comDH[1 : , :]:
    if i > m:
      break
    else:
      if not symbolic:
        # Center of Mass Homogeneous Transformation Matrix
        COM = dq.leftOperator(dq.Rz(frame[0])).dot(dq.rightOperator(dq.Rx(frame[3]))).dot(dq.rightOperator(dq.Tx(frame[2]))).dot(dq.Tz(frame[1]))
        
        # Rigid body's Dual Quaternion
        B = dq.leftOperator(dq.conjugate(framesDQ[i - 1])).dot(framesDQ[i])
        
        # Forward kinematics to Center of Mass
        fkCOMDQ = dq.leftOperator(framesDQ[i]).dot(dq.rightOperator(COM)).dot(dq.conjugate(B))
      else:
        # Center of Mass Homogeneous Transformation Matrix
        COM = dq.symbolicLeftOperator(dq.symbolicRz(frame[0])) * dq.symbolicRightOperator(dq.symbolicRx(frame[3])) * dq.symbolicRightOperator(dq.symbolicTx(frame[2])) * dq.symbolicTz(frame[1])
        
        # Rigid body's Dual Quaternion
        B = dq.symbolicLeftOperator(dq.symbolicConjugate(framesDQ[i - 1])) * framesDQ[i]
        
        # Forward kinematics to Center of Mass
        fkCOMDQ = nsimplify(simplify(dq.symbolicLeftOperator(framesDQ[i]) * dq.symbolicRightOperator(COM) * dq.symbolicConjugate(B)), tolerance = 1e-10, rational = False)
        
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
      z = framesHTM[j][0 : 3, 2]
      r = fkHTM[0 : 3, 3] - framesHTM[j][0 : 3, 3]
      J[0 : 3, j] = np.cross(z, r)
      J[3 : 6, j] = z
    return J

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
    J[:, j] = 0.5 * dq.leftOperator(framesDQ[j]).dot(dq.rightOperator(fkDQ)).dot(dq.rightOperator(dq.conjugate(framesDQ[j]))).dot(xi[:, j])
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
    a = 0.5 * dq.leftOperator(framesDQ[j]).dot(dq.rightOperator(dq.conjugate(framesDQ[j]))).dot(dq.rightOperator(fkDQ)).dot(xid[:, j])
    b = 0.5 * dq.dualCrossOperator(W[:, j].reshape((8, 1))).dot(dq.leftOperator(framesDQ[j])).dot(dq.rightOperator(dq.conjugate(framesDQ[j]))).dot(dq.rightOperator(fkDQ)).dot(xi[:, j])
    c = 0.25 * dq.leftOperator(framesDQ[j]).dot(dq.rightOperator(dq.conjugate(framesDQ[j]))).dot(dq.rightOperator(fkDQ)).dot(dq.rightOperator(W[:, -1].reshape((8, 1)))).dot(xi[:, j])
    Jd[:, j] = a + b + c
  return Jd

def inverseHTM(robot, q0, Hd, K, m):
    """
      Using Homogeneous Transformation Matrices, this function computes Inverse Kinematics to m - th rigid body given joints positions in radians. Robot's kinematic parameters have to be set before using this function
      robot: object (robot.jointsPositions, robot.linksLengths)
      q0: np.array (two - dimensional)
      Hd: np.array (two - dimensional)
      K: np.array (two - dimensional)
      m: int
    """
    r, s = Hd.shape
    # Check if this is an Homogeneous Transformation Matrix
    if r == 4:
      # Convert it into an Axis - Angle Vector 
      Xd = mv.axisAngle(Hd)
    # Check if this is an Axis - Angle Vector
    elif r == 6:
      # Store it as the desired Axis - Angle Vector
      Xd = Hd
    
    q = q0.reshape(q0.shape)
    for j in range(1, 15000):
      robot.jointsPositions = q[:, -1].reshape(q0.shape)
      framesH, fkH = forwardHTM(robot, m)
      X = mv.axisAngle(fkH)
      e = Xd - X
      if all(value <= 0.001 for value in abs(e)):
        break
      J = jacobianHTM(robot, m)
      q = np.append(q, dy.solver(f = (np.linalg.pinv(J)).dot(K).dot(e), F = q[:, -1].reshape(q0.shape), dt = 3/1000), axis = 1)
    return q
    
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
    robot.jointsPositions = q[:, -1].reshape(q0.shape)
    framesDQ, Q = forwardDQ(robot, m)
    e = Qd - Q
    if all(value <= 0.001 for value in abs(e)):
      break
    J = jacobianDQ(robot, m, xi)
    q = np.append(q, dy.solver(f = (np.linalg.pinv(J)).dot(K).dot(e), F = q[:, -1].reshape(q0.shape), dt = 3/1000), axis = 1)
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
    J[:, j] = dq.leftOperator(framesDQ[j + 1]).dot(dq.rightOperator(dq.conjugate(framesDQ[j + 1]))).dot(xi[:, j])
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
  W = relativeVelocityDQ(robot, m = m, n = n, W0 = W0, qd = qd, xi = xi)
  for j in range(n):
    K[:, j] = dq.leftOperator(framesQ[j + 1]).dot(dq.rightOperator(dq.conjugate(framesQ[j + 1]))).dot((dq.dualCrossOperator(W[:, j + 1]).dot(xi[:, j])) + xid[:, j])
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
  W = np.append(W0, np.zeros((8, n)), axis = 1)
  for j in range(n):
    Q = dq.leftOperator(framesQ[j + 1]).dot(dq.conjugate(framesQ[j]))
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
  framesCOMDQ, fkCOMDQ = forwardCOMDQ(robot, m = m)
  framesDQ, fkDQ = forwardDQ(robot, m = m)
  Wcom = np.append(W0, np.zeros((8, COMs)), axis = 1)
  for j in range(COMs):
    fkQcomi_i = dq.leftOperator(dq.conjugate(framesDQ[j])).dot(framesCOMDQ[j + 1])
    fkQcomi_com = dq.leftOperator(dq.conjugate(framesCOMDQ[j])).dot(framesCOMDQ[j + 1])
    Wcom[:, j + 1] = (dq.leftOperator(dq.conjugate(fkQcomi_com)).dot(dq.rightOperator(fkQcomi_com)).dot(Wcom[:, j])) + (dq.leftOperator(dq.conjugate(fkQcomi_i)).dot(dq.rightOperator(fkQcomi_i)).dot(xi[:, j] * qd[j, :]))
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
  J = jacobianVDQ(robot, m = m, n = n, xi = xi)
  K = jacobianADQ(robot, m = m, n = n, W0 = W0, qd = qd, xi = xi, xid = xid)
  
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
  J = jacobianVDQ(robot, m = m, n = n, xi = xi)
  K = jacobianADQ(robot, m = m, n = n, W0 = W0, qd = qd, xi = xi, xid = xid)
  
  # Velocity in Dual form
  V = velocityDQ(robot, m = m, n = n, qd = qd, xi = xi)
  
  # Coriollis
  C = np.append(np.zeros((4, 1)), dq.crossOperatorExtension(V[0 : 4, :]).dot(dq.crossOperatorExtension(V[0 : 4, :]).dot(r))).reshape((8, 1))
  
  # Position matrix
  M = np.append(np.append(np.eye(4), np.zeros((4, 4)), axis = 1), np.append(dq.crossOperatorExtension(r), np.eye(4), axis = 1), axis = 0)
  
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
  framesCOMDQ, fkCOMDQ = forwardCOMDQ(robot, m = m)
  framesDQ, fkDQ = forwardDQ(robot, m = m)
  Acom = np.append(A0, np.zeros((8, COMs)), axis = 1)
  Wcom = velocityPropagationDQ(robot, m = m, COMs = COMs, W0 = W0, qd = qd, xi = xi)
  for j in range(COMs):
    fkQi_comi = dq.leftOperator(dq.conjugate(framesCOMDQ[j + 1])).dot(framesDQ[j + 1])
    fkQcomi_i = dq.leftOperator(dq.conjugate(framesDQ[j])).dot(framesCOMDQ[j + 1])
    fkQcomi_com = dq.leftOperator(dq.conjugate(framesCOMDQ[j])).dot(framesCOMDQ[j + 1])
    a = dq.leftOperator(fkQi_comi).dot(dq.rightOperator(dq.conjugate(fkQi_comi))).dot(xi[:, j] * qd[j, :])
    b = dq.dualCrossOperator(Wcom[:, j]).dot(a)
    c = Acom[:, j] + b
    d = dq.leftOperator(dq.conjugate(fkQcomi_com)).dot(dq.rightOperator(fkQcomi_com)).dot(c)
    e = (xid[:, j] * qd[j, :]) + (xi[:, j] * qdd[j, :])
    f = dq.leftOperator(dq.conjugate(fkQcomi_i)).dot(dq.rightOperator(fkQcomi_i)).dot(e)
    Acom[:, j + 1] = d + f
  return Acom