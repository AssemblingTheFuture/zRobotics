import DenavitHartenberg as dh
import DualQuaternions as dq
import Dynamics as dy
import Movements as mv
import numpy as np

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
    