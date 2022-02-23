# Access to parent folder to get its files
import sys, os
sys.path.append(sys.path[0].replace(r'/lib/kinematics', r''))

# Libraries
import numpy as np
from lib.kinematics.DQ import *
from lib.movements.DQ import *
from sympy import *

"""
  UNDER DEVELOPMENT
"""

def jacobianVDQ(robot, xi, symbolic = False):
  """Using Dual Quaternions, this function computes Inertial Velocity Jacobian matrix to n - th joint, given number of frames actuated by joints and joints positions in radians. Robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): _description_
    xi (np.array): axes of rotation of each joint
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    J (np.array): Inertial Velocity Jacobian Matrix
    J (SymPy Matrix): Inertial Velocity Jacobian Matrix
  """
  
  # Get number of joints (generalized coordinates)
  n = robot.jointsPositions.shape[0]
  
  # Calculate forward kinematics
  fkDQ = forwardDQ(robot, symbolic)
  
  # Initializes jacobian matrix with zeros
  J = zeros(8, n) if symbolic else np.zeros((8, n))
  
  # Iterates through all generalized coordinates
  for j in range(n):
  
    # Calculate Jacobian Matrix
    J[:, j] = nsimplify(simplify(dq.symbolicLeftOperator(fkDQ[j + 1]) * dq.symbolicRightOperator(dq.symbolicConjugate(fkDQ[j + 1])) * Matrix(xi[:, j])), tolerance = 1e-10, rational = False) if symbolic else dq.leftOperator(fkDQ[j + 1]).dot(dq.rightOperator(dq.conjugate(fkDQ[j + 1]))).dot(xi[:, j])
  
  return J


def velocityDQ(robot, qd, xi, symbolic = False):
  """Using Dual Quaternions, this function computes Instantaneous Inertial Velocity to m - th frame, given joints velocities in radians/second. Robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): _description_
    xi (np.array): axes of rotation of each joint
    qd (np.array): joints velocities
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    V (np.array): Inertial Velocity
    V (SymPy Matrix): Inertial Velocity
  """
  
  # Calculate forward kinematics
  fkDQ = forwardDQ(robot, symbolic)
  
  # Get end-effector position in euclidean space
  r = dq.symbolicToR3(fkDQ[-1]) if symbolic else dq.toR3(fkDQ[-1])
  
  # End-effector position matrix
  M = eye(4).col_insert(4, zeros(4)).row_insert(4, dq.symbolicCrossOperatorExtension(r).col_insert(4, eye(4))) if symbolic else np.append(np.append(np.eye(4), np.zeros((4, 4)), axis=1), np.append(dq.crossOperatorExtension(r), np.eye(4), axis = 1), axis = 0)
  
  # Inertial Velocity Jacobian Matrix
  J = jacobianVDQ(robot, xi, symbolic)
  
  # End-effector inertial velocity
  V = M.inv() * J * qd if symbolic else np.linalg.pinv(M).dot(J).dot(qd)
  
  return V


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


def jacobianADQ(robot, m, n, W0, qd, xi, xid):
  """Using Dual Quaternions, this function computes Inertial Acceleration Jacobian matrix to n - th joint, given joints positions in radians. Robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): _description_
    xi (np.array): axes of rotation of each joint
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    J (np.array): Inertial Velocity Jacobian Matrix
    J (SymPy Matrix): Inertial Velocity Jacobian Matrix
  """
  
  framesQ, fkQ = forwardDQ(robot, m)
  K = np.zeros((8, n))
  W = relativeVelocityDQ(robot, m=m, n = n, W0 = W0, qd = qd, xi = xi)
  for j in range(n):
    K[:, j] = dq.leftOperator(fkQ[j + 1]).dot(dq.rightOperator(dq.conjugate(fkQ[j + 1]))).dot((dq.dualCrossOperator(W[:, j + 1]).dot(xi[:, j])) + xid[:, j])
  
  return K


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


if __name__ == '__main__':
  
  """
    THIS SECTION IS FOR TESTING PURPOSES ONLY
  """
  
  print("Z")