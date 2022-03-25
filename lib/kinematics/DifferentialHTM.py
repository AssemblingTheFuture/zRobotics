# Access to parent folder to get its files
from audioop import cross
import sys, os
sys.path.append(sys.path[0].replace(r'/lib/kinematics', r''))

# Libraries
from lib.kinematics.HTM import *
import numpy as np
from sympy import *

def geometricStateSpace(robot : object, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes state-space equation (using Geometric Jacobian Matrix) of a serial robot given joints velocities. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    qd (np.array): joints velocities
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    Xd (np.array): state-space equation (X'(t), numerical)
    Xd (SymPy Matrix): state-space equation (X'(t), symbolic)
  """

  # Calculate Inertial Geometric Jacobian Matrix 
  J = geometricJacobian(robot, symbolic)
    
  # Calculate state-space equation (symbolic or numerical)
  Xd = J * robot.qdSymbolic if symbolic else J.dot(robot.jointsVelocities)
    
  return Xd

def geometricCOMStateSpace(robot : object, COM : int, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes state-space equation (using Geometric Jacobian Matrix for Centers of Mass) of a serial robot given joints velocities. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    COM (int): center of mass that will be analyzed
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    Xd (np.array): state-space equation (X'(t), numerical)
    Xd (SymPy Matrix): state-space equation (X'(t), symbolic)
  """

  # Calculate Inertial Geometric Jacobian Matrix 
  Jcom = geometricJacobianCOM(robot, COM, symbolic)
    
  # Calculate state-space equation (symbolic or numerical)
  Xd = Jcom * robot.qdSymbolic if symbolic else Jcom.dot(robot.jointsVelocities)
    
  return Xd

def analyticStateSpace(robot : object, dq = 0.001, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes state-space equation (using Analytic Jacobian Matrix) of a serial robot given joints velocities. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    dq (float, optional): step size for numerical derivative. Defaults to 0.001.
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    Xd (np.array): state-space equation (X'(t), numerical)
    Xd (SymPy Matrix): state-space equation (X'(t), symbolic)
  """

  # Calculate Inertial Analytic Jacobian Matrix 
  J = analyticJacobian(robot, dq, symbolic)
    
  # Calculate state-space equation (symbolic or numerical)
  Xd = J * robot.qdSymbolic if symbolic else J.dot(robot.jointsVelocities)
    
  return Xd

def analyticCOMStateSpace(robot : object, COM : int, dq = 0.001, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes state-space equation (using Analytic Jacobian Matrix for Centers of Mass) of a serial robot given joints velocities. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    COM (int): center of mass that will be analyzed
    dq (float, optional): step size for numerical derivative. Defaults to 0.001
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    Xd (np.array): state-space equation (X'(t), numerical)
    Xd (SymPy Matrix): state-space equation (X'(t), symbolic)
  """

  # Calculate Inertial Analytic Jacobian Matrix to Center of Mass
  J = analyticJacobianCOM(robot, COM, dq, symbolic)
    
  # Calculate state-space equation (symbolic or numerical)
  Xd = J * robot.qdSymbolic if symbolic else J.dot(robot.jointsVelocities)
    
  return Xd

def angularVelocityPropagation(robot : object, w0 : np.array, qd : np.array, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes angular velocity to the i-th reference frame of a serial robot given initial velocity. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    w0 (np.array): initial angular velocity of the system (equals to zero if the robot's base is not mobile)
    qd (np.array): velocities of each joint
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    W (np.array): velocity of each reference frame attached to joints (numerical)
    W (SymPy Matrix): velocity of each reference frame attached to joints (symbolic)
  """
  
  # Initial conditions
  W = [w0]
  
  # Calculate forward kinematics to know the position and axis of actuation of each joint
  fkHTM = forwardHTM(robot, symbolic = symbolic)
  
  # Get number of joints (generalized coordinates)
  n = robot.jointsPositions.shape[0]
  
  # Iterates through all colums (generalized coordinates)
  for i in range(n):     
    
    # Check in what row of Denavit Hartenberg Parameters Matrix is the current joint (the sum is because of the way Python indexes arrays)
    row, column = robot.whereIsTheJoint(i + 1)
    
    # Get pose of reference frame where current joint is attached
    H = fkHTM[row - 1]
    
    # Get axis of actuation of current joint
    z = trigsimp(H[0: 3, 2]) if symbolic else H[0: 3, 2]
    
    # Calculate angular velocity up to this point
    w = nsimplify(W[-1] + (z * qd[i]), tolerance = 1e-10) if symbolic else W[-1] + (z * qd[i, :]).reshape((3, 1))

    # Append each calculated angular velocity
    W.append(trigsimp(w) if symbolic else w)
      
  return W

def linearVelocityPropagation(robot : object, v0 : np.array, W : np.array, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes linear velocity to the i-th reference frame of a serial robot given initial velocity. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    v0 (np.array): initial linear velocity of the system (equals to zero if the robot's base is not mobile)
    W (np.array): angular velocities of the system (this have to be calculated with "angularVelocityPropagation" function)
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    V (np.array): velocity of each reference frame attached to joints (numerical)
    V (SymPy Matrix): velocity of each reference frame attached to joints (symbolic)
  """
  
  # Initial conditions
  V = [v0]
  
  # Calculate forward kinematics to know the position and axis of actuation of each joint
  fkHTM = forwardHTM(robot, symbolic = symbolic)
  
  # Get number of joints (generalized coordinates)
  n = robot.jointsPositions.shape[0]
  
  # Iterates through all colums (generalized coordinates)
  for i in range(n):     
    
    # Check in what row of Denavit Hartenberg Parameters Matrix is the current joint (the sum is because of the way Python indexes arrays)
    row, column = robot.whereIsTheJoint(i + 1)
    
    # Get relative position of rigid body
    r = trigsimp(fkHTM[row][0 : 3, - 1] - fkHTM[row - 1][0 : 3, - 1]) if symbolic else fkHTM[row][0 : 3, - 1] - fkHTM[row - 1][0 : 3, - 1]
    
    # Calculate linear velocity up to this point
    v = nsimplify(V[-1] + (W[i + 1].cross(r)), tolerance = 1e-10) if symbolic else V[-1] + (np.cross(W[i + 1], r, axis = 0))

    # Append each calculated linear velocity
    V.append(trigsimp(v) if symbolic else v)
      
  return V

def angularAccelerationPropagation(robot : object, dw0 : np.array, W : list, qd : np.array, qdd : np.array, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes angular acceleration to the i-th reference frame of a serial robot given initial acceleration. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    dw0 (np.array): initial angular acceleration of the system (equals to zero if the robot's base is not mobile)
    W (np.array): angular velocities of the system (this have to be calculated with "angularVelocityPropagation" function)
    qd (np.array): velocities of each joint
    qdd (np.array): accelerations of each joint
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    dW (np.array): angular acceleration of each reference frame attached to joints (numerical)
    dW (SymPy Matrix): angular acceleration of each reference frame attached to joints (symbolic)
  """
  
  # Initial conditions
  dW = [dw0]
  
  # Calculate forward kinematics to know the position and axis of actuation of each joint
  fkHTM = forwardHTM(robot, symbolic = symbolic)
  
  # Get number of joints (generalized coordinates)
  n = robot.jointsPositions.shape[0]
  
  # Iterates through all colums (generalized coordinates)
  for i in range(n):     
    
    # Check in what row of Denavit Hartenberg Parameters Matrix is the current joint (the sum is because of the way Python indexes arrays)
    row, column = robot.whereIsTheJoint(i + 1)
    
    # Get pose of reference frame where current joint is attached
    H = fkHTM[row - 1]
    
    # Get axis of actuation of current joint
    z = trigsimp(H[0: 3, 2]) if symbolic else H[0: 3, 2]
    
    # Calculate angular velocity up to this point
    dw = nsimplify(dW[-1] + ((W[i + 1].cross(z)) * qd[i]) + (z * qdd[i]), tolerance = 1e-10) if symbolic else dW[-1] + (np.cross(W[i + 1], z, axis = 0) * qd[i]) + (z * qdd[i]).reshape((3, 1))

    # Append each calculated angular velocity
    dW.append(trigsimp(dw) if symbolic else dw)
      
  return dW

def linearAccelerationPropagation(robot : object, dv0 : np.array, W : list, dW : list, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes angular acceleration to the i-th reference frame of a serial robot given initial acceleration. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    dv0 (np.array): initial linear acceleration of the system (equals to zero if the robot's base is not mobile)
    W (np.array): angular velocities of the system (this have to be calculated with "angularVelocityPropagation" function)
    dW (np.array): angular accelerations of the system (this have to be calculated with "angularAccelerationPropagation" function)
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    dV (np.array): linear acceleration of each reference frame attached to joints (numerical)
    dV (SymPy Matrix): linear acceleration of each reference frame attached to joints (symbolic)
  """
  
  # Initial conditions
  dV = [dv0]
  
  # Calculate forward kinematics to know the position and axis of actuation of each joint
  fkHTM = forwardHTM(robot, symbolic = symbolic)
  
  # Get number of joints (generalized coordinates)
  n = robot.jointsPositions.shape[0]
  
  # Iterates through all colums (generalized coordinates)
  for i in range(n):     
    
    # Check in what row of Denavit Hartenberg Parameters Matrix is the current joint (the sum is because of the way Python indexes arrays)
    row, column = robot.whereIsTheJoint(i + 1)
    
    # Get relative position of rigid body
    r = trigsimp(fkHTM[row][0 : 3, - 1] - fkHTM[row - 1][0 : 3, - 1]) if symbolic else fkHTM[row][0 : 3, - 1] - fkHTM[row - 1][0 : 3, - 1]
    
    # Calculate linear acceleration up to this point
    dv = nsimplify(dV[-1] + (dW[i + 1].cross(r)) + W[i + 1].cross((W[i + 1].cross(r))), tolerance = 1e-10) if symbolic else dV[-1] + (np.cross(dW[i + 1], r, axis = 0)) + np.cross(W[i + 1], np.cross(W[i + 1], r, axis = 0), axis = 0)

    # Append each calculated linear velocity
    dV.append(trigsimp(dv) if symbolic else dv)
      
  return dV

if __name__ == '__main__':
  
  """
    THIS SECTION IS FOR TESTING PURPOSES ONLY
  """
  
  print("Z")