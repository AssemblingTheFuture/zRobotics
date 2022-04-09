# Access to parent folder to get its files
import sys, os
sys.path.append(sys.path[0].replace(r'/lib/kinematics', r''))

# Libraries
import numpy as np
from lib.kinematics.DQ import *
from lib.movements.DQ import *
from sympy import *

def dqVelocityPropagation(robot : object, w0 : np.array, qd : np.array, symbolic = False):
  """Using Dual Quaternions, this function computes velocity (both linear and angular) to the i-th reference frame of a serial robot given initial velocity. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    w0 (np.array): initial velocity of the system (equals to zero if the robot's base is not mobile)
    qd (np.array): velocities of each joint
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    W (np.array): velocity to each reference frame (numerical)
    W (SymPy Matrix): velocity to each reference frame (symbolic)
  """
  
  # Initial conditions
  W = [w0]
  
  # Calculate forward kinematics to know the position and axis of actuation of each joint
  fkDQ = forwardDQ(robot, symbolic)
  
  # Get number of reference frames
  m = robot.dhParameters.shape[0]
  
  # Iterates through all reference frames (excepting inertial one)
  for k in range(1, m):
    
    # Get Denavit - Hartenberg Parameters Matrix of current frame
    frame = robot.symbolicDHParameters[k, :]
    
    # Check if this frame contains any of the "n" joints
    containedJoints = np.in1d(robot.qSymbolic, frame)
    
    # If any joint is in the current reference frame
    if any(element == True for element in containedJoints):
          
      # Get the number of the joint
      joint = np.where(containedJoints == True)[0][-1]
    
      # Get pose of reference frame where joint is attached
      Q = fkDQ[k - 1]
      
      # Get axis of actuation of the joint (screw vector)
      xi = robot.xi[:, joint].reshape((8, 1))
      
      # Relative velocity calculation equals to left(Q) * right(conjugate(Q)) * xi * qdi
      wJoint = dqMultiplication(dqMultiplication(Q, xi, symbolic), conjugateDQ(Q, symbolic), symbolic) * qd[joint] if symbolic else dqMultiplication(dqMultiplication(Q, xi), conjugateDQ(Q)) * qd[joint]
      
    else:
      
      # Relative angular velocity calculation equals to zero (no effects caused by joints)
      wJoint = zeros(8, 1) if symbolic else np.zeros((8, 1))
    
    # Create relative position cross operator between i-th and i + 1 frames
    ri = crossOperatorExtension(dqToR3(fkDQ[k], symbolic) - dqToR3(fkDQ[k - 1], symbolic), symbolic)
    
    # Relative position matrix between i-th and i + 1 frames
    Mi = Matrix([[eye(4), zeros(4)], [-ri, eye(4)]]) if symbolic else np.append(np.append(np.eye(4), np.zeros((4, 4)), axis = 1), np.append(-ri, np.eye(4), axis = 1), axis = 0)
    
    # Create position cross operator for i + 1 reference frame
    r = crossOperatorExtension(dqToR3(fkDQ[k], symbolic), symbolic)
    
    # Create inverse position matrix for i + 1 reference frame
    M = Matrix([[eye(4), zeros(4)], [-r, eye(4)]]) if symbolic else np.append(np.append(np.eye(4), np.zeros((4, 4)), axis = 1), np.append(-r, np.eye(4), axis = 1), axis = 0)
    
    # Calculate angular velocity up to this point
    w = trigsimp((Mi * W[-1]) + (M * wJoint)) if symbolic else Mi.dot(W[-1]) + M.dot(wJoint)
    
    # Append each calculated angular velocity
    W.append(nsimplify(w.evalf(), tolerance = 1e-10) if symbolic else w)
      
  return W

def dqAccelerationPropagation(robot : object, dw0 : np.array, Wdq : np.array, qd : np.array, qdd : np.array, symbolic = False):
  """Using Dual Quaternions, this function computes acceleration (both linear and angular) to the i-th reference frame of a serial robot given initial acceleration. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    dw0 (np.array): initial acceleration of the system (equals to zero if the robot's base is not mobile)
    Wdq (np.array): inertial velocity of the system using dual quaternions (equals to zero if the robot's base is not mobile)
    qd (np.array): velocities of each joint
    qdd (np.array): accelerations of each joint
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    dW (np.array): acceleration to each reference frame (numerical)
    dW (SymPy Matrix): acceleration to each reference frame (symbolic)
  """
  
  # Initial conditions
  dW = [dw0]
  
  # Calculate forward kinematics to know the position and axis of actuation of each joint
  fkDQ = forwardDQ(robot, symbolic)
  
  # Get number of reference frames
  m = robot.dhParameters.shape[0]
  
  # Iterates through all reference frames (excepting inertial one)
  for k in range(1, m):
    
    # Get Denavit - Hartenberg Parameters Matrix of current frame
    frame = robot.symbolicDHParameters[k, :]
    
    # Check if this frame contains any of the "n" joints
    containedJoints = np.in1d(robot.qSymbolic, frame)
    
    # If any joint is in the current reference frame
    if any(element == True for element in containedJoints):
          
      # Get the number of the joint
      joint = np.where(containedJoints == True)[0][-1]
    
      # Get pose of reference frame where joint is attached
      Q = fkDQ[k - 1]
      
      # Get axis of actuation of the joint (screw vector)
      xi = robot.xi[:, joint].reshape((8, 1))
      
      # Get derivative of axis of actuation of the joint (screw vector)
      xid = robot.xid[:, joint].reshape((8, 1))
           
      # Relative acceleration calculation equals to left(Q) * right(conjugate(Q)) * (xid * qdi + xi * qddi)
      dwJoint = dqMultiplication(dqMultiplication(Q, (xid * qd[joint]) + (xi * qdd[joint]), symbolic), conjugateDQ(Q, symbolic), symbolic) if symbolic else dqMultiplication(dqMultiplication(Q, (xid * qd[joint]) + (xi * qdd[joint])), conjugateDQ(Q))
      
      # Create cross operator for the position of the i-th reference frame
      ri = crossOperatorExtension(dqToR3(fkDQ[k - 1], symbolic), symbolic)
      
      # Create inverse position matrix for i-th reference frame
      Mi = Matrix([[eye(4), zeros(4, 1)], [ri, eye(4)]]) if symbolic else np.append(np.append(np.eye(4), np.zeros((4, 4)), axis = 1), np.append(ri, np.eye(4), axis = 1), axis = 0)
      
      # Dual velocity of the i-th frame seen from itself
      dualW = dqMultiplication(dqMultiplication(conjugateDQ(Q, symbolic), Mi * Wdq[k - 1], symbolic), Q, symbolic) if symbolic else dqMultiplication(dqMultiplication(conjugateDQ(Q), Mi.dot(Wdq[k - 1])), Q)
      
      # Centripetal effect of the joint with respect to the inertial frame
      dwCentripetalJoint = dqMultiplication(dqMultiplication(Q, dualCrossOperator(dualW, symbolic) * xi * qd[joint], symbolic), conjugateDQ(Q, symbolic), symbolic) if symbolic else dqMultiplication(dqMultiplication(Q, dualCrossOperator(dualW).dot(xi * qd[joint])), conjugateDQ(Q))
      
    else:
      
      # Relative angular acceleration calculation equals to zero (no effects caused by joints)
      dwJoint = zeros(8, 1) if symbolic else np.zeros((8, 1))
      
      # Centripetal acceleration calculation to i-th frame (with respect to inertial one) equals to zero (no effects caused by joints)
      dwCentripetalJoint = zeros(8, 1) if symbolic else np.zeros((8, 1))
    
    # Create relative position cross operator between i-th and i + 1 frames
    ri = crossOperatorExtension(dqToR3(fkDQ[k], symbolic) - dqToR3(fkDQ[k - 1], symbolic), symbolic)
    
    # Relative position matrix between i-th and i + 1 frames
    Mi = Matrix([[eye(4), zeros(4)], [-ri, eye(4)]]) if symbolic else np.append(np.append(np.eye(4), np.zeros((4, 4)), axis = 1), np.append(-ri, np.eye(4), axis = 1), axis = 0)
    
    # Create position cross operator for i + 1 reference frame
    r = crossOperatorExtension(dqToR3(fkDQ[k], symbolic), symbolic)
    
    # Create inverse position matrix for i + 1 reference frame
    M = Matrix([[eye(4), zeros(4, 1)], [-r, eye(4)]]) if symbolic else np.append(np.append(np.eye(4), np.zeros((4, 4)), axis = 1), np.append(-r, np.eye(4), axis = 1), axis = 0)
    
    # Centripetal effects
    dwCentripetal = np.append(np.zeros((4, 1)), crossOperatorExtension(Wdq[k][0 : 4, :]).dot(Wdq[k][4 : 8, :]) - crossOperatorExtension(Wdq[k - 1][0 : 4, :]).dot(Wdq[k - 1][4 : 8, :]), axis = 0)
    
    # Calculate angular velocity up to this point
    dw = trigsimp((Mi * dW[-1]) + dwCentripetal + M * (dwCentripetalJoint + dwJoint)) if symbolic else Mi.dot(dW[-1]) + dwCentripetal + M.dot(dwCentripetalJoint + dwJoint)
    
    # Append each calculated angular velocity
    dW.append(nsimplify(dw.evalf(), tolerance = 1e-10) if symbolic else dw)
      
  return dW

"""
  UNDER DEVELOPMENT
"""

if __name__ == '__main__':
  
  """
    THIS SECTION IS FOR TESTING PURPOSES ONLY
  """
  
  print("Z")