# Access to parent folder to get its files
import sys, os
sys.path.append(sys.path[0].replace(r'/lib/kinematics', r''))

# Libraries
import numpy as np
from lib.kinematics.HTM import *
from lib.kinematics.DifferentialHTM import *
from lib.kinematics.DQ import *
from lib.movements.DQ import *
from sympy import *

"""
  UNDER DEVELOPMENT
"""

def dqVelocityPropagation(robot : object, w0 : np.array, qd : np.array, symbolic = False):
  """Using Dual Quaternions, this function computes dual velocity to the i-th reference frame of a serial robot given initial velocity. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    w0 (np.array): initial dual velocity of the system (equals to zero if the robot's base is not mobile)
    qd (np.array): velocities of each joint
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    W (np.array): dual velocity to each reference frame (numerical)
    W (SymPy Matrix): dual velocity to each reference frame (symbolic)
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
      
      # Relative dual velocity calculation equals to left(Q) * right(conjugate(Q)) * xi * qdi
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
    w = trigsimp((Mi * W[-1]) + (M* wJoint)) if symbolic else Mi.dot(W[-1]) + M.dot(wJoint.reshape((8, 1)))
    
    # Append each calculated angular velocity
    W.append(nsimplify(w.evalf(), tolerance = 1e-10) if symbolic else w)
      
  return W

if __name__ == '__main__':
  
  """
    THIS SECTION IS FOR TESTING PURPOSES ONLY
  """
  
  print("Z")