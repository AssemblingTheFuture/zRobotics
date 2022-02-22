# Access to parent folder to get its files
import sys, os
sys.path.append(sys.path[0].replace(r'/lib/kinematics', r''))

# Libraries
import numpy as np
from lib.kinematics.HTM import *
from sympy import *

def geometricStateSpace(robot, qd, symbolic = False):
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
  Xd = nsimplify(simplify(J * qd), tolerance = 1e-10, rational = False) if symbolic else J.dot(qd)
    
  return Xd

def analyticStateSpace(robot, qd, dq = 0.001, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes state-space equation (using Analytic Jacobian Matrix) of a serial robot given joints velocities. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    qd (np.array): joints velocities
    dq (float, optional): step size for numerical derivative. Defaults to 0.001.
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    Xd (np.array): state-space equation (X'(t), numerical)
    Xd (SymPy Matrix): state-space equation (X'(t), symbolic)
  """

  # Calculate Inertial Geometric Jacobian Matrix 
  J = analyticJacobian(robot, dq, symbolic)
    
  # Calculate state-space equation (symbolic or numerical)
  Xd = nsimplify(simplify(J * qd), tolerance = 1e-10, rational = False) if symbolic else J.dot(qd)
    
  return Xd

if __name__ == '__main__':
  
  """
    THIS SECTION IS FOR TESTING PURPOSES ONLY
  """
  
  print("Z")