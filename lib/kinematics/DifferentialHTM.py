# Access to parent folder to get its files
from audioop import cross
import keyword
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

def geometricDerivativeStateSpace(robot : object, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes derivative of state-space equation (using Geometric Jacobian Matrix) of a serial robot given joints velocities and accelerations. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    Xdd (np.array): derivative of state-space equation (X''(t), numerical)
    Xdd (SymPy Matrix): derivative state-space equation (X''(t), symbolic)
  """

  # Calculate Inertial Geometric Jacobian Matrix 
  J = geometricJacobian(robot, symbolic)
  
  # Calculate derivative of Inertial Geometric Jacobian Matrix 
  dJ = geometricJacobianDerivative(robot, symbolic = symbolic)
    
  # Calculate derivative of state-space equation (symbolic or numerical)
  Xdd = (dJ * robot.qdSymbolic) + (J * robot.qddSymbolic) if symbolic else (dJ.dot(robot.jointsVelocities)) + (J.dot(robot.jointsAccelerations))
    
  return Xdd

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

def geometricCOMDerivativeStateSpace(robot : object, COM : int, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes derivative of state-space equation (using Geometric Jacobian Matrix) for centers of mass of a serial robot given joints velocities and accelerations. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    Xdd (np.array): derivative of state-space equation (X''(t), numerical)
    Xdd (SymPy Matrix): derivative state-space equation (X''(t), symbolic)
  """

  # Calculate Inertial Geometric Jacobian Matrix to Center of Mass
  J = geometricJacobianCOM(robot, COM, symbolic)
    
  # Calculate derivative of Inertial Geometric Jacobian Matrix to Center of Mass
  dJ = geometricJacobianDerivativeCOM(robot, COM, symbolic = symbolic)
    
  # Calculate derivative of state-space equation (symbolic or numerical)
  Xdd = (dJ * robot.qdSymbolic) + (J * robot.qddSymbolic) if symbolic else (dJ.dot(robot.jointsVelocities)) + (J.dot(robot.jointsAccelerations))
    
  return Xdd

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

def velocityPropagation(robot : object, v0 : np.array, w0 : np.array, qd : np.array, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes both linear and angular velocity to the i-th reference frame of a serial robot given initial velocity. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    v0 (np.array): initial linear velocity of the system (equals to zero if the robot's base is not mobile)
    w0 (np.array): initial angular velocity of the system (equals to zero if the robot's base is not mobile)
    qd (np.array): velocities of each joint
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    V (np.array): velocity to each reference frame (numerical)
    V (SymPy Matrix): velocity to each reference frame (symbolic)
  """
  
  # Initial conditions
  V = [np.append(v0, w0, axis = 0)]
  
  # Calculate forward kinematics to know the position and axis of actuation of each joint
  fkHTM = forwardHTM(robot, symbolic)
  
  # Get number of reference frames
  m = robot.dhParameters.shape[0]
  
  # Iterates through all reference frames (excepting inertial one)
  for k in range(1, m):
    
    # Get Denavit - Hartenberg Parameters Matrix of current frame
    frame = robot.symbolicDHParameters[k, :]
    
    # Check if this frame contains any of the "n" joints
    containedJoints = np.in1d(robot.qSymbolic, frame)
    
    # Create relative position cross operator between i-th and i + 1 frames
    ri = crossMatrix(fkHTM[k][0 : 3, -1] - fkHTM[k - 1][0 : 3, -1], symbolic)
    
    # Relative position matrix between i-th and i + 1 frames
    Mi = Matrix([[eye(3), -ri], [zeros(3), eye(3)]]) if symbolic else np.append(np.append(np.eye(3), -ri, axis = 1), np.append(np.zeros((3, 3)), np.eye(3), axis = 1), axis = 0)
    
    # If any joint is in the current reference frame
    if any(element == True for element in containedJoints):
          
      # Get the number of the joint
      joint = np.where(containedJoints == True)[0][-1]
    
      # Get axis of actuation where joint is attached
      z = fkHTM[k - 1][0: 3, 2]
      
      # Relative velocity calculation depending on the joint's movement
      vJoint = Mi * Matrix([zeros(3, 1), z]) * qd[joint] if symbolic else Mi.dot(np.append(np.zeros((3, 1)), z.reshape((3, 1)), axis = 0)) * qd[joint]
      
    else:
      
      # Relative velocity calculation equals to zero (no effects caused by joints)
      vJoint = zeros(3, 1) if symbolic else np.zeros((3, 1))
    
    # Calculate Velocities (both linear and angular) up to this point
    v = trigsimp((Mi * V[-1]) + vJoint) if symbolic else (Mi.dot(V[-1])) + vJoint

    # Append each calculated velocity
    V.append(nsimplify(v.evalf(), tolerance = 1e-10) if symbolic else v)
      
  return V

def accelerationPropagation(robot : object, dv0 : np.array, dw0 : np.array, V : list, qd : np.array, qdd : np.array, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes both angular and linear acceleration to the i-th reference frame of a serial robot given initial acceleration. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    dv0 (np.array): initial linear acceleration of the system (equals to zero if the robot's base is not mobile)
    dw0 (np.array): initial angular acceleration of the system (equals to zero if the robot's base is not mobile)
    V (np.array): angular and linear velocities of the system (this have to be calculated with "velocityPropagation" function)
    qd (np.array): velocities of each joint
    qdd (np.array): accelerations of each joint
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    dV (np.array): acceleration to each reference frame (numerical)
    dV (SymPy Matrix): acceleration to each reference frame (symbolic)
  """
  
  # Initial conditions
  dV = [np.append(dv0, dw0, axis = 0)]
  
  # Calculate forward kinematics to know the position and axis of actuation of each joint
  fkHTM = forwardHTM(robot, symbolic)
  
  # Get number of reference frames
  m = robot.dhParameters.shape[0]
   
  # Iterates through all reference frames (excepting inertial one)
  for k in range(1, m):
    
    # Get Denavit - Hartenberg Parameters Matrix of current frame
    frame = robot.symbolicDHParameters[k, :]
    
    # Check if this frame contains any of the "n" joints
    containedJoints = np.in1d(robot.qSymbolic, frame)
    
    # Relative position between i-th and i + 1 frames
    r = fkHTM[k][0 : 3, -1] - fkHTM[k - 1][0 : 3, -1]
    
    # Create relative position cross operator between i-th and i + 1 frames
    ri = crossMatrix(r, symbolic)
    
    # Relative position matrix between i-th and i + 1 frames
    Mi = Matrix([[eye(3), -ri], [zeros(3), eye(3)]]) if symbolic else np.append(np.append(np.eye(3), -ri, axis = 1), np.append(np.zeros((3, 3)), np.eye(3), axis = 1), axis = 0)
    
    # If any joint is in the current reference frame
    if any(element == True for element in containedJoints):
      
      # Get the number of the joint
      joint = np.where(containedJoints == True)[0][-1]
    
      # Get axis of actuation where joint is attached
      z = fkHTM[k - 1][0: 3, 2]
      
      # Relative position matrix for non-linear terms
      M = Matrix([[zeros(3), ri * crossMatrix(z, symbolic)], [zeros(3), -crossMatrix(z, symbolic)]]) if symbolic else np.append(np.append(np.zeros((3, 3)), ri.dot(crossMatrix(z)), axis = 1), np.append(np.zeros((3, 3)), -crossMatrix(z), axis = 1), axis = 0)
      
      # Relative acceleration calculation for joints actuation
      dVjoint = (M * V[k] * qd[joint]) + (Mi * Matrix([zeros(3, 1), z]) * qdd[joint]) if symbolic else (M.dot(V[k]) * qd[joint]) + (Mi.dot(np.append(np.zeros((3, 1)), z.reshape((3, 1)), axis = 0)) * qdd[joint])
      
    else:
      
      # Relative acceleration calculation equals to zero (no effects caused by joints)
      dVjoint = zeros(6, 1) if symbolic else np.zeros((6, 1))
          
    # Calculate acceleration up to this point
    dv = trigsimp((Mi * dV[-1]) + dVjoint) if symbolic else Mi.dot(dV[-1]) + dVjoint + np.append((np.cross(V[k][3 : 6], np.cross(V[k][3 : 6], r, axis = 0), axis = 0)), np.zeros((3, 1)), axis = 0)

    # Append each calculated acceleration
    dV.append(nsimplify(dv.evalf(), tolerance = 1e-10) if symbolic else dv)
      
  return dV

def velocityPropagationCOM(robot : object, vCOM0 : np.array, wCOM0 : np.array, V : list, qd : np.array, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes both angular and linear velocities to the j-th center of mass of a serial robot given reference frames' ones. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    vCOM0 (np.array): initial linear velocity of the system (equals to zero if the robot's base is not mobile)
    wCOM0 (np.array): initial angular velocity of the system (equals to zero if the robot's base is not mobile)
    V (list): angular and linear velocities of the system (this have to be calculated with "velocityPropagation" function)
    qd (np.array): velocities of each joint
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    Vcom (np.array): angular and linear velocity of each center of mass (numerical)
    Vcom (SymPy Matrix): angular and linear velocity of each center of mass (symbolic)
  """
  
  # Initial conditions
  Vcom = [np.append(vCOM0, wCOM0, axis = 0)]
  
  # Calculate forward kinematics to know the position and axis of actuation of each joint
  fkHTM = forwardHTM(robot, symbolic)
  
  # Calculate forward kinematics to know the position and axis of actuation of each center of mass
  fkCOMHTM = forwardCOMHTM(robot, symbolic)
  
  # Get number of reference frames
  m = robot.dhParameters.shape[0]
  
  # Iterates through all reference frames (excepting inertial one)
  for k in range(1, m):
    
    # Get Denavit - Hartenberg Parameters Matrix of current frame
    frame = robot.symbolicDHParametersCOM[k, :]
    
    # Check if current frame contain any of the joints
    containedJoints = np.in1d(robot.qSymbolic, frame)
    
    # Check if current frame contains any of the centers of mass
    containedCOMs = np.in1d(robot.symbolicCOMs, frame)
    
    # If any joint is in the current reference frame
    if any(element == True for element in containedCOMs) and any(element == True for element in containedJoints):
      
      # Get the number of the center of mass (the sum is because of the way Python indexes arrays)
      COM = np.where(containedCOMs == True)[0][-1] + 1
      
      # Get the number of the associated joint
      joint = np.where(containedJoints == True)[0][-1]
    
      # Get relative position of center of mass with respect to the i-th joint
      ri = crossMatrix(fkCOMHTM[COM][0 : 3, -1] - fkHTM[joint][0 : 3, -1], symbolic)
      
      # Relative position matrix between i-th and center of mass frames
      Mi = Matrix([[eye(3), -ri], [zeros(3), eye(3)]]) if symbolic else np.append(np.append(np.eye(3), -ri, axis = 1), np.append(np.zeros((3, 3)), np.eye(3), axis = 1), axis = 0)
    
      # Get axis of actuation of the joint
      z = fkHTM[joint][0 : 3, 2]

      # Relative velocity calculation caused by the joint
      vJoint = Mi * Matrix([zeros(3, 1), z]) * qd[joint] if symbolic else Mi.dot(np.append(np.zeros((3, 1)), z.reshape((3, 1)), axis = 0)) * qd[joint]
      
      # Calculate velocity up to this point
      vCOM = trigsimp((Mi * V[k - 1]) + vJoint) if symbolic else Mi.dot(V[k - 1]) + vJoint
      
      # Append each calculated velocity
      Vcom.append(nsimplify(vCOM.evalf(), tolerance = 1e-10) if symbolic else vCOM)
    
    # If there's no joint in current frame
    elif any(element == True for element in containedCOMs):
      
      # Get relative position of center of mass with respect to the i-th joint
      ri = crossMatrix(fkCOMHTM[COM][0 : 3, -1] - fkHTM[k - 1][0 : 3, -1], symbolic)
      
      # Relative position matrix between i-th and center of mass frames
      Mi = Matrix([[eye(3), -ri], [zeros(3), eye(3)]]) if symbolic else np.append(np.append(np.eye(3), -ri, axis = 1), np.append(np.zeros((3, 3)), np.eye(3), axis = 1), axis = 0)
    
      # Calculate velocity up to this point
      vCOM = trigsimp(Mi * V[k - 1]) if symbolic else Mi.dot(V[k - 1])
      
      # Append each calculated velocity
      Vcom.append(nsimplify(vCOM.evalf(), tolerance = 1e-10) if symbolic else vCOM)
  
  return Vcom

def angularAccelerationPropagationCOM(robot : object, dwCOM0 : np.array, Wcom : list, dW : list, qd : np.array, qdd : np.array, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes angular acceleration to the j-th center of mass of a serial robot given initial acceleration. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    dwCOM0 (np.array): initial angular acceleration of the system (equals to zero if the robot's base is not mobile)
    Wcom (list): angular velocities of each center of mass (this have to be calculated with "angularVelocityPropagationCOM" function)
    dW (list): angular accelerations of the system (this have to be calculated with "angularAccelerationPropagation" function)
    qd (np.array): velocities of each joint
    qdd (np.array): accelerations of each joint
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    dWcom (np.array): angular acceleration to each center of mass (numerical)
    dWcom (SymPy Matrix): angular acceleration to each center of mass (symbolic)
  """
  
  # Initial conditions
  dWcom = [dwCOM0]
  
  # Calculate forward kinematics to know the position and axis of actuation of each center of mass
  fkCOMHTM = forwardCOMHTM(robot, symbolic)
  
  # Get number of reference frames
  m = robot.dhParameters.shape[0]
  
  # Iterates through all reference frames (excepting inertial one)
  for k in range(1, m):
    
    # Get Denavit - Hartenberg Parameters Matrix of current frame
    frame = robot.symbolicDHParametersCOM[k, :]
    
    # Check if current frame contain any of the joints
    containedJoints = np.in1d(robot.qSymbolic, frame)
    
    # Check if current frame contains any of the centers of mass
    containedCOMs = np.in1d(robot.symbolicCOMs, frame)
    
    # If any joint is in the current reference frame and also there is a center of mass
    if any(element == True for element in containedJoints) and any(element == True for element in containedCOMs):
          
      # Get the number of the center of mass (this is because of the way Python indexes arrays)
      COM = np.where(containedCOMs == True)[0][-1] + 1
      
      # Get the number of the associated joint
      joint = np.where(containedJoints == True)[0][-1]
    
      # Get axis of actuation where joint is attached
      z = fkCOMHTM[COM][0: 3, 2]
      
      # Relative angular acceleration calculation equals to ((w x z) * qdi) + (z * qddi)
      dwCOMJoint = ((Wcom[COM].cross(z)) * qd[joint]) + (z * qdd[joint]) if symbolic else (np.cross(Wcom[COM], z, axis = 0) * qd[joint]) + (z * qdd[joint]).reshape((3, 1))
      
      # Calculate angular velocity up to this point
      
      dwCOM = trigsimp(dW[k - 1] + dwCOMJoint) if symbolic else dW[k - 1] + dwCOMJoint.reshape((3, 1))

      # Append each calculated angular velocity
      dWcom.append(nsimplify(dwCOM.evalf(), tolerance = 1e-10) if symbolic else dwCOM)
    
    # Else, if there's no joint but a center of mass only
    elif any(element == True for element in containedCOMs):
      
      # Append previously calculated angular velocity
      dWcom.append(nsimplify(dwCOM.evalf(), tolerance = 1e-10) if symbolic else dwCOM)
  
  return dWcom

def linearAccelerationPropagationCOM(robot : object, dvCOM0 : np.array, Wcom : np.array, dWcom : np.array, dV : list, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes linear acceleration to the j-th center of mass of a serial robot given initial acceleration. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    dvCOM0 (np.array): initial linear acceleration of the system (equals to zero if the robot's base is not mobile)
    Wcom (list): angular velocities to each center of mass (this have to be calculated with "angularVelocityPropagationCOM" function)
    dWcom (list): angular accelerations to each center of mass (this have to be calculated with "angularAccelerationPropagationCOM" function)
    dV (list): linear velocities to each reference frame (this have to be calculated with "linearAccelerationPropagation" function)
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    dVcom (np.array): acceleration of each center of mass (numerical)
    dVcom (SymPy Matrix): acceleration of each center of mass (symbolic)
  """
  
  # Initial conditions
  dVcom = [dvCOM0]
  
  # Calculate forward kinematics to know the position and axis of actuation of each joint
  fkHTM = forwardHTM(robot, symbolic)
  
  # Calculate forward kinematics to know the position and axis of actuation of each center of mass
  fkCOMHTM = forwardCOMHTM(robot, symbolic)
  
  # Get number of reference frames
  m = robot.dhParameters.shape[0]
  
  # Iterates through all reference frames (excepting inertial one)
  for k in range(1, m):
        
    # Get Denavit - Hartenberg Parameters Matrix of current frame
    frame = robot.symbolicDHParametersCOM[k, :]
    
    # Check if current frame contains any of the centers of mass
    containedCOMs = np.in1d(robot.symbolicCOMs, frame)
    
    # If any joint is in the current reference frame and also there is a center of mass
    if any(element == True for element in containedCOMs):
          
      # Get the number of the center of mass (the sum is because of the way Python indexes arrays)
      COM = np.where(containedCOMs == True)[0][-1] + 1
      
      # Get relative position of center of mass
      rCOM = fkCOMHTM[COM][0 : 3, - 1] - fkHTM[k - 1][0 : 3, - 1]
      
      # Calculate linear velocity up to this point
      dvCOM = trigsimp(dV[k -1] + (dWcom[COM].cross(rCOM)) + (Wcom[COM].cross(Wcom[COM].cross(rCOM)))) if symbolic else dV[k - 1] + (np.cross(dWcom[COM], rCOM, axis = 0)) + (np.cross(Wcom[COM], np.cross(Wcom[COM], rCOM, axis = 0), axis = 0))

      # Append each calculated linear velocity
      dVcom.append(nsimplify(dvCOM.evalf(), tolerance = 1e-10) if symbolic else dvCOM)
      
  return dVcom

if __name__ == '__main__':
  
  """
    THIS SECTION IS FOR TESTING PURPOSES ONLY
  """
  
  print("Z")