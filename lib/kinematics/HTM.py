# Access to parent folder to get its files
import sys, os

from pandas import array
sys.path.append(sys.path[0].replace(r'/lib/kinematics', r''))

# Libraries
import numpy as np
from lib.movements.HTM import *
from lib.dynamics.Solver import *
from sympy import *

def forwardHTM(robot : object, symbolic = False):
    """Using Homogeneous Transformation Matrices, this function computes forward kinematics of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

    Args:
        robot (Serial): serial robot (this won't work with other type of robots)
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        framesHTM (list): Homogeneous Transformation Matrices with frames' poses (numerical or symbolical)
    """

    # Initial conditions
    framesHTM = []

    if symbolic:
      
      # Get Denavit - Hartenberg Parameters Matrix
      DH = robot.symbolicDHParameters
    
    else:
      
      # Update Denavit - Hartenberg Parameters Matrix
      robot.denavitHartenberg()
    
      # Get Denavit - Hartenberg Matrix
      DH = robot.dhParameters

    # Create Homogeneous Transformation Matrix for Inertial Frame
    fkHTM = eye(4) if symbolic else np.identity(4)
    
    # Iteration through all the rows in Denavit - Hartenberg Matrix
    for frame in range(DH.rows) if symbolic else DH:
        
      # Operates matrices: Rz * Tz * Tx * Rx
      fkHTM = trigsimp(fkHTM * rz(DH[frame, 0], symbolic) * tz(DH[frame, 1], symbolic) * tx(DH[frame, 2], symbolic) * rx(DH[frame, 3], symbolic)) if symbolic else fkHTM.dot(rz(frame[0]).dot(tz(frame[1])).dot(tx(frame[2])).dot(rx(frame[3])))

      # Append each calculated Homogeneous Transformation Matrix
      framesHTM.append(nsimplify(fkHTM.evalf(), tolerance = 1e-10) if symbolic else fkHTM)

    return framesHTM

def forwardCOMHTM(robot : object, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes forward kinematics of a serial robot's centers of mass given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (Serial): serial robot (this won't work with other type of robots)
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    framesHTM (list): Homogeneous Transformation Matrices with COMs' poses (numerical or symbolical)
  """
    
  # Calculate forward kinematics
  framesHTM = forwardHTM(robot, symbolic)

  # Initial conditions
  framesCOMHTM = [eye(4) if symbolic else np.identity(4)]
  
  if symbolic:
      
    # Get Denavit - Hartenberg Parameters Matrix
    comDH = robot.symbolicDHParametersCOM
    
  else:
      
    # Update Denavit - Hartenberg Parameters Matrix
    robot.denavitHartenbergCOM()
    
    # Get Denavit - Hartenberg Matrix
    comDH = robot.dhParametersCOM
    
  # Iteration through all the Centers of Mass
  for i in range(robot.symbolicCOMs.shape[0]):
    
    # Check where is the current Center of Mass
    rowCOM, column = robot.whereIsTheCOM(COM = i + 1)
    
    # Center of Mass Homogeneous Transformation Matrix 
    COM = rz(comDH[rowCOM, 0] if column >= 0 else 0, symbolic) * tz(comDH[rowCOM, 1] if column >= 1 else 0, symbolic) * tx(comDH[rowCOM, 2] if column >= 2 else 0, symbolic) * rx(comDH[rowCOM, 3] if column >= 3 else 0, symbolic) if symbolic else rz(comDH[rowCOM, 0] if column >= 0 else 0).dot(tz(comDH[rowCOM, 1] if column >= 1 else 0)).dot(tx(comDH[rowCOM, 2] if column >= 2 else 0)).dot(rx(comDH[rowCOM, 3] if column >= 3 else 0))
    
    # Forward kinematics to Center of Mass
    fkCOMHTM = trigsimp(framesHTM[rowCOM - 1] * COM) if symbolic else framesHTM[rowCOM - 1].dot(COM)
    
    # Append results
    framesCOMHTM.append(nsimplify(fkCOMHTM.evalf(), tolerance = 1e-10) if symbolic else fkCOMHTM)
    
  return framesCOMHTM

def axisAngle(H : np.array, symbolic = False):
  """This function computes the axis - angle vector «X» using the Homogeneous Transformation Matrix of a reference frame

  Args:
    H (np.array): Homogeneous Transformation Matrix (numerical)
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    X (NumPy Array): Axis - Angle vector (numerical)
    X (SymPy Matrix): Axis - Angle vector (symbolical)
  """
  
  # Calculate angle of rotation
  theta = acos((H[0 : 3, 0 : 3].trace() - 1) / 2) if symbolic else np.arccos((np.trace(H[0 : 3, 0 : 3]) - 1)/2)
  
  # Calculate axis of rotation
  n = (1 / (2 * sin(theta))) * Matrix([[H[2, 1] - H[1, 2]],
                                       [H[0, 2] - H[2, 0]],
                                       [H[1 ,0] - H[0, 1]]]) if symbolic else (1/(2 * np.sin(theta))) * np.array([[H[2, 1] - H[1, 2]],
                                                                                                                  [H[0, 2] - H[2, 0]],
                                                                                                                  [H[1 ,0] - H[0, 1]]])
  
  # Append position and orientation in one single vector
  X = H[0 : 3, 3].row_insert(3, trigsimp(theta * n)) if symbolic else np.append(H[0 : 3, 3], theta * n)
  
  return nsimplify(X.evalf(), tolerance = 1e-10) if symbolic else X.reshape((6, 1))

def geometricJacobian(robot : object, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes Geometric Jacobian Matrix of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (Serial): serial robot (this won't work with other type of robots)
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    J (np.array): Geometric Jacobian Matrix (numerical)
    J (SymPy Matrix): Geometric Jacobian Matrix (symbolical)
  """

  # Get number of joints (generalized coordinates)
  n = robot.jointsPositions.shape[0]
  
  # Calculate forward kinematics
  fkHTM = forwardHTM(robot, symbolic)
  
  # Initializes jacobian matrix with zeros
  J = zeros(6, n) if symbolic else np.zeros((6, n))
  
  # Iterates through all colums (generalized coordinates)
  for j in range(n):
    
    # Check in what row of Denavit Hartenberg Parameters Matrix is the current joint (the sum is because of the way Python indexes arrays)
    row, column = robot.whereIsTheJoint(j + 1)
    
    # Get row where joint is stored
    frame = robot.symbolicDHParameters[4 * (row) : 4 * (row + 1)] if symbolic else robot.dhParameters[row, :]
    
    # Get pose of the joint
    H = fkHTM[row - 1] * rz(frame[0] if column >= 0 else 0, symbolic) * tz(frame[1] if column >= 1 else 0, symbolic) * tx(frame[2] if column >= 2 else 0) * rx(frame[3] if column >= 3 else 0)  if symbolic else fkHTM[row - 1].dot(rz(frame[0] if column >= 0 else 0)).dot(tz(frame[1] if column >= 1 else 0)).dot(tx(frame[2] if column >= 2 else 0, symbolic)).dot(rx(frame[3] if column >= 3 else 0))
    
    # Get axis of actuation of current joint
    z = H[0: 3, 2]
      
    # Calculate distance between end - effector and current joint
    r = fkHTM[-1][0: 3, 3] - H[0: 3, 3]
    
    # Calculate axes of actuation of Center of Mass or End - Effector caused by current joint
    J[0: 3, j] = nsimplify(trigsimp(z.cross(r)).evalf(), tolerance = 1e-10) if symbolic else np.cross(z, r)
    
    # Set axis of actuation
    J[3: 6, j] = nsimplify(trigsimp(z).evalf(), tolerance = 1e-10) if symbolic else z
    
  return J

def geometricJacobianDerivative(robot : object, dq = 0.001, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes the derivative of Geometric Jacobian Matrix of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (Serial): serial robot (this won't work with other type of robots)
    dq (float, optional): step size for numerical derivative. Defaults to 0.001.
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    dJ (np.array): Derivative of Geometric Jacobian Matrix (numerical)
    dJ (SymPy Matrix): Derivative of Geometric Jacobian Matrix (symbolical)
  """

  # Get number of joints (generalized coordinates)
  n = robot.jointsPositions.shape[0]
  
  # Auxiliar variable to keep original joints positions
  q = robot.jointsPositions.copy()
  
  # Initializes auxiliar derivative matrix with zeros
  V = zeros(6, n) if symbolic else np.zeros((6, n))
  
  # Derivative of Jacobian Matrix
  dJ = zeros(6, n) if symbolic else np.zeros((6, n))
  
  # Calculate jacobian matrix
  J = geometricJacobian(robot, symbolic)
  
  # Iterates through all colums (generalized coordinates)
  for j in range(n):
    
    # If symbolic calculation was requested
    if symbolic:
            
      # Differentiates current column with respect to joints positions
      V = J[:, j].jacobian(robot.qSymbolic)

    # Else, calculate derivative numerically
    else:
            
      # Iterates through all the generalized coordinates to calculate the derivative of current column
      for k in range(n):
                  
        # Set increment to current generalized coordinate: z[j] = q[j] + dq
        robot.jointsPositions[k] += dq
                      
        # Calculate geometric jacobian matrix with current step size
        Ji = geometricJacobian(robot, symbolic)
                      
        # Calculate derivative: [Ji[:, j](q + dq) - J[:, j](q)] / dq
        V[:, k] = ((Ji[:, j] - J[:, j]) / dq)
                      
        # Eliminates step size by copying original values from auxiliar variable
        robot.jointsPositions[:, :] = q

    # Sum the previous derivative to get the derivative of jacobian matrix and multiply it by qi'(t)
    dJ += V * robot.qdSymbolic[j] if symbolic else V * robot.jointsVelocities[j]
    
  return dJ

def geometricJacobianCOM(robot : object, COM: int, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes Geometric Jacobian Matrix to a desired Center of Mass of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (Serial): serial robot (this won't work with other type of robots)
    COM (int): center of mass that will be analyzed
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    J (np.array): Geometric Jacobian Matrix (numerical)
    J (SymPy Matrix): Geometric Jacobian Matrix (symbolical)
  """

  # Get number of joints (generalized coordinates)
  n = robot.jointsPositions.shape[0]
  
  # Calculate forward kinematics
  fkHTM = forwardHTM(robot, symbolic)
  
  # Calculate forward kinematics to each Center of Mass
  fkCOMHTM = forwardCOMHTM(robot, symbolic)
    
  # Check in what row of Denavit Hartenberg Parameters Matrix is the Center of Mass (the sum is because of the way Python indexes arrays)
  rowCOM = robot.whereIsTheCOM(COM)[0]
  
  # Initializes jacobian matrix with zeros
  J = zeros(6, n) if symbolic else np.zeros((6, n))
  
  # Iterates through all colums (generalized coordinates)
  for i in range(n):
    
    # Check in what row of Denavit Hartenberg Parameters Matrix is the current joint (the sum is because of the way Python indexes arrays)
    row, column = robot.whereIsTheJoint(i + 1)
    
    # If the current joint is coupled after the desired center of mass, and any Center of Mass is analyzed
    if row > rowCOM:
      
      # Stop the algorithm, because current joint won't affect the desired center of mass
      break
    
    # Get row where joint is stored
    frame = robot.symbolicDHParameters[4 * (row) : 4 * (row + 1)] if symbolic else robot.dhParameters[row, :]
    
    # Get pose of joint
    H = fkHTM[row - 1] * rz(frame[0] if column >= 0 else 0, symbolic) * tz(frame[1] if column >= 1 else 0, symbolic) * tx(frame[2] if column >= 2 else 0) * rx(frame[3] if column >= 3 else 0) if symbolic else fkHTM[row - 1].dot(rz(frame[0] if column >= 0 else 0)).dot(tz(frame[1] if column >= 1 else 0)).dot(tx(frame[2] if column >= 2 else 0, symbolic)).dot(rx(frame[3] if column >= 3 else 0))
    
    # Get axis of actuation of current joint
    z = H[0: 3, 2]
      
    # Calculate distance between Desired Center of Mass and current joint
    r = fkCOMHTM[COM][0: 3, 3] - H[0: 3, 3]
    
    # Calculate axes of actuation of Center of Mass or End - Effector caused by current joint
    J[0: 3, i] = nsimplify(trigsimp(z.cross(r)).evalf(), tolerance = 1e-10) if symbolic else np.cross(z, r)
    
    # Set axis of actuation
    J[3: 6, i] = nsimplify(trigsimp(z).evalf(), tolerance = 1e-10) if symbolic else z
    
  return J

def geometricJacobianDerivativeCOM(robot : object, COM : int, dq = 0.001, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes the derivative of Geometric Jacobian Matrix of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (Serial): serial robot (this won't work with other type of robots)
    COM (int): center of mass that will be analyzed
    dq (float, optional): step size for numerical derivative. Defaults to 0.001.
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    dJ (np.array): Derivative of Geometric Jacobian Matrix for a Center of Mass (numerical)
    dJ (SymPy Matrix): Derivative of Geometric Jacobian Matrix for a Center of Mass (symbolical)
  """

  # Get number of joints (generalized coordinates)
  n = robot.jointsPositions.shape[0]
  
  # Auxiliar variable to keep original joints positions
  q = robot.jointsPositions.copy()
  
  # Initializes auxiliar derivative matrix with zeros
  V = zeros(6, n) if symbolic else np.zeros((6, n))
  
  # Derivative of Jacobian Matrix
  dJ = zeros(6, n) if symbolic else np.zeros((6, n))
  
  # Calculate jacobian matrix
  J = geometricJacobianCOM(robot, COM, symbolic)
  
  # Iterates through all colums (generalized coordinates)
  for j in range(n):
    
    # If symbolic calculation was requested
    if symbolic:
            
      # Differentiates current column with respect to joints positions
      V = J[:, j].jacobian(robot.qSymbolic)

    # Else, calculate derivative numerically
    else:
            
      # Iterates through all the generalized coordinates to calculate the derivative of current column
      for k in range(n):
                  
        # Set increment to current generalized coordinate: z[j] = q[j] + dq
        robot.jointsPositions[k] += dq
                      
        # Calculate geometric jacobian matrix with current step size
        Ji = geometricJacobianCOM(robot, COM, symbolic)
                      
        # Calculate derivative: [Ji[:, j](q + dq) - J[:, j](q)] / dq
        V[:, k] = ((Ji[:, j] - J[:, j]) / dq)
                      
        # Eliminates step size by copying original values from auxiliar variable
        robot.jointsPositions[:, :] = q

    # Sum the previous derivative to get the derivative of jacobian matrix and multiply it by qi'(t)
    dJ += V * robot.qdSymbolic[j] if symbolic else V * robot.jointsVelocities[j]
    
  return dJ

def analyticJacobian(robot : object, dq = 0.001, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes Analytic Jacobian Matrix of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (Serial): serial robot (this won't work with other type of robots)
    dq (float, optional): step size for numerical derivative. Defaults to 0.001.
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    J (np.array): Inertial Analytic Jacobian Matrix (numerical)
    J (SymPy Matrix): Inertial Analytic Jacobian Matrix (symbolical)
  """
  
  # Calculate forward kinematics: f(q)
  fkHTM = forwardHTM(robot, symbolic)
  
  # Convert result into an Axis - Angle vector: x(q)
  x = axisAngle(fkHTM[-1], symbolic)
  
  if symbolic:
    
    # Calculate Analytic Jacobian Matrix by differentiating Axis - Angle vector with SymPy functions
    return nsimplify(trigsimp(x.jacobian(robot.qSymbolic)).evalf(), tolerance = 1e-10)
    
  else:
        
    # Get number of joints (generalized coordinates)
    n = robot.jointsPositions.shape[0]
  
    # Initializes jacobian matrix with zeros
    J = np.zeros((6, n))
    
    # Auxiliar variable to keep original joints positions
    q = robot.jointsPositions.copy()
    
    # Iterates through all colums (generalized coordinates)
    for j in range(n):
      
      # Set increment to current generalized coordinate: z[j] = q[j] + dq
      robot.jointsPositions[j] += dq
        
      # Calculate forward kinematics with step size: f(z) = f(q + dq)
      f = forwardHTM(robot)
      
      # Convert result into an Axis - Angle vector: X(q + dq)
      X = axisAngle(f[-1])
      
      # Calculate analytic jacobian matrix: [X(q + dq) - x(q)] / dq
      J[: , j] = ((X - x) / dq).flatten()
      
      # Eliminates step size by copying original values from auxiliar variable
      robot.jointsPositions[:, :] = q
    
    return J

def analyticJacobianCOM(robot : object, COM : int, dq = 0.001, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes Analytic Jacobian Matrix to a Center of Mass of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (Serial): serial robot (this won't work with other type of robots)
    COM (int): center of mass that will be analyzed
    dq (float, optional): step size for numerical derivative. Defaults to 0.001.
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    J (np.array): Inertial Analytic Jacobian Matrix to Center of Mass (numerical)
    J (SymPy Matrix): Inertial Analytic Jacobian Matrix to Center of Masas (symbolical)
  """
  
  # Calculate forward kinematics: f(q)
  fkCOMHTM = forwardCOMHTM(robot, symbolic)
  
  # Convert result into an Axis - Angle vector: x(q)
  x = axisAngle(fkCOMHTM[COM], symbolic)
  
  if symbolic:
    
    # Calculate Analytic Jacobian Matrix by differentiating Axis - Angle vector with SymPy functions
    return nsimplify(trigsimp(x.jacobian(robot.qSymbolic)).evalf(), tolerance = 1e-10)
    
  else:
        
    # Get number of joints (generalized coordinates)
    n = robot.jointsPositions.shape[0]
  
    # Initializes jacobian matrix with zeros
    J = np.zeros((6, n))
    
    # Auxiliar variable to keep original joints positions
    q = robot.jointsPositions.copy()
    
    # Iterates through all colums (generalized coordinates)
    for j in range(n):
      
      # Set increment to current generalized coordinate: z[j] = q[j] + dq
      robot.jointsPositions[j] += dq
        
      # Calculate forward kinematics with step size: f(z) = f(q + dq)
      f = forwardCOMHTM(robot)
      
      # Convert result into an Axis - Angle vector: X(q + dq)
      X = axisAngle(f[COM])
      
      # Calculate analytic jacobian matrix: [X(q + dq) - x(q)] / dq
      J[: , j] = ((X - x) / dq).flatten()
      
      # Eliminates step size by copying original values from auxiliar variable
      robot.jointsPositions[:, :] = q
    
    return J

def inverseHTM(robot : object, q0 : np.array, Hd : np.array, K : np.array, jacobian = 'geometric'):
  """Using Homogeneous Transformation Matrices, this function computes Inverse Kinematics of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (Serial): serial robot (this won't work with other type of robots)
    q0 (np.array): initial conditions of joints
    Hd (np.array): desired pose represented with an Homogeneous Transformation Matrix or Axis - Angle vector
    K (np.array): gain matrix to guarantee stability
    jacobian (str, optional): 'geometric' or 'analytic'

  Returns:
    q (np.array): joints positions to reach desired pose
  """
  
  # Get the shape of Hd
  r, s = Hd.shape
  
  # Convert desired pose to Axis - Angle Vector if this is an Homogeneous Transformation Matrix
  Xd = axisAngle(Hd) if r == 4 else Hd
  
  # Extract rotation matrix from desired pose and convert it to Homogeneous Transformation Matrix
  Rd = np.append(np.append(Hd[0 : 3, 0 : 3], np.zeros((3, 1)), axis = 1), np.array([[0, 0, 0, 1]]), axis = 0)
  
  # Set initial conditions
  q = q0.reshape(q0.shape)
  
  # Auxiliar variable to keep original joints positions
  z = robot.jointsPositions.copy()
  
  # Iterator
  j = 0
  
  # While iterator is less than 15000 or absolute value of max
  while j < 15000:
    
    # Set calculated positions on robot
    robot.jointsPositions = q[:, -1].reshape(q0.shape)
    
    # Calculate forward kinematics
    fkH = forwardHTM(robot)
    
    # Set Axis - Angle vector
    X = axisAngle(fkH[-1])
    
    # Extract rotation matrix from current pose and convert it to Homogeneous Transformation Matrix
    R = np.append(np.append(fkH[-1][0 : 3, 0 : 3], np.zeros((3, 1)), axis = 1), np.array([[0, 0, 0, 1]]), axis = 0)
    
    # Calculate error
    e = np.append(Xd[0 : 3, :] - X[0 : 3, :], axisAngle(Rd.dot(R.T))[3 : 6], axis = 0)
    
    # If error is less equal to 0.001
    if np.abs(np.max(e)) <= 0.001:
      
      # Solution found!
      print("\nSolution found using Homogeneous Transformation Matrices!")
      
      # Stop loop
      break
    
    # Calculate Jacobian Matrix
    J = geometricJacobian(robot) if jacobian == 'geometric' else analyticJacobian(robot) if jacobian == 'analytic' else np.zeros((6, q0.shape[0]))
    
    # Calculate new joints positions
    solution = rungeKutta4(f = (np.linalg.pinv(J)).dot(K).dot(e), F = q[:, -1].reshape(q0.shape))
    
    # Append results
    q = np.append(q, solution, axis = 1)
    
    j += 1
  
  # Restore original joints positions to don't affect other calculations
  robot.jointsPositions = z.copy()
     
  return q

if __name__ == '__main__':
  
  """
    THIS SECTION IS FOR TESTING PURPOSES ONLY
  """
  
  print("Z")