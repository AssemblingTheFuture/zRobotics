# Access to parent folder to get its files
import sys, os
sys.path.append(sys.path[0].replace(r'/lib/kinematics', r''))

# Libraries
import numpy as np
from lib.movements.DQ import *
from lib.dynamics.Solver import *
from sympy import *

def forwardDQ(robot : object, symbolic = False):
  """Using Dual Quaternions, this function computes forward kinematics of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    framesDQ (list): Dual Quaternions with frames' poses (numeric or symbolic)
  """
  
  # Initial conditions
  framesDQ = []

  if symbolic:
      
    # Get Denavit - Hartenberg Parameters Matrix
    DH = robot.symbolicDHParameters
    
  else:
      
    # Update Denavit - Hartenberg Parameters Matrix
    robot.denavitHartenberg()
    
    # Get Denavit - Hartenberg Matrix
    DH = robot.dhParameters

  # Create Dual Quaternion for Inertial Frame
  fkDQ = Matrix([1, 0, 0, 0, 0, 0, 0, 0]) if symbolic else np.array([[1, 0, 0, 0, 0, 0, 0, 0]]).T
  
  # Iteration through all the rows in Denavit - Hartenberg Matrix
  for frame in range(DH.rows) if symbolic else DH:

    # Operates dual quaternions: Rz * Tz * Tx * Rx
    fkDQ = trigsimp(dqMultiplication(dqMultiplication(dqMultiplication(dqMultiplication(fkDQ, dqRz(DH[frame, 0], symbolic), symbolic), dqTz(DH[frame, 1], symbolic), symbolic), dqTx(DH[frame, 2], symbolic), symbolic), dqRx(DH[frame, 3], symbolic), symbolic)) if symbolic else dqMultiplication(dqMultiplication(dqMultiplication(dqMultiplication(fkDQ, dqRz(frame[0])), dqTz(frame[1])), dqTx(frame[2])), dqRx(frame[3]))

    # Append each calculated Homogeneous Transformation Matrix
    framesDQ.append(nsimplify(fkDQ.evalf(), tolerance = 1e-10) if symbolic else fkDQ)
    
  return framesDQ

def forwardCOMDQ(robot : object, symbolic = False):
  """Using Dual Quaternions, this function computes forward kinematics of a serial robot's centers of mass given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    framesHTM (list): Dual Quaternions with COMs' poses (numeric or symbolic)
  """
  
  # Calculate forward kinematics
  framesDQ = forwardDQ(robot, symbolic)

  # Initial conditions
  framesCOMDQ = [Matrix([1, 0, 0, 0, 0, 0, 0, 0])] if symbolic else [np.array([[1], [0], [0], [0], [0], [0], [0], [0]])]

  if symbolic:
      
    # Get Denavit - Hartenberg Parameters Matrix
    comDH = robot.symbolicDHParametersCOM
    
  else:
      
    # Update Denavit - Hartenberg Parameters Matrix
    robot.denavitHartenbergCOM()
    
    # Get Denavit - Hartenberg Matrix
    comDH = robot.dhParametersCOM
    
  # Iteration through all the rows in Denavit - Hartenberg Matrix for Centers of Mass
  for i in range(robot.symbolicCOMs.shape[0]):
        
    # Check where is the current Center of Mass
    rowCOM, column = robot.whereIsTheCOM(COM = i + 1)
    
    # Center of Mass Homogeneous Transformation Matrix
    COM = dqMultiplication(dqMultiplication(dqMultiplication(dqRz(comDH[rowCOM, 0] if column >= 0 else 0, symbolic), dqTz(comDH[rowCOM, 1] if column >= 1 else 0, symbolic), symbolic), dqTx(comDH[rowCOM, 2] if column >= 2 else 0, symbolic), symbolic), dqRx(comDH[rowCOM, 3] if column >= 3 else 0, symbolic), symbolic) if symbolic else dqMultiplication(dqMultiplication(dqMultiplication(dqRz(comDH[rowCOM, 0] if column >= 0 else 0), dqTz(comDH[rowCOM, 1] if column >= 1 else 0)), dqTx(comDH[rowCOM, 2] if column >= 2 else 0)), dqRx(comDH[rowCOM, 3] if column >= 3 else 0))

    # Forward kinematics to Center of Mass
    fkCOMDQ = trigsimp(dqMultiplication(framesDQ[rowCOM - 1], COM, symbolic)) if symbolic else dqMultiplication(framesDQ[rowCOM - 1], COM)

    # Append results
    framesCOMDQ.append(nsimplify(fkCOMDQ.evalf(), tolerance = 1e-10) if symbolic else fkCOMDQ)
      
  return framesCOMDQ

def jacobianDQ(robot : object, symbolic = False):
  """Using Dual Quaternions, this function computes Jacobian Matrix of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    J (NumPy Array): Inertial Jacobian Matrix for Quaternion's Rate of Change (numeric)
    J (SymPy Matrix): Inertial Jacobian Matrix for Quaternion's Rate of Change (symbolic)
  """

  # Get number of joints (generalized coordinates)
  n = robot.jointsPositions.shape[0]
  
  # Calculate forward kinematics
  fkDQ = forwardDQ(robot, symbolic)
  
  # Initializes jacobian matrix with zeros
  J = zeros(8, n) if symbolic else np.zeros((8, n))
  
  # Iterates through all generalized coordinates
  for j in range(n):
    
    # Check in what row of Denavit Hartenberg Parameters Matrix is the current joint (the sum is because of the way Python indexes arrays)
    row, column = robot.whereIsTheJoint(j + 1)
    
    # Axis of actuation of current joint
    xi = Matrix(robot.xi[:, j]) if symbolic else robot.xi[:, j].reshape((8, 1))
    
    # Calculate Jacobian Matrix
    J[:, j] = 0.5 * dqMultiplication(dqMultiplication(dqMultiplication(fkDQ[row - 1], xi, symbolic), conjugateDQ(fkDQ[row - 1]), symbolic), fkDQ[-1], symbolic) if symbolic else 0.5 * dqMultiplication(dqMultiplication(dqMultiplication(fkDQ[row - 1], xi), conjugateDQ(fkDQ[row - 1])), fkDQ[-1]).flatten()
  
  return nsimplify(trigsimp(J).evalf(), tolerance = 1e-10) if symbolic else J

def jacobianVelocityDQ(robot : object, symbolic = False):
  """Using Dual Quaternions, this function computes the Dual Inertial Velocity Jacobian Matrix for velocity calcuation of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    Jv (NumPy Array): Dual Inertial Velocity Jacobian Matrix (numeric)
    Jv (SymPy Matrix): Dual Inertial Velocity Jacobian Matrix (symbolic)
  """

  # Get number of joints (generalized coordinates)
  n = robot.jointsPositions.shape[0]
  
  # Calculate forward kinematics
  fkDQ = forwardDQ(robot, symbolic)
  
  # Initializes jacobian matrix with zeros
  J = zeros(8, n) if symbolic else np.zeros((8, n))
  
  # Iterates through all generalized coordinates
  for j in range(n):
    
    # Check in what row of Denavit Hartenberg Parameters Matrix is the current joint (the sum is because of the way Python indexes arrays)
    row, column = robot.whereIsTheJoint(j + 1)
    
    # Axis of actuation of current joint
    xi = Matrix(robot.xi[:, j]) if symbolic else robot.xi[:, j].reshape((8, 1))
    
    # Calculate Jacobian Matrix
    J[:, j] = dqMultiplication(dqMultiplication(fkDQ[row - 1], xi, symbolic), conjugateDQ(fkDQ[row - 1], symbolic), symbolic) if symbolic else dqMultiplication(dqMultiplication(fkDQ[row - 1], xi), conjugateDQ(fkDQ[row - 1])).flatten()
  
  # Create position vector from inertial frame to end - effector (from DQ space to R3)
  r = dqToR3(fkDQ[-1], symbolic)
  
  # Create position matrix
  M = Matrix([[eye(4), zeros(4)], [-crossOperatorExtension(r, symbolic), eye(4)]]) if symbolic else np.append(np.append(np.eye(4), np.zeros((4, 4)), axis = 1), np.append(-crossOperatorExtension(r), np.eye(4), axis = 1), axis = 0)
  
  return nsimplify(trigsimp(M * J).evalf(), tolerance = 1e-10) if symbolic else M.dot(J)

def jacobianVelocityDQCOM(robot : object, COM : int, symbolic = False):
  """Using Dual Quaternions, this function computes the Dual Inertial Velocity Jacobian Matrix for any center of mass of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    COM (int): center of mass that will be analyzed
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    Jv (NumPy Array): Inertial Velocity Jacobian Matrix (numeric)
    Jv (SymPy Matrix): Inertial Velocity Jacobian Matrix (symbolic)
  """

  # Get number of joints (generalized coordinates)
  n = robot.jointsPositions.shape[0]
  
  # Calculate forward kinematics
  fkDQ = forwardDQ(robot, symbolic)
  
  # Calculate forward kinematics to each Center of Mass
  fkCOMDQ = forwardCOMDQ(robot, symbolic)
    
  # Check in what row of Denavit Hartenberg Parameters Matrix is the Center of Mass (the sum is because of the way Python indexes arrays)
  rowCOM = robot.whereIsTheCOM(COM)[0]
  
  # Initializes jacobian matrix with zeros
  J = zeros(8, n) if symbolic else np.zeros((8, n))
  
  # Iterates through all generalized coordinates
  for j in range(n):
    
    # Check in what row of Denavit Hartenberg Parameters Matrix is the current joint (the sum is because of the way Python indexes arrays)
    row, column = robot.whereIsTheJoint(j + 1)
    
    # If the current joint is coupled after the desired center of mass, and any Center of Mass is analyzed
    if row > rowCOM:
      
      # Stop the algorithm, because current joint won't affect the desired center of mass
      break
    
    # Axis of actuation of current joint
    xi = Matrix(robot.xi[:, j]) if symbolic else robot.xi[:, j].reshape((8, 1))
    
    # Calculate Jacobian Matrix
    J[:, j] = dqMultiplication(dqMultiplication(fkDQ[row - 1], xi, symbolic), conjugateDQ(fkDQ[row - 1], symbolic), symbolic) if symbolic else dqMultiplication(dqMultiplication(fkDQ[row - 1], xi), conjugateDQ(fkDQ[row - 1])).flatten()
  
  # Create position vector from inertial frame to end - effector (from DQ space to R3)
  r = dqToR3(fkCOMDQ[COM], symbolic)
  
  # Create position matrix
  M = Matrix([[eye(4), zeros(4)], [-crossOperatorExtension(r, symbolic), eye(4)]]) if symbolic else np.append(np.append(np.eye(4), np.zeros((4, 4)), axis = 1), np.append(-crossOperatorExtension(r), np.eye(4), axis = 1), axis = 0)
  
  return nsimplify(trigsimp(M * J).evalf(), tolerance = 1e-10) if symbolic else M.dot(J)

def jacobianVelocityDerivativeDQ(robot : object, symbolic = False):
  """Using Dual Quaternions, this function computes the time derivative of Dual Inertial Velocity Jacobian Matrix for acceleration calcuation of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    dJv (NumPy Array): Time Derivative of Dual Inertial Velocity Jacobian Matrix (numeric)
    dJv (SymPy Matrix): Time Derivative of Dual Inertial Velocity Jacobian Matrix (symbolic)
  """
  
  # Get number of joints (generalized coordinates)
  n = robot.jointsPositions.shape[0]
  
  # Calculate forward kinematics
  fkDQ = forwardDQ(robot, symbolic)
  
  # Initializes derivative of jacobian matrix with zeros
  dJ = zeros(8, n) if symbolic else np.zeros((8, n))
  
  # Create position vector from inertial frame to end - effector (from DQ space to R3)
  r = dqToR3(fkDQ[-1], symbolic)
  
  # Create position matrix
  M = Matrix([[eye(4), zeros(4)], [-crossOperatorExtension(r, symbolic), eye(4)]]) if symbolic else np.append(np.append(np.eye(4), np.zeros((4, 4)), axis = 1), np.append(-crossOperatorExtension(r), np.eye(4), axis = 1), axis = 0)
  
  # Create position matrix for non linear terms: w_{i + 1} x v_{i + 1} - w_{i} x v_{i}
  M_k = Matrix([[zeros(4), zeros(4)], [-crossOperatorExtension(r, symbolic), eye(4)]]) if symbolic else np.append(np.append(np.zeros((4, 4)), np.zeros((4, 4)), axis = 1), np.append(-crossOperatorExtension(r), np.eye(4), axis = 1), axis = 0)
    
  # Iterates through all generalized coordinates
  for j in range(n):
    
    # Check in what row of Denavit Hartenberg Parameters Matrix is the current joint (the sum is because of the way Python indexes arrays)
    row, column = robot.whereIsTheJoint(j + 1)
    
    # Axis of actuation of current joint
    xi = Matrix(robot.xi[:, j]) if symbolic else robot.xi[:, j].reshape((8, 1))
    
    # Calculates the adjoint transformation of the current axis of actuation (xi)
    Ad = dqMultiplication(dqMultiplication(fkDQ[row - 1], xi, symbolic), conjugateDQ(fkDQ[row - 1], symbolic), symbolic)
    
    # Derivative of axis of actuation of current joint
    xid = Matrix(robot.xid[:, j]) if symbolic else robot.xid[:, j].reshape((8, 1))
    
    # Calculates the adjoint transformation of the current axis of actuation (xid)
    dAd = dqMultiplication(dqMultiplication(fkDQ[row - 1], xid, symbolic), conjugateDQ(fkDQ[row - 1], symbolic), symbolic)
                
    # Iterates through all the joints to calculate cross product: W_{i} x Ad(xi)
    for k in range(j):
      
      # Check in what row of Denavit Hartenberg Parameters Matrix is the current joint (the sum is because of the way Python indexes arrays)
      row_k, column_k = robot.whereIsTheJoint(k + 1)
      
      # Axis of actuation of current joint
      xi_k = Matrix(robot.xi[:, k]) if symbolic else robot.xi[:, k].reshape((8, 1))
      
      # Calculates the adjoint transformation of the current axis of actuation (xi_k)
      Ad_k = dqMultiplication(dqMultiplication(fkDQ[row_k - 1], xi_k, symbolic), conjugateDQ(fkDQ[row_k - 1], symbolic), symbolic)
      
      # Non linear terms calculation for th Time Derivative of Jacobian Matrix
      dJ[:, j] += dualCrossOperator(Ad_k * robot.qdSymbolic[k], symbolic) * Ad if symbolic else (dualCrossOperator(Ad_k * robot.jointsVelocities[k]).dot(Ad)).flatten()
    
    # Calculates the adjoint transformation of the current derivative of the axis of actuation (xid)
    dJ[:, j] = M * (dAd + dJ[:, j]) if symbolic else M.dot(dAd + dJ[:, j].reshape((8, 1))).flatten()
    
    # Iterates through all the joints to calculate non linear terms: w_{i + 1} x v_{i + 1} - w_{i} x v_{i}
    for k in range(n):
      
      # Check in what row of Denavit Hartenberg Parameters Matrix is the current joint (the sum is because of the way Python indexes arrays)
      row_k, column_k = robot.whereIsTheJoint(k + 1)
      
      # Axis of actuation of current joint
      xi_k = Matrix(robot.xi[:, k]) if symbolic else robot.xi[:, k].reshape((8, 1))
      
      # Calculates the adjoint transformation of the current axis of actuation (xi_k)
      Ad_k = dqMultiplication(dqMultiplication(fkDQ[row_k - 1], xi_k, symbolic), conjugateDQ(fkDQ[row_k - 1], symbolic), symbolic)
      
      # Non linear terms calculation for th Time Derivative of Jacobian Matrix
      dJ[:, j] += dualCrossOperator(Ad, symbolic) * (M_k * Ad_k * robot.qdSymbolic[k]) if symbolic else (dualCrossOperator(Ad).dot(M_k.dot(Ad_k) * robot.jointsVelocities[k])).flatten()
  
  return nsimplify(dJ.evalf(), tolerance = 1e-10) if symbolic else dJ

def jacobianVelocityDerivativeDQCOM(robot : object, COM : int, symbolic = False):
  """Using Dual Quaternions, this function computes the time derivative of Dual Inertial Velocity Jacobian Matrix for any center of mass of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    COM (int): center of mass that will be analyzed
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    dJvCOM (NumPy Array): Time Derivative of Dual Inertial Velocity Jacobian Matrix (numeric)
    dJvCOM (SymPy Matrix): Time Derivative of Dual Inertial Velocity Jacobian Matrix (symbolic)
  """
  
  # Get number of joints (generalized coordinates)
  n = robot.jointsPositions.shape[0]
  
  # Calculate forward kinematics
  fkDQ = forwardDQ(robot, symbolic)
  
  # Calculate forward kinematics to each Center of Mass
  fkCOMDQ = forwardCOMDQ(robot, symbolic)
  
  # Check in what row of Denavit Hartenberg Parameters Matrix is the Center of Mass (the sum is because of the way Python indexes arrays)
  rowCOM = robot.whereIsTheCOM(COM)[0]
  
  # Initializes derivative of jacobian matrix with zeros
  dJ = zeros(8, n) if symbolic else np.zeros((8, n))
  
  # Create position vector from inertial frame to end - effector (from DQ space to R3)
  r = dqToR3(fkCOMDQ[COM], symbolic)
  
  # Create position matrix
  M = Matrix([[eye(4), zeros(4)], [-crossOperatorExtension(r, symbolic), eye(4)]]) if symbolic else np.append(np.append(np.eye(4), np.zeros((4, 4)), axis = 1), np.append(-crossOperatorExtension(r), np.eye(4), axis = 1), axis = 0)
  
  # Create position matrix for non linear terms: w_{i + 1} x v_{i + 1} - w_{i} x v_{i}
  M_k = Matrix([[zeros(4), zeros(4)], [-crossOperatorExtension(r, symbolic), eye(4)]]) if symbolic else np.append(np.append(np.zeros((4, 4)), np.zeros((4, 4)), axis = 1), np.append(-crossOperatorExtension(r), np.eye(4), axis = 1), axis = 0)
    
  # Iterates through all generalized coordinates
  for j in range(n):
    
    # Check in what row of Denavit Hartenberg Parameters Matrix is the current joint (the sum is because of the way Python indexes arrays)
    row, column = robot.whereIsTheJoint(j + 1)
    
    # If the current joint is coupled after the desired center of mass, and any Center of Mass is analyzed
    if row > rowCOM:
      
      # Stop the algorithm, because current joint won't affect the desired center of mass
      break
    
    # Axis of actuation of current joint
    xi = Matrix(robot.xi[:, j]) if symbolic else robot.xi[:, j].reshape((8, 1))
    
    # Calculates the adjoint transformation of the current axis of actuation (xi)
    Ad = dqMultiplication(dqMultiplication(fkDQ[row - 1], xi, symbolic), conjugateDQ(fkDQ[row - 1], symbolic), symbolic)
    
    # Derivative of axis of actuation of current joint
    xid = Matrix(robot.xid[:, j]) if symbolic else robot.xid[:, j].reshape((8, 1))
    
    # Calculates the adjoint transformation of the current axis of actuation (xid)
    dAd = dqMultiplication(dqMultiplication(fkDQ[row - 1], xid, symbolic), conjugateDQ(fkDQ[row - 1], symbolic), symbolic)
                
    # Iterates through all the joints to calculate cross product: W_{i} x Ad(xi)
    for k in range(j):
      
      # Check in what row of Denavit Hartenberg Parameters Matrix is the current joint (the sum is because of the way Python indexes arrays)
      row_k, column_k = robot.whereIsTheJoint(k + 1)
      
      # If the current joint is coupled after the desired center of mass, and any Center of Mass is analyzed
      if row_k > rowCOM:
        
        # Stop the algorithm, because current joint won't affect the desired center of mass
        break
      
      # Axis of actuation of current joint
      xi_k = Matrix(robot.xi[:, k]) if symbolic else robot.xi[:, k].reshape((8, 1))
      
      # Calculates the adjoint transformation of the current axis of actuation (xi_k)
      Ad_k = dqMultiplication(dqMultiplication(fkDQ[row_k - 1], xi_k, symbolic), conjugateDQ(fkDQ[row_k - 1], symbolic), symbolic)
      
      # Non linear terms calculation for th Time Derivative of Jacobian Matrix
      dJ[:, j] += dualCrossOperator(Ad_k * robot.qdSymbolic[k], symbolic) * Ad if symbolic else (dualCrossOperator(Ad_k * robot.jointsVelocities[k]).dot(Ad)).flatten()
    
    # Calculates the adjoint transformation of the current derivative of the axis of actuation (xid)
    dJ[:, j] = M * (dAd + dJ[:, j]) if symbolic else M.dot(dAd + dJ[:, j].reshape((8, 1))).flatten()
    
    # Iterates through all the joints to calculate non linear terms: w_{i + 1} x v_{i + 1} - w_{i} x v_{i}
    for k in range(n):
      
      # Check in what row of Denavit Hartenberg Parameters Matrix is the current joint (the sum is because of the way Python indexes arrays)
      row_k, column_k = robot.whereIsTheJoint(k + 1)
      
      # If the current joint is coupled after the desired center of mass, and any Center of Mass is analyzed
      if row_k > rowCOM:
        
        # Stop the algorithm, because current joint won't affect the desired center of mass
        break
      
      # Axis of actuation of current joint
      xi_k = Matrix(robot.xi[:, k]) if symbolic else robot.xi[:, k].reshape((8, 1))
      
      # Calculates the adjoint transformation of the current axis of actuation (xi_k)
      Ad_k = dqMultiplication(dqMultiplication(fkDQ[row_k - 1], xi_k, symbolic), conjugateDQ(fkDQ[row_k - 1], symbolic), symbolic)
      
      # Non linear terms calculation for th Time Derivative of Jacobian Matrix
      dJ[:, j] += dualCrossOperator(Ad, symbolic) * (M_k * Ad_k * robot.qdSymbolic[k]) if symbolic else (dualCrossOperator(Ad).dot(M_k.dot(Ad_k) * robot.jointsVelocities[k])).flatten()
  
  return nsimplify(dJ.evalf(), tolerance = 1e-10) if symbolic else dJ

def inverseDQ(robot : object, q0 : np.array, Qd : np.array, K : np.array):
  """Using Dual Quaternions, this function computes Inverse Kinematics of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
    q0 (np.array): initial conditions of joints
    Qd (np.array): desired pose represented with a Dual Quaternion
    K (np.array): gain matrix to guarantee stability

  Returns:
    q (np.array): joints positions to reach desired pose
  """
  
  # Set initial conditions
  q = q0.reshape(q0.shape)
  
  # Auxiliar variable to keep original joints positions
  z = robot.jointsPositions.copy()
  
  # Iterator
  j = 0
  
  while j < 15000:
    
    # Set calculated positions on robot
    robot.jointsPositions = q[:, -1].reshape(q0.shape)
    
    # Calculate forward kinematics
    Q = forwardDQ(robot)
    
    # Calculate error
    e = Qd - Q[-1]
    
    # If error is less equal to 0.001
    if np.abs(np.max(e)) <= 0.001:
      
      # Solution found!
      print("\nSolution found using Dual Quaternions!")
      
      # Stop loop
      break
    
    # Calculate inertial jacobian matrix
    J = jacobianDQ(robot)
    
    # Calculate new joints positions
    solution = rungeKutta4(f = (np.linalg.pinv(J)).dot(K).dot(e), F = q[:, -1].reshape(q0.shape))
    
    # Append result
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