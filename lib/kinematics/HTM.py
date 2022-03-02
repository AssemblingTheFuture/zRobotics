# Access to parent folder to get its files
import sys, os
sys.path.append(sys.path[0].replace(r'/lib/kinematics', r''))

# Libraries
import numpy as np
from lib.movements.HTM import *
from lib.dynamics.Solver import *
from sympy import *

def forwardHTM(robot, symbolic = False):
    """Using Homogeneous Transformation Matrices, this function computes forward kinematics of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

    Args:
        robot (object): serial robot (this won't work with other type of robots)
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
        
        # Operates matrices symbolically: Rz * Tz * Tx * Rx
        fkHTM = fkHTM * rz(DH[frame, 0], symbolic = True) * tz(DH[frame, 1], symbolic = True) * tx(DH[frame, 2], symbolic = True) * rx(DH[frame, 3], symbolic = True) if symbolic else fkHTM.dot(rz(frame[0]).dot(tz(frame[1])).dot(tx(frame[2])).dot(rx(frame[3])))

        # Append each calculated Homogeneous Transformation Matrix
        framesHTM.append(fkHTM)

    return framesHTM

def forwardCOMHTM(robot, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes forward kinematics of a serial robot's centers of mass given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
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
  
  # Iterator
  i = 1
    
  # Iteration through all the rows in Denavit - Hartenberg Matrix
  for frame in range(comDH[1 : , :].rows) if symbolic else comDH[1 : , :]:
          
    # Center of Mass Homogeneous Transformation Matrix 
    COM = rz(comDH[frame + 1, 0], symbolic = True) * tz(comDH[frame + 1, 1], symbolic = True) * tx(comDH[frame + 1, 2], symbolic = True) * rx(comDH[frame + 1, 3], symbolic = True) if symbolic else rz(frame[0]).dot(tz(frame[1])).dot(tx(frame[2])).dot(rx(frame[3]))
    
    # Forward kinematics to Center of Mass
    fkCOMHTM = framesHTM[i - 1] * COM if symbolic else framesHTM[i - 1].dot(COM)
        
    framesCOMHTM.append(fkCOMHTM)
    
    i += 1
    
  return framesCOMHTM

def axisAngle(H, symbolic = False):
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
  X = H[0 : 3, 3].row_insert(3, simplify(theta * n)) if symbolic else np.append(H[0 : 3, 3], theta * n)
  
  return X if symbolic else X.reshape((6, 1))

def geometricJacobian(robot, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes Geometric Jacobian Matrix of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
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
    row = robot.whereIsTheJoint(j + 1)
    
    # Get axis of actuation of current joint ("row - 1" represents the reference frame where ONLY the joint is attached)
    z = fkHTM[row - 1][0: 3, 2]
      
    # Calculate distance between end - effector and current joint
    r = fkHTM[-1][0: 3, 3] - fkHTM[row - 1][0: 3, 3]
    
    # Calculate axes of actuation of Center of Mass or End - Effector caused by current joint
    J[0: 3, j] = z.cross(r) if symbolic else np.cross(z, r)
    
    # Set axis of actuation
    J[3: 6, j] = z
    
  return J

def geometricJacobianCOM(robot, COM, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes Geometric Jacobian Matrix to a desired Center of Mass of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
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
  rowCOM = robot.whereIsTheCOM(COM)
  
  # Initializes jacobian matrix with zeros
  J = zeros(6, n) if symbolic else np.zeros((6, n))
  
  # Iterates through all colums (generalized coordinates)
  for j in range(n):
    
    # Check in what row of Denavit Hartenberg Parameters Matrix is the current joint (the sum is because of the way Python indexes arrays)
    row = robot.whereIsTheJoint(j + 1)
    
    # If the current joint is coupled after the desired center of mass, and any Center of Mass is analyzed
    if row > rowCOM:
      
      # Stop the algorithm, because this joint won't affect the desired center of mass
      break
    
    # Get axis of actuation of current joint ("row - 1" represents the reference frame where ONLY the joint is attached)
    z = fkHTM[row - 1][0: 3, 2]
      
    # Calculate distance between Desired Center of Mass and current joint
    r = fkCOMHTM[rowCOM][0: 3, 3] - fkHTM[row - 1][0: 3, 3]
    
    # Calculate axes of actuation of Center of Mass or End - Effector caused by current joint
    J[0: 3, j] = z.cross(r) if symbolic else np.cross(z, r)
    
    # Set axis of actuation
    J[3: 6, j] = z
    
  return J

def analyticJacobian(robot, dq = 0.001, symbolic = False):
  """Using Homogeneous Transformation Matrices, this function computes Analytic Jacobian Matrix of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
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
    return x.jacobian(robot.qSymbolic)
    
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

def inverseHTM(robot, q0, Hd, K, jacobian = 'geometric'):
  """Using Homogeneous Transformation Matrices, this function computes Inverse Kinematics of a serial robot given joints positions in radians. Serial robot's kinematic parameters have to be set before using this function

  Args:
    robot (object): serial robot (this won't work with other type of robots)
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