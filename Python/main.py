import Dynamics as dy
import Kinematics as k
import Movements as mv
import multiprocessing as mp
import numpy as np
from Plot import *
import random
import Robot
from sympy import *

if __name__ == '__main__':
  """
    1. Creates robot's generalized coordinates (as two dimensional array) and links' lengths
  """
  
  # Generalized coordinates
  q = np.random.rand(4, 1)
  
  # Links
  L = [0.3, 0.5, 0.4]
  
  # Centers of Mass
  Lcom = [0.15, 0.25, 0.2]

  """
    2. Creates robot as an object
  """
  uRobot = Robot.System(jointsPositions = q, linksLengths = L, COMs = Lcom, name = 'uRobot')

  """
    3. Gets robots' Denavit - Hartenberg parameters (numeric and symbolic)
  """
  uRobot.denavitHartenberg()
  uRobot.symbolicDenavitHartenberg()
  
  """
    3.1 Gets robots' Denavit - Hartenberg COMs' parameters (numeric and symbolic)
  """
  uRobot.denavitHartenbergCOM()
  uRobot.symbolicDenavitHartenbergCOM()

  """
    4. Calculate robot's forward kinematics (using Homogeneous Transformation Matrices or Dual Quaternions)
  """
  
  # Homogeneous Transformation Matrices
  fkHTM = k.forwardHTM(uRobot)
  # symbolicFKHTM = k.forwardHTM(uRobot, symbolic = True)
  
  # Dual Quaternions
  fkDQ = k.forwardDQ(uRobot)
  # symbolicFKDQ = k.forwardDQ(uRobot, symbolic = True)
  
  """
    4.1 Calculate robot's forward kinematics to m - th center of mass (using Homogeneous Transformation Matrices or Dual Quaternions)
  """
  # Homogeneous Transformation Matrices
  fkCOMHTM = k.forwardCOMHTM(uRobot)
  # symbolicFKCOMHTM = k.forwardCOMHTM(uRobot, symbolic = True)
  
  # Dual Quaternions
  fkCOMDQ = k.forwardCOMDQ(uRobot)
  # symbolicDQCOMHTM = k.forwardCOMDQ(uRobot, symbolic = True)

  """
    5. Calculate Axis - Angle vector using Homogeneous Transformation Matrices (if necessary; this is OPTIONAL)
  
  """
  
  X = mv.axisAngle(fkHTM[-1])
  # symbolicX = mv.axisAngle(symbolicFKHTM[-1], symbolic = True)
  
  """
    6. Calculate Inverse Kinematics
  """ 
  
  """
    6.1 Calculate robot's Jacobian Matrix (using Homogeneous Transformation Matrices or Dual Quaternions)
  """
  # Screw vectors stored in a matrix (mandatory)
  xi = np.array([[0, 0, 0, 0],
                 [0, 0, 0, 0],
                 [0, 0, 0, 0],
                 [1, 1, 1, 1],
                 [0, 0, 0, 0],
                 [0, 0, 0, 0],
                 [0, 0, 0, 0],
                 [0, 0, 0, 0]])

  # Time derivative of screw vectors stored in a matrix (mandatory)
  xid = np.zeros((8, 4))

  # Geometric Jacobian Matrix (OPTIONAL)
  Jg = k.geometricJacobian(uRobot)
  # symbolicJg = k.geometricJacobian(uRobot, symbolic = True)
  
  # Analytic Jacobian Matrix (OPTIONAL)
  Ja = k.analyticJacobian(uRobot)
  # symbolicJa = k.analyticJacobian(uRobot, symbolic = True)
  
  # Dual Jacobian Matrix (OPTIONAL)
  Jdq = k.jacobianDQ(uRobot, xi)
  # Jdq = k.jacobianDQ(uRobot, xi, symbolic = True)

  # Time derivative of Dual Jacobian Matrix
  # W = np.array(np.zeros((1, 4)).tolist() + np.random.randn(3, 4).tolist() + np.zeros((1, 4)).tolist() + np.random.randn(3, 4).tolist())
  # JdDQ = k.dotJacobianDQ(uRobot, m = 5, W = W, xi = xi, xid = xid)
  
  """ 
    6.2 Calculate robot's Inverse Kinematics to a single point (using Homogeneous Transformation Matrices or Dual Quaternions)
  """
  qHTM = k.inverseHTM(uRobot, q0 = np.random.rand(4, 1), Hd = fkHTM[-1], K = 50 * np.eye(6))
  qDQ = k.inverseDQ(uRobot, q0 = np.random.rand(4, 1), Qd = fkDQ[-1], K = 50 * np.eye(8), xi = xi)
  
  """
    7. Differential Kinematics
  """

  """
    7.1 Calculate robot's Inertial Velocity Jacobian Matrix (using Dual Quaternions, Homogeneous Transformation Matrix does not need a different Jacobian)
  """
  
  Jvdq = k.jacobianVDQ(uRobot, xi)
  # symbolicJvdq = k.jacobianVDQ(uRobot, xi, symbolic = True)

  """
    7.2 Calculate Instantaneous Inertial Velocity to m - th frame
  """
  
  # Joints velocities
  qd = np.random.rand(4, 1)
  
  # End-effector inertial velocity (using geometric jacobian matrix) with Homogeneous Transformation Matrices
  geometricXd = k.geometricStateSpace(uRobot, qd)
  # symbolicXd = k.geometricalStateSpace(uRobot, qd, symbolic = True)
  
  # End-effector inertial linear velocity + inertial rate of change of its rotations (using analytic jacobian matrix) with Homogeneous Transformation Matrices
  analyticXd = k.analyticStateSpace(uRobot, qd)
  # symbolicXd = k.analyticalStateSpace(uRobot, qd, symbolic = True)
  
  # End-effector inertial velocity with Dual Quaternions
  Vdq = k.velocityDQ(uRobot, qd, xi)
  # symbolicVdq = k.velocityDQ(uRobot, qd = uRobot.qdSymbolic, xi =xi, symbolic = True)
  
  """
    # 6.3 Trajectory planning in Joints' Space, by means of randomizing joints positions. These will be computed to create the trajectory to be followed
  """ 
  
  """
  # 1. Set number of points
  points = 7
    
  # 2. Set time intervals (randomly)
  time = np.append(np.array([0]), np.random.uniform(0.5, 2, points))
  m, n = q.shape
    
  # 3. Set joints' initial conditions (randomly)
  jointsHTM = np.random.rand(m, n)
  jointsDQ = np.random.rand(m, n)
    
  # 4. Iterate over each point
  for position in range(points):
    # 4.1 Set random positions for each joint
    uRobot.jointsPositions = np.random.rand(m, n)
    
    # 4.2 Calculate forward kinematics to demonstrate that robot can reach those random joints' positions
    framesHTM, fkHTM = k.forwardHTM(uRobot, m = 5)
    framesDQ, fkDQ = k.forwardDQ(uRobot, m = 5)
    
    # 4.3 Use previous results as desired pose representation to store the joints' positions that have to be reached
    jointsHTM = np.append(jointsHTM, k.inverseHTM(uRobot, q0 = jointsHTM[:, -1].reshape((m, 1)), Hd = fkHTM, K = np.eye(6), m = 5)[:, -1].reshape((m, 1)), axis = 1)
    jointsDQ = np.append(jointsDQ, k.inverseDQ(uRobot, q0 = jointsDQ[:, -1].reshape((m, 1)), Qd = fkDQ, K = np.eye(8), xi = xi, m = 5)[:, -1].reshape((m, 1)), axis = 1)
    
  # 5. Calculate trajectories for each joints' computation
  qHTM, qdHTM, qddHTM, xHTM = dy.trajectory(P = jointsHTM, steps = time, absolut = False)
  qDQ, qdDQ, qddDQ, xDQ = dy.trajectory(P = jointsDQ, steps = time, absolut = False)
  
  """
    # 6.3 Calculate trajectory in Task Space based on trajectory in Joints' Space (this is also in section 7)
  """
  # plot.trajectory3D(robot = uRobot, q = qHTM, m = 5)
  
  """
    # 7. Animate robot with joints' positions without multiprocessing (this also modifies them in the object). You can uncomment any of these:
  """
  # animation(uRobot, q = qHTM, plotBodies = True, plotFrames = False, plotCOMs = True, delayPerFrame = 1, repeatAnimation = False)
  # animation(uRobot, q = qDQ, plotBodies = True, plotFrames = True, plotCOMs = False, delayPerFrame = 1, repeatAnimation = False)
  
  """
    # 7.1 Plot and Animate robot's behavior (joint's positions, end - effector, dynamics, etc) using multiprocessing
  """
  
  processes = [mp.Process(target = animation, args = (uRobot, qHTM)),
               mp.Process(target = animation, args = (uRobot, qDQ)),
               mp.Process(target = trajectory, args = (qHTM, jointsHTM, time, r"Trajectory Planning for Joints' Positions (HTM)", r'$\theta_')),
               mp.Process(target = trajectory, args = (qDQ, jointsDQ, time, r"Trajectory Planning for Joints' Positions (DQ)", r'$\theta_')),
               mp.Process(target = trajectory3D, args = (uRobot, qHTM, 5)),
               mp.Process(target = trajectory3D, args = (uRobot, qDQ, 5))]
  
  for process in processes:
    process.start()
    
  for process in processes:
    process.join()
  
  for process in processes:
    process.close()

"""

  

  """
    8.3 Calculate Instantaneous Joints' Velocities given m - th frame one
  """
  qdHTM = k.jointsVelocitiesHTM(uRobot, m = 5, Vhtm = Vhtm)
  qdDQ = k.jointsVelocitiesDQ(uRobot, m = 5, n = 4, Vdq = Vdq, xi = xi)

  """
    8.4 Calculate Instantaneous Relative Inertial Velocity to n - th frame attached to each joint
  """
  Wi = k.relativeVelocityDQ(uRobot, m = 5, n = 4, W0 = np.zeros((8, 1)), qd = np.random.rand(4, 1), xi = xi)

  """
    8.5 Calculate Instantaneous Inertial Velocity to p - th Center of Masss
  """
  Wcom = k.velocityPropagationDQ(uRobot, m = 5, COMs = 4, W0 = np.zeros((8, 1)), qd = np.random.rand(4, 1), xi = xi)

  """
    8.6 Calculate robot's Inertial Acceleration Jacobian Matrix (using Dual Quaternions)
  """
  Kadq = k.jacobianADQ(uRobot, m = 5, n = 4, W0 = np.zeros((8, 1)), qd = np.random.rand(4, 1), xi = xi, xid = xid)

  """
    8.7 Calculate Instantaneous Inertial Acceleration to m - th frame
  """
  Adq = k.accelerationDQ(uRobot, m = 5, n = 4, W0 = np.zeros((8, 1)), qd = np.random.rand(4, 1), qdd = np.random.rand(4, 1), xi = xi, xid = xid)
  
  """
    8.8 Calculate Instantaneous Inertial Acceleration to p - th Center of Masss
  """
  Acom = k.accelerationPropagationDQ(uRobot, m = 5, COMs = 4, W0 = np.zeros((8, 1)), A0 = np.append(np.zeros((7, 1)), np.array([[-9.80665]]), axis = 0), qd = np.random.rand(4, 1), qdd = np.random.rand(4, 1), xi = xi, xid = xid)

  """
    8.9 Calculate Instantaneous Joints' Accelerations given m - th frame one
  """
  qddDQ = k.jointsAccelerationsDQ(uRobot, m = 5, n = 4, W0 = np.zeros((8, 1)), qd = qdDQ, Adq = Adq, xi = xi, xid = xid)

  print("Z")
  # END