import DenavitHartenberg as dh
import Dynamics as dy
import Kinematics as k
import Movements as mv
import multiprocessing as mp
import numpy as np
import Plot as plot
import random
import Robot

if __name__ == '__main__':
  """
    1. Creates robot's generalized coordinates (as two dimensional array) and links' lengths
  """
  q = np.random.rand(4, 1)
  L = [0.3, 0.5, 0.4]
  Lcom = [0.15, 0.25, 0.2]

  """
    2. Creates robot as an object
  """
  uRobot = Robot.System(jointsPositions = q, linksLengths = L, centersOfMass = Lcom, name = 'uRobot')

  """
    3. Gets robots' Denavit - Hartenberg parameters
  """
  DH = dh.matrix(uRobot)

  """
    4. Computes robot's forward kinematics (using Homogeneous Transformation Matrices or Dual Quaternions)
  """
  framesHTM, fkHTM = k.forwardHTM(uRobot, m = 5)
  framesDQ, fkDQ = k.forwardDQ(uRobot, m = 5)
  
  """
    4.1 Computes robot's forward kinematics to m - th center of mass (using Homogeneous Transformation Matrices or Dual Quaternions)
  """
  framesCOMHTM, fkCOMHTM = k.forwardCOMHTM(uRobot, m = 5)
  framesCOMDQ, fkCOMDQ = k.forwardCOMDQ(uRobot, m = 5)

  """
    5. Compute Axis - Angle vector using Homogeneous Transformation Matrices (if necessary; this is OPTIONAL)
  """ 
  X = mv.axisAngle(fkHTM)

  """
    6. Compute Inverse Kinematics
  """ 

  """
    6.1 Set desired pose representation (mandatory). We will set random joints positions to set random «fkHTM» and «fkDQ» as desired pose representations
  """ 
  points = 4
  time = np.array([0, 1, 3.5, 2.2])
  randomPoseHTM = []
  randomPoseDQ = []
  for i in range(points):
    uRobot.jointsPositions = np.random.rand(4, 1)
    randomframesHTM, randomfkHTM = k.forwardHTM(uRobot, m = 5)
    randomframesDQ, randomfkDQ = k.forwardDQ(uRobot, m = 5)
    randomPoseHTM.append(randomfkHTM)
    randomPoseDQ.append(randomfkDQ)
  randomPoseDQ = np.array(randomPoseDQ).reshape((8, points))

  """
    6.2 Computes the path to be followed by means of defining the points to be reached in specific intervals of time
  """
  
  Xhtm = dy.path(P = randomPoseHTM, steps = time, plot = True)
  Xdq = dy.path(P = randomPoseDQ, steps = time, plot = True)

  """
    6.3 Computes robot's Jacobian Matrix (using Homogeneous Transformation Matrices or Dual Quaternions)
  """

  # Screw vectors stored in a matrix
  xi = np.array([[0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [1, 1, 1, 1],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0]])

  # Time derivative of screw vectors stored in a matrix
  xid = np.zeros((8, 4))

  Jhtm = k.jacobianHTM(uRobot, m = 5)

  Jdq = k.jacobianDQ(uRobot, m = 5, xi = xi)

  """ 
    6.4 Computes robot's Inverse Kinematics (using Homogeneous Transformation Matrices or Dual Quaternions)
  """
  qHTM = k.inverseHTM(uRobot, q0 = np.random.rand(4, 1), Hd = fkHTM, K = np.eye(6), m = 5)
  qDQ = k.inverseDQ(uRobot, q0 = np.random.rand(4, 1), Qd = fkDQ, K = np.eye(8), xi = xi, m = 5)
  
  """
    7. Animate robot with joints' positions without multiprocessing (this also modifies them in the object). You can uncomment any of these:
  """
  # plot.animation(uRobot, q = qHTM, plotBodies = True, plotFrames = False, plotCOMs = True, delayPerFrame = 1, repeatAnimation = False)
  # plot.animation(uRobot, q = qDQ, plotBodies = True, plotFrames = True, plotCOMs = False, delayPerFrame = 1, repeatAnimation = False)
  
  """
    7.1 Plot and Animate robot's behavior (joint's positions, end - effector, dynamics, etc) using multiprocessing
  """
  
  processes = [mp.Process(target = plot.animation, args = (uRobot, qHTM)),
               mp.Process(target = plot.animation, args = (uRobot, qDQ)),
               mp.Process(target = plot.graph, args = (qHTM, r'Joints Positions (using HTM)', r'$\theta_', r'(k)$', r'Samples $k$ [3 $\frac{ms}{sample}$]', r'Amplitude [$rad$]', True, "qHTM", True)),
               mp.Process(target = plot.graph, args = (qDQ, r'Joints Positions (using DQ)', r'$\theta_', r'(k)$', r'Samples $k$ [3 $\frac{ms}{sample}$]', r'Amplitude [$rad$]', True, "qDQ", False))]
  
  for process in processes:
    process.start()
    
  for process in processes:
    process.join()
  
  for process in processes:
    process.close()

  """
    8. Differential Kinematics
  """

  """
    8.1 Computes robot's Inertial Velocity Jacobian Matrix (using Dual Quaternions, Homogeneous Transformation Matrix does not need a different Jacobian)
  """
  Jvdq = k.jacobianVDQ(uRobot, m = 5, n = 4, xi = xi)

  """
    8.2 Computes Instantaneous Inertial Velocity to m - th frame
  """
  Vhtm = k.velocityHTM(uRobot, m = 5, qd = np.random.rand(4, 1))
  Vdq = k.velocityDQ(uRobot, m = 5, n = 4, qd = np.random.rand(4, 1), xi = xi)

  """
    8.3 Computes Instantaneous Joints' Velocities given m - th frame one
  """
  qdHTM = k.jointsVelocitiesHTM(uRobot, m = 5, Vhtm = Vhtm)
  qdDQ = k.jointsVelocitiesDQ(uRobot, m = 5, n = 4, Vdq = Vdq, xi = xi)

  """
    8.4 Computes Instantaneous Relative Inertial Velocity to n - th frame attached to each joint
  """
  Wi = k.relativeVelocityDQ(uRobot, m = 5, n = 4, W0 = np.zeros((8, 1)), qd = np.random.rand(4, 1), xi = xi)

  """
    8.5 Computes Instantaneous Inertial Velocity to p - th Center of Masss
  """
  Wcom = k.velocityPropagationDQ(uRobot, m = 5, COMs = 4, W0 = np.zeros((8, 1)), qd = np.random.rand(4, 1), xi = xi)

  """
    8.6 Computes robot's Inertial Acceleration Jacobian Matrix (using Dual Quaternions)
  """
  Kadq = k.jacobianADQ(uRobot, m = 5, n = 4, W0 = np.zeros((8, 1)), qd = np.random.rand(4, 1), xi = xi, xid = xid)

  """
    8.7 Computes Instantaneous Inertial Acceleration to m - th frame
  """
  Adq = k.accelerationDQ(uRobot, m = 5, n = 4, W0 = np.zeros((8, 1)), qd = np.random.rand(4, 1), qdd = np.random.rand(4, 1), xi = xi, xid = xid)
  
  """
    8.8 Computes Instantaneous Inertial Acceleration to p - th Center of Masss
  """
  Acom = k.accelerationPropagationDQ(uRobot, m = 5, COMs = 4, W0 = np.zeros((8, 1)), A0 = np.append(np.zeros((7, 1)), np.array([[-9.80665]]), axis = 0), qd = np.random.rand(4, 1), qdd = np.random.rand(4, 1), xi = xi, xid = xid)

  """
    8.9 Computes Instantaneous Joints' Accelerations given m - th frame one
  """
  qddDQ = k.jointsAccelerationsDQ(uRobot, m = 5, n = 4, W0 = np.zeros((8, 1)), qd = qdDQ, Adq = Adq, xi = xi, xid = xid)

  print("Z")
  # END