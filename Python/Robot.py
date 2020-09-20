import numpy as np
import matplotlib.pyplot as plt
import Movements as mv
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

class System:
  def __init__(self, jointsPositions, linksLengths, name = ''):
    """
      Robot's constructor
    """
    # Robot's name
    self.name = name

    # Kinematic Parameters
    self.jointsPositions = jointsPositions
    self.linksLengths = linksLengths

    # Initial conditions
    self.dhParameters = [[0, 0, 0, 0]]

  def denavitHartenberg(self, theta = 0, d = 0, a = 0, alpha = 0):
    """
      Denavit - Hartenberg parameters for n - th rigid body
    """

    # Denavit - Hartenberg matrix
    self.dhParameters.append([theta, d, a, alpha])
    return [theta, d, a, alpha]
  
  def forwardKinematics(self):
    """
      Computes forward kinematics to n - th rigid body given joints positions in radians
    """

    # Initial conditions
    self._frames = []

    # Modifies joint position in Denavit - Hartenberg Matrix
    for joint in range(len(self.jointsPositions)):
      self.dhParameters[joint + 1][0] = self.jointsPositions[joint]
    

    # Computes forward kinematics, from inertial frame to n - th one
    self.fkHTM = np.identity(4)
    for link in self.dhParameters:
      self.fkHTM = self.fkHTM.dot(mv.rz(link[0]).dot(mv.tz(link[1])).dot(mv.tx(link[2])).dot(mv.rx(link[3])))
      self._frames.append(self.fkHTM)
        
  def plot(self, q = np.array([[]]), delayPerFrame = 200, repeatAnimation = False, save = False):
    """
      Plot robot's behavior and reference frames attached to each joint
    """
    
    # If «q» is not empty
    if q.size != 0:
      self.jointsPositions = q

    # Set figure's parameters
    fig = plt.figure()
    ax = fig.gca(projection = '3d')
    ax.set_xlabel('$x$', color = 'red', fontsize = 16)
    ax.set_ylabel('$y$', color = 'green', fontsize = 16)
    ax.set_zlabel('$z$', color = 'blue', fontsize = 16)

    # Plot robot and reference frames
    def robot(q):
      plt.cla()
      self.jointsPositions = q
      self.forwardKinematics()
      
      # Inertial frame
      ax.plot(xs = [0, 1], ys = [0, 0], zs = [0, 0], color = 'red', linestyle = 'dashed', marker = 'o')
      ax.plot(xs = [0, 0], ys = [0, 1], zs = [0, 0], color = 'green', linestyle = 'dashed', marker = 'o')
      ax.plot(xs = [0, 0], ys = [0, 0], zs = [0, 1], color = 'blue', linestyle = 'dashed', marker = 'o')

      # Variable to increment or decrement robot dimensions and to scale its plot
      for frame in range(len(self._frames) - 1):
        if any(link < 1 for link in self.linksLengths):
          a = 10
        elif any(link > 100 for link in self.linksLengths):
          a = 0.01
        else:
          a = 1

        # Rigid body
        ba = a * self._frames[frame][: -1, 3]
        bb = a * self._frames[frame + 1][: -1, 3]
        
        # x - axis
        xa = bb
        xb = xa + self._frames[frame + 1][: -1, 0]

        # y - axis
        ya = bb
        yb = ya + self._frames[frame + 1][: -1, 1]

        # z - axis
        za = bb
        zb = za + self._frames[frame + 1][: -1, 2]

        # Figure limits
        limit = a * np.sqrt(sum([link ** 2 for link in self.linksLengths]))

        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_zlim(-limit, limit)
        
        # x - frame
        ax.plot(xs = [xa[0], xb[0]], ys = [xa[1], xb[1]], zs = [xa[2], xb[2]], color = 'red', linestyle = 'dashed', marker = 'o')

        # y - frame
        ax.plot(xs = [ya[0], yb[0]], ys = [ya[1], yb[1]], zs = [ya[2], yb[2]], color = 'green', linestyle = 'dashed', marker = 'o')

        # z - frame
        ax.plot(xs = [za[0], zb[0]], ys = [za[1], zb[1]], zs = [za[2], zb[2]], color = 'blue', linestyle = 'dashed', marker = 'o')

        # Rigid Body
        ax.plot(xs = [ba[0], bb[0]], ys = [ba[1], bb[1]], zs = [ba[2], bb[2]], color = 'magenta', linewidth = 4.5, marker = 'o')

    # Plot or animate robot
    ani = FuncAnimation(fig, robot, frames = self.jointsPositions, interval = delayPerFrame, repeat = repeatAnimation)
    plt.show()
