import DenavitHartenberg as dh
import numpy as np
import matplotlib.pyplot as plt
import Movements as mv
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

class System:
  def __init__(self, jointsPositions, linksLengths, name = ''):
    """
      Robot's constructor
      jointsPositions: np.array (two - dimensional)
      linksLengths: np.array or list (one - dimensional)
      name: string (optional)
    """
    # Robot's name
    self.name = name

    # Kinematic Parameters
    self.jointsPositions = jointsPositions
    self.linksLengths = linksLengths

  def _checkValues(self, q, L):
    """
      Checks if generalized coordinates vector and lengths list are not empty
    """
    # If «q» is not empty
    if q.size != 0:
      self.jointsPositions = q
    
    # If «L» is not empty
    if L:
      self.linksLengths = L

  def denavitHartenberg(self, q = np.array([[]]), L = []):
    """
      Denavit - Hartenberg parameters for n - th rigid body
      theta: rotation on «z» axis
      d: translation on «z» axis
      a: translation on «x» axis
      alpha: rotation on «x» axis
    """

    # Checks if arrays are not empty
    self._checkValues(q, L)

    # Denavit - Hartenberg matrix
    self.dhParameters = dh.matrix(self.jointsPositions, self.linksLengths)
  
  def forwardKinematics(self, q, L, m = 0):
    """
      Computes forward kinematics to m - th rigid body given joints positions in radians. Robot's kinematic parameters have to be set before using this function
    """
    # Initial conditions
    self._frames = []
    
    # Modifies joint position in Denavit - Hartenberg Matrix
    self.denavitHartenberg(q, L)
    
    # Computes forward kinematics, from inertial frame to n - th one
    self.fkHTM = np.identity(4)
    i = 0
    for frames in self.dhParameters:
      self.fkHTM = self.fkHTM.dot(mv.rz(frames[0]).dot(mv.tz(frames[1])).dot(mv.tx(frames[2])).dot(mv.rx(frames[3])))
      self._frames.append(self.fkHTM)
      i += 1
      if i == m:
        break
        
  def plot(self, q = np.array([[]]), L = [], delayPerFrame = 200, repeatAnimation = False):
    """
      Plot robot's behavior and reference frames attached to each joint
      q: np.array (two - dimensional)
      delayPerFrame: int or float. Represents delay before showing next frame in milliseconds
      repeatAnimation: boolean
    """
    
    # Checks if arrays are not empty
    self._checkValues(q, L)

    # Set figure's parameters
    fig = plt.figure()
    ax = fig.gca(projection = '3d')

    # Plot robot and reference frames
    def robot(q):
      # Clear figure
      plt.cla()

      # Set title and axes text
      ax.set_title(self.name, fontsize = 16)
      ax.set_xlabel('$x$', color = 'red', fontsize = 16)
      ax.set_ylabel('$y$', color = 'green', fontsize = 16)
      ax.set_zlabel('$z$', color = 'blue', fontsize = 16)

      # Set current joints positions
      self.jointsPositions = q

      # Number of reference frames
      m = self.jointsPositions.shape[0] + 1

      # Number of generalized coordinates
      n = self.jointsPositions.shape[0]

      # Computes forward kinematics given current joints' positions
      self.forwardKinematics(self.jointsPositions.reshape((1, n)), self.linksLengths, m)
      
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
