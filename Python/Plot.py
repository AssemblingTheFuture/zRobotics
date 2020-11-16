import Kinematics as k
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import numpy as np

def animation(robot, q = np.array([[]]), delayPerFrame = 200, repeatAnimation = False):
    """
      Plot robot's behavior and reference frames attached to each joint
      q: np.array (two - dimensional)
      delayPerFrame: int or float. Represents delay before showing next frame in milliseconds
      repeatAnimation: boolean
    """

    # Set figure's parameters
    fig = plt.figure()
    ax = fig.gca(projection = '3d')

    # Plot robot and reference frames
    def system(q):
      # Clear figure
      plt.cla()

      # Set title and axes text
      ax.set_title(robot.name, fontsize = 16)
      ax.set_xlabel('$x$', color = 'red', fontsize = 16)
      ax.set_ylabel('$y$', color = 'green', fontsize = 16)
      ax.set_zlabel('$z$', color = 'blue', fontsize = 16)

      # Number of reference frames
      m = q.shape[0] + 1

      # Number of generalized coordinates
      n = q.shape[0]
      
      # Set current joints positions
      robot.jointsPositions = q.reshape((n, 1))

      # Computes forward kinematics given current joints' positions
      framesHTM, fkHTM = k.forwardHTM(robot, m = m)
      
      # Inertial frame
      ax.plot(xs = [0, 1], ys = [0, 0], zs = [0, 0], color = 'red', linestyle = 'dashed', marker = 'o')
      ax.plot(xs = [0, 0], ys = [0, 1], zs = [0, 0], color = 'green', linestyle = 'dashed', marker = 'o')
      ax.plot(xs = [0, 0], ys = [0, 0], zs = [0, 1], color = 'blue', linestyle = 'dashed', marker = 'o')

      # Variable to increment or decrement robot dimensions and to scale its plot
      for frame in range(len(framesHTM) - 1):
        if any(link < 1 for link in robot.linksLengths):
          a = 10
        elif any(link > 100 for link in robot.linksLengths):
          a = 0.01
        else:
          a = 1

        # Rigid body
        ba = a * framesHTM[frame][: -1, 3]
        bb = a * framesHTM[frame + 1][: -1, 3]
        
        # x - axis
        xa = bb
        xb = xa + framesHTM[frame + 1][: -1, 0]

        # y - axis
        ya = bb
        yb = ya + framesHTM[frame + 1][: -1, 1]

        # z - axis
        za = bb
        zb = za + framesHTM[frame + 1][: -1, 2]

        # Figure limits
        limit = a * np.sqrt(sum([link ** 2 for link in robot.linksLengths]))

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
    ani = FuncAnimation(fig, system, frames = q.T, interval = delayPerFrame, repeat = repeatAnimation)
    plt.show()