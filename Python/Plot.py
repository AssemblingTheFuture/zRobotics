from Kinematics import *
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from Movements import *
import numpy as np
import os

def graph(function, title = r'', labels = r'', complement = r'', xlabel = r'', ylabel = r'', save = False, name = "zGraph", transparent = False, GUI = False):
  """
    Plots any given function
    function: np.array (two - dimensional)
    title: string
    xlabel: string
    ylabel: string
  """
  fig = plt.figure()
  plt.clf()
  plt.title(title)
  for i in range(function.shape[0]):
    plt.plot(function[i, :], label = labels + str(i + 1) + complement)
  plt.xlabel(xlabel)
  plt.ylabel(ylabel)
  plt.grid(True)
  plt.legend(loc = 'best')
  if save == True:
    if os.path.isfile('./' + name + '.png'):
      os.remove('./' + name + '.png')
    plt.savefig(name + '.png', transparent = transparent)
  
  if GUI:
    return fig
  else:
    plt.show()

def path(function, points, steps, title = r"", variable = r"", h = 0.003, GUI = False):
  """
    Plots any given path to be followed
    function: np.array (two - dimensional)
    points: np.array (two - dimensional)
    steps: np.array (one - dimensional)
    h: float
    title: string
    variable: string
  """
  fig = plt.figure()
  plt.title(title)
  t = np.cumsum(steps)
  for path in range(function.shape[0]):
    plt.plot(function[path, :], label = variable + str(path + 1) + r"$")
    plt.scatter(x = t / h, y = points[path, :], c = "red")
  plt.title(title)
  plt.xlabel(r"Time [miliseconds]")
  plt.ylabel(r"Amplitude")
  plt.legend(loc = "best")
  plt.grid()
  if GUI:
    return fig
  else:
    plt.show()

def path3D(robot, q, m, GUI = False):
  """
    Plots robot's end - effector behavior in R3 given joints' paths
    q: np.array (two - dimensional)
  """
  
  fig = plt.figure()
  ax = fig.gca(projection = '3d')
  
  fig1 = plt.figure()
  ax1 = fig1.gca()
  
  fig2 = plt.figure()
  ax2 = fig2.gca()
  
  fig3 = plt.figure()
  ax3 = fig3.gca()
  
  # Set title and axes text for position
  ax.set_title("End - effector Position", fontsize = 16)
  ax.set_xlabel(r'$x$', color = 'red', fontsize = 16)
  ax.set_ylabel(r'$y$', color = 'green', fontsize = 16)
  ax.set_zlabel(r'$z$', color = 'blue', fontsize = 16)
  
  # Set title and axes text for orientation with respect to «x»
  ax1.set_title("End - effector Orientation", fontsize = 16)
  ax1.set_xlabel(r'Time [milliseconds]', fontsize = 14)
  ax1.set_ylabel(r'x [rad]', fontsize = 14)
  
  # Set title and axes text for orientation with respect to «y»
  ax2.set_title("End - effector Orientation", fontsize = 16)
  ax2.set_xlabel(r'Time [milliseconds]', fontsize = 14)
  ax2.set_ylabel(r'y [rad]', fontsize = 14)
  
  # Set title and axes text for orientation with respect to «z»
  ax3.set_title("End - effector Orientation", fontsize = 16)
  ax3.set_xlabel(r'Time [milliseconds]', fontsize = 14)
  ax3.set_ylabel(r'z [rad]', fontsize = 14)
  
  r, s = q.shape
  
  robot.jointsPositions = q[:, 0].reshape((r, 1))
  framesHTM, fkHTM = forwardHTM(robot, m = m)
  X = axisAngle(fkHTM)
  
  for j in range(1, s):
    # Set current joints positions
    robot.jointsPositions = q[:, j].reshape((r, 1))

    # Computes forward kinematics given current joints' positions
    framesHTM, fkHTM = forwardHTM(robot, m = m)
    
    # Gets Axis - Angle vector
    X = np.append(X, axisAngle(fkHTM), axis = 1)
    
  # End - effector position (3D Plot)
  ax.plot(xs = X[0, :], ys = X[1, :], zs = X[2, :], color = 'magenta')
  
  # End - effector orientation (2D Plot)
  ax1.plot(X[3, :], color = 'red')
  ax1.grid()
  ax2.plot(X[4, :], color = 'green')
  ax2.grid()
  ax3.plot(X[5, :], color = 'blue')
  ax3.grid()
  
  if GUI:
    return fig, fig1, fig2, fig3
  else:
    plt.show()

def animation(robot, q, plotBodies = True, plotFrames = False, plotCOMs = False, delayPerFrame = 1, repeatAnimation = False, GUI = False):
    """
      Plots robot's behavior and reference frames attached to each joint and/or Center of Mass
      q: np.array (two - dimensional)
      plotBodies: boolean
      plotFrames: boolean
      plotCOMs: boolean
      delayPerFrame: int or float. Represents delay before showing next frame in milliseconds
      repeatAnimation: boolean
    """

    # Set figure's parameters
    fig = plt.figure()
    ax = fig.gca(projection = '3d')

    # Variable to increment or decrement robot dimensions and to scale its plot
    if any(link < 1 for link in robot.linksLengths):
      a = 10
    elif any(link > 100 for link in robot.linksLengths):
      a = 0.01
    else:
      a = 1
        
    # Figure limits
    limit = a * np.sqrt(sum([link ** 2 for link in robot.linksLengths]))

    # Number of reference frames
    m = q.shape[0] + 1

    # Number of generalized coordinates
    n = q.shape[0]
    
    # Set title and axes text
    ax.set_title(robot.name, fontsize = 16)
    ax.set_xlabel('$x$', color = 'red', fontsize = 16)
    ax.set_ylabel('$y$', color = 'green', fontsize = 16)
    ax.set_zlabel('$z$', color = 'blue', fontsize = 16)
      
    # Inertial frame
    ax.plot(xs = [0, 1], ys = [0, 0], zs = [0, 0], color = 'red', linestyle = 'dashed', marker = 'o')
    ax.plot(xs = [0, 0], ys = [0, 1], zs = [0, 0], color = 'green', linestyle = 'dashed', marker = 'o')
    ax.plot(xs = [0, 0], ys = [0, 0], zs = [0, 1], color = 'blue', linestyle = 'dashed', marker = 'o')
    
    # Plot robot and reference frames
    def system(q):
      # Clear figure
      plt.cla()
      
      # Sets figure limits
      ax.set_xlim(-limit, limit)
      ax.set_ylim(-limit, limit)
      ax.set_zlim(-limit, limit)
      
      # Set current joints positions
      robot.jointsPositions = q.reshape((n, 1))

      # Computes forward kinematics given current joints' positions
      framesHTM, fkHTM = forwardHTM(robot, m = m)
      
      # Computes forward kinematics to COMs given current joints' positions
      framesCOMHTM, fkCOMHTM = forwardCOMHTM(robot, m = 5)

      for frame in range(1, len(framesHTM)):
        """
          Rigid Body
        """
        if plotBodies:
          ba = a * framesHTM[frame - 1][: -1, 3]
          bb = a * framesHTM[frame + 0][: -1, 3]
          ax.plot(xs = [ba[0], bb[0]], ys = [ba[1], bb[1]], zs = [ba[2], bb[2]], color = 'brown', linewidth = 4.5, marker = 'o')
        
        """
          Joints' frames
        """
        if plotFrames:
          # x - axis
          xa = bb
          xb = xa + framesHTM[frame][: -1, 0]
          ax.plot(xs = [xa[0], xb[0]], ys = [xa[1], xb[1]], zs = [xa[2], xb[2]], color = 'red', linestyle = 'dashed', marker = '2')
        
          # y - axis
          ya = bb
          yb = ya + framesHTM[frame][: -1, 1]
          ax.plot(xs = [ya[0], yb[0]], ys = [ya[1], yb[1]], zs = [ya[2], yb[2]], color = 'green', linestyle = 'dashed', marker = '2')
        
          # z - axis
          za = bb
          zb = za + framesHTM[frame][: -1, 2]
          ax.plot(xs = [za[0], zb[0]], ys = [za[1], zb[1]], zs = [za[2], zb[2]], color = 'blue', linestyle = 'dashed', marker = '2')
        
        """
          Centers of Mass' frames
        """
        if plotCOMs:        
          # x - axis
          xCOMa = a * framesCOMHTM[frame][: -1, 3]
          xCOMb = xCOMa + framesHTM[frame][: -1, 0]
          ax.plot(xs = [xCOMa[0], xCOMb[0]], ys = [xCOMa[1], xCOMb[1]], zs = [xCOMa[2], xCOMb[2]], color = 'red', linestyle = ':')
        
          # y - axis
          yCOMa = a * framesCOMHTM[frame][: -1, 3]
          yCOMb = yCOMa + framesHTM[frame][: -1, 1]
          ax.plot(xs = [yCOMa[0], yCOMb[0]], ys = [yCOMa[1], yCOMb[1]], zs = [yCOMa[2], yCOMb[2]], color = 'green', linestyle = ':')
        
          # z - axis
          zCOMa = a * framesCOMHTM[frame][: -1, 3]
          zCOMb = zCOMa + framesHTM[frame][: -1, 2]
          ax.plot(xs = [zCOMa[0], zCOMb[0]], ys = [zCOMa[1], zCOMb[1]], zs = [zCOMa[2], zCOMb[2]], color = 'blue', linestyle = ':')
      
    # Plot or animate robot
    ani = FuncAnimation(fig, system, frames = q.T, interval = delayPerFrame, repeat = repeatAnimation)
    if GUI:
      return fig
    else:
      plt.show()