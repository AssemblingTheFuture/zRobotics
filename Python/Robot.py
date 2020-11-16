import DenavitHartenberg as dh
import Kinematics as k
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
