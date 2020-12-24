import DenavitHartenberg as dh
import Kinematics as k
import numpy as np
import matplotlib.pyplot as plt
import Movements as mv
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

class System:
  def __init__(self, jointsPositions, linksLengths, centersOfMass, dhParameters = np.array([[0, 0, 0, 0]]), dhParametersCOM = np.array([[0, 0, 0, 0]]), xi = [], xid = [], name = ''):
    """
      Robot's constructor
      jointsPositions: np.array (two - dimensional)
      linksLengths: list (one - dimensional)
      centersOfMass: list (one - dimensional)
      name: string (optional)
    """
    # Robot's name
    self.name = name

    # Kinematic Parameters
    self.jointsPositions = jointsPositions
    self.linksLengths = linksLengths
    self.centersOfMass = centersOfMass
    
    # Actuation axes
    self.xi = xi
    self.xid = xid
    
    # Denavit - Hartenberg Parameters
    self.dhParameters = dhParameters
    self.dhParametersCOM = dhParametersCOM