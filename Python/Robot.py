import DenavitHartenberg as dh
import Kinematics as k
import numpy as np
import matplotlib.pyplot as plt
import Movements as mv
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from sympy import *

class System:
  
  # Object contructor
  def __init__(self, jointsPositions, linksLengths, centersOfMass, dhParameters = [], symbolicDHParameters = [], dhParametersCOM = [], symbolicDHParametersCOM = [], xi = [], xid = [], name = ''):
    
    """Object constructor

    Args:
      jointsPositions (np.array): joints set in radians
      linksLengths (list): length of each robot's link in meters
      centersOfMass (list): relative position of each center of mass
      name (str, optional): robot's name (if any)
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
    self.symbolicDHParameters = symbolicDHParameters
    self.dhParametersCOM = dhParametersCOM
    self.symbolicDHParametersCOM = symbolicDHParametersCOM
    
    # Symbolic parameters
    self.symbolicJointsPositions = np.array([[Symbol(f"q{i + 1}"),] for i in range(jointsPositions.shape[0])])
    self.symbolicLinksLengths = [Symbol(f"l{i + 1}") for i in range(len(linksLengths))]
    self.symbolicCentersOfMass = [Symbol(f"lcom{i + 1}") for i in range(len(linksLengths))]
    
    