import DenavitHartenberg as dh
import Kinematics as k
import numpy as np
import matplotlib.pyplot as plt
import Movements as mv
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from sympy import *

class System:
  def __init__(self, jointsPositions, linksLengths, centersOfMass, dhParameters = [], symbolicDHParameters = [], dhParametersCOM = [], symbolicDHParametersCOM = [], xi = [], xid = [], name = ''):
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
    self.symbolicDHParameters = symbolicDHParameters
    self.dhParametersCOM = dhParametersCOM
    self.symbolicDHParametersCOM = symbolicDHParametersCOM
    
    # Symbolic parameters
    self.symbolicJointsPositions = np.array([[Symbol(f"q{i + 1}"),] for i in range(jointsPositions.shape[0])])
    self.symbolicLinksLengths = [Symbol(f"l{i + 1}") for i in range(len(linksLengths))]
    self.symbolicCentersOfMass = [Symbol(f"lcom{i + 1}") for i in range(len(linksLengths))]
    
    