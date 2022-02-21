import numpy as np
from sympy import *

class System:
  
  # Object contructor
  def __init__(self, jointsPositions, linksLengths, COMs, dhParameters = [], symbolicDHParameters = [], dhParametersCOM = [], symbolicDHParametersCOM = [], xi = [], xid = [], name = ''):
    """Object constructor

    Args:
      jointsPositions (np.array): joints set in radians
      linksLengths (list): length of each robot's link in meters
      COM (list): relative position of each center of mass
      name (str, optional): robot's name (if any)
    """ 

    # Robot's name
    self.name = name

    # Kinematic Parameters
    self.jointsPositions = jointsPositions
    self.linksLengths = linksLengths
    self.COMs = COMs
    
    # Symbolic parameters
    self.qSymbolic = Matrix([[f"q{i + 1}",] for i in range(self.jointsPositions.shape[0])])
    self.qdSymbolic = Matrix([[f"qd{i + 1}",] for i in range(self.jointsPositions.shape[0])])
    self.symbolicLinks = Matrix([f"L{i + 1}" for i in range(len(self.linksLengths))])
    self.symbolicCOMs = Matrix([f"Lcom{i + 1}" for i in range(len(self.COMs))])
    
    # Actuation axes
    self.xi = xi
    self.xid = xid
    
  def denavitHartenberg(self):
    """Denavit - Hartenberg parameters for n - th rigid body

      theta: rotation on «z» axis
      d: translation on «z» axis
      a: translation on «x» axis
      alpha: rotation on «x» axis
    """ 

    self.dhParameters = np.array([[0, 0, 0, 0],
                                  [self.jointsPositions[0, 0], self.linksLengths[0], 0.000000000000000000, np.pi / 2],
                                  [self.jointsPositions[1, 0], 0.000000000000000000, self.linksLengths[1], 0.0000000],
                                  [self.jointsPositions[2, 0], 0.000000000000000000, 0.000000000000000000, np.pi / 2],
                                  [self.jointsPositions[3, 0], self.linksLengths[2], 0.000000000000000000, 0.0000000]])
    
  def symbolicDenavitHartenberg(self):
    """Denavit - Hartenberg parameters for n - th rigid body

      theta: rotation on «z» axis
      d: translation on «z» axis
      a: translation on «x» axis
      alpha: rotation on «x» axis
    """ 
    self.symbolicDH = Matrix([[0, 0, 0, 0],
                              [self.qSymbolic[0, 0], self.symbolicLinks[0], 0.0000000000000000000, np.pi / 2],
                              [self.qSymbolic[1, 0], 0.0000000000000000000, self.symbolicLinks[1], 0.0000000],
                              [self.qSymbolic[2, 0], 0.0000000000000000000, 0.0000000000000000000, np.pi / 2],
                              [self.qSymbolic[3, 0], self.symbolicLinks[2], 0.0000000000000000000, 0.0000000]])
    
  def denavitHartenbergCOM(self):
    """Denavit - Hartenberg parameters for n - th center of mass

      theta: rotation on «z» axis
      d: translation on «z» axis
      a: translation on «x» axis
      alpha: rotation on «x» axis
    """ 
    
    self.dhParametersCOM = np.array([[0, 0, 0, 0],
                                     [self.jointsPositions[0, 0], self.COMs[0], 0.0000000000, np.pi / 2],
                                     [self.jointsPositions[1, 0], 0.0000000000, self.COMs[1], 0.0000000],
                                     [self.jointsPositions[2, 0], 0.0000000000, 0.0000000000, np.pi / 2],
                                     [self.jointsPositions[3, 0], self.COMs[2], 0.0000000000, 0.0000000]])
    
  def symbolicDenavitHartenbergCOM(self):
    """Denavit - Hartenberg parameters for n - th center of mass

      theta: rotation on «z» axis
      d: translation on «z» axis
      a: translation on «x» axis
      alpha: rotation on «x» axis
    """ 
    self.symbolicDHParametersCOM = Matrix([[0, 0, 0, 0],
                                           [self.qSymbolic[0, 0], self.symbolicCOMs[0], 0.000000000000000000, np.pi / 2],
                                           [self.qSymbolic[1, 0], 0.000000000000000000, self.symbolicCOMs[1], 0.0000000],
                                           [self.qSymbolic[2, 0], 0.000000000000000000, 0.000000000000000000, np.pi / 2],
                                           [self.qSymbolic[3, 0], self.symbolicCOMs[2], 0.000000000000000000, 0.0000000]])