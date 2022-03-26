import numpy as np
from sympy import *

# Main object
class Robot:
  
  def __init__(self, name = ''):
    """Object constructor

    Args:
      name (str, optional): robot's name (if any)
    """ 

    # Robot's name
    self.name = name   

# Inherited    
class Serial(Robot):
  
  """Serial Robot

  Args:
      Robot (obj): inheritance
  """  
  
  def __init__(self, jointsPositions : np.array, jointsVelocities : np.array, jointsAccelerations : np.array, linksLengths : list, COMs : list, mass : list, inertia : list, xi = [], xid = [], name = ''):
    """Object constructor

    Args:
      jointsPositions (np.array): joints set in radians
      jointsVelocities (np.array): velocities set in radians/second
      jointsAccelerations (np.array): accelerations set in radians/second^2
      linksLengths (list): length of each robot's link in meters
      COMs (list): relative position of each center of mass
      mass (list): mass of each rigid body in the system
      inertia (list): tensor of inertia of each rigid body in the system
      xi (np.array, optional): axes of actuation of each joint. Defaults to [].
      xid (np.array, optional): derivative of the axes of actuation of each joint. Defaults to [].
      name (str, optional): name of the robot. Defaults to ''.
    """
    
    # Robot initializer
    Robot.__init__(self, name = name)
    
    # Kinematic Parameters
    self.jointsPositions = jointsPositions
    self.linksLengths = linksLengths
    self.COMs = COMs
    
    # Dynamic Parameters
    self.jointsVelocities = jointsVelocities
    self.jointsAccelerations = jointsAccelerations
    self.mass = mass
    self.inertia = inertia
    
    # Symbolic Joints: q(t), q'(t) and q''(t)
    self.qSymbolic = Matrix([[f"q{i + 1}",] for i in range(self.jointsPositions.shape[0])])
    self.qdSymbolic = Matrix([[f"qd{i + 1}",] for i in range(self.jointsPositions.shape[0])])
    self.qddSymbolic = Matrix([[f"qdd{i + 1}",] for i in range(self.jointsPositions.shape[0])])
    
    # Symbolic Geometrical Properties
    self.symbolicLinks = Matrix([f"L{i + 1}" for i in range(len(self.linksLengths))])
    self.symbolicCOMs = Matrix([f"Lcom{i + 1}" for i in range(len(self.COMs))])
    
    # Symbolic Physical Properties
    self.symbolicMass = Matrix([[f"m{i + 1}",] for i in range(len(self.mass))])
    self.symbolicInertia = [Matrix([[f"+Ixx{i + 1}", f"-Ixy{i + 1}", f"-Ixz{i + 1}"],
                                    [f"-Ixy{i + 1}", f"+Iyy{i + 1}", f"-Iyz{i + 1}"],
                                    [f"-Ixz{i + 1}", f"-Iyz{i + 1}", f"+Izz{i + 1}"]]) for i in range(len(self.mass))]
    
    # Set Denavit - Hartenberg Parameters Matrix (numerical and symbolical)
    self.denavitHartenberg()
    self.denavitHartenberg(symbolic = True)
    
    # Set Denavit - Hartenberg Parameters Matrix to Centers of Mass (numerical and symbolical)
    self.denavitHartenbergCOM()
    self.denavitHartenbergCOM(symbolic = True)
    
    # Actuation axes
    self.xi = xi
    self.xid = xid
    
  def denavitHartenberg(self, symbolic = False):
    """Denavit - Hartenberg parameters for i - th reference frame, ordered as follows:

      theta: rotation on «z» axis
      d: translation on «z» axis
      a: translation on «x» axis
      alpha: rotation on «x» axis
    """ 
    
    if symbolic:
          
      """
      # Two-link planar robot
      self.symbolicDHParameters = Matrix([[0, 0, 0, 0],
                                          [self.qSymbolic[0, 0], 0, self.symbolicLinks[0], 0],
                                          [self.qSymbolic[1, 0], 0, self.symbolicLinks[1], 0]])
      """
      
      
      # Three-link spatial robot
      self.symbolicDHParameters = Matrix([[0, 0, 0, 0],
                                          [self.qSymbolic[0, 0], self.symbolicLinks[0], 0.0000000000000000000, np.pi / 2],
                                          [self.qSymbolic[1, 0], self.symbolicLinks[1], 0.0000000000000000000, 0.0000000],
                                          [0.000000000000000000, 0.0000000000000000000, self.symbolicLinks[2], 0.0000000],
                                          [self.qSymbolic[2, 0], self.symbolicLinks[3], 0.0000000000000000000, 0.0000000],
                                          [(np.pi / 2) + 0.0000, 0.0000000000000000000, 0.0000000000000000000, np.pi / 2],
                                          [0.000000000000000000, self.symbolicLinks[4], 0.0000000000000000000, 0.0000000]])
      
      """      
      # 4 degrees-of-freedom robot
      self.symbolicDHParameters = Matrix([[0, 0, 0, 0],
                                          [self.qSymbolic[0, 0], self.symbolicLinks[0], 0.0000000000000000000, np.pi / 2],
                                          [self.qSymbolic[1, 0], 0.0000000000000000000, self.symbolicLinks[1], 0.0000000],
                                          [self.qSymbolic[2, 0], 0.0000000000000000000, 0.0000000000000000000, np.pi / 2],
                                          [self.qSymbolic[3, 0], self.symbolicLinks[2], 0.0000000000000000000, 0.0000000]])
      """
      
    else:
      
      """
      # Two-link planar robot
      self.dhParameters = np.array([[0, 0, 0, 0],
                                    [self.jointsPositions[0, 0], 0, self.linksLengths[0], 0],
                                    [self.jointsPositions[1, 0], 0, self.linksLengths[1], 0]])
      """
      
      
      # Three-link spatial robot
      self.dhParameters = np.array([[0, 0, 0, 0],
                                    [self.jointsPositions[0, 0], self.linksLengths[0], 0.000000000000000000, np.pi / 2],
                                    [self.jointsPositions[1, 0], self.linksLengths[1], 0.000000000000000000, 0.0000000],
                                    [0.000000000000000000000000, 0.000000000000000000, self.linksLengths[2], 0.0000000],
                                    [self.jointsPositions[2, 0], self.linksLengths[3], 0.000000000000000000, 0.0000000],
                                    [(np.pi / 2) + 0.0000000000, 0.000000000000000000, 0.000000000000000000, np.pi / 2],
                                    [0.000000000000000000000000, self.linksLengths[4], 0.000000000000000000, 0.0000000]])
      
      """
      # 4 degrees-of-freedom robot
      self.dhParameters = np.array([[0, 0, 0, 0],
                                    [self.jointsPositions[0, 0], self.linksLengths[0], 0.000000000000000000, np.pi / 2],
                                    [self.jointsPositions[1, 0], 0.000000000000000000, self.linksLengths[1], 0.0000000],
                                    [self.jointsPositions[2, 0], 0.000000000000000000, 0.000000000000000000, np.pi / 2],
                                    [self.jointsPositions[3, 0], self.linksLengths[2], 0.000000000000000000, 0.0000000]])
      """
      
  def denavitHartenbergCOM(self, symbolic = False):
    """Denavit - Hartenberg parameters for j - th rigid body, ordered as follows:

      theta: rotation on «z» axis
      d: translation on «z» axis
      a: translation on «x» axis
      alpha: rotation on «x» axis
    """
    
    if symbolic:
      
      """
      # Two-link planar robot
      self.symbolicDHParametersCOM = Matrix([[0, 0, 0, 0],
                                             [self.qSymbolic[0, 0], 0, self.symbolicCOMs[0], 0],
                                             [self.qSymbolic[1, 0], 0, self.symbolicCOMs[1], 0]])
      """
      
      
      # Three-link spatial robot
      self.symbolicDHParametersCOM = Matrix([[0, 0, 0, 0],
                                             [self.qSymbolic[0, 0], self.symbolicCOMs[0], 0.000000000000000000, np.pi / 2],
                                             [self.qSymbolic[1, 0], self.symbolicCOMs[1], 0.000000000000000000, 0.0000000],
                                             [0.000000000000000000, 0.000000000000000000, self.symbolicCOMs[2], 0.0000000],
                                             [self.qSymbolic[2, 0], self.symbolicCOMs[3], 0.000000000000000000, 0.0000000],
                                             [(np.pi / 2) + 0.0000, 0.000000000000000000, 0.000000000000000000, np.pi / 2],
                                             [0.000000000000000000, self.symbolicCOMs[4], 0.000000000000000000, 0.0000000]])
      
      """
      # 4 degrees-of-freedom robot
      self.symbolicDHParametersCOM = Matrix([[0, 0, 0, 0],
                                             [self.qSymbolic[0, 0], self.symbolicCOMs[0], 0.000000000000000000, np.pi / 2],
                                             [self.qSymbolic[1, 0], 0.000000000000000000, self.symbolicCOMs[1], 0.0000000],
                                             [self.qSymbolic[2, 0], 0.000000000000000000, 0.000000000000000000, np.pi / 2],
                                             [self.qSymbolic[3, 0], self.symbolicCOMs[2], 0.000000000000000000, 0.0000000]])
      """
      
    else:
     
      """
      # Two-link planar robot
      self.dhParametersCOM = np.array([[0, 0, 0, 0],
                                       [self.jointsPositions[0, 0], 0, self.COMs[0], 0],
                                       [self.jointsPositions[1, 0], 0, self.COMs[1], 0]])
            
      """
     
      
      # Three-link spatial robot
      self.dhParametersCOM = np.array([[0, 0, 0, 0],
                                       [self.jointsPositions[0, 0], self.COMs[0], 0.0000000000, np.pi / 2],
                                       [self.jointsPositions[1, 0], self.COMs[1], 0.0000000000, 0.0000000],
                                       [0.000000000000000000000000, 0.0000000000, self.COMs[2], 0.0000000],
                                       [self.jointsPositions[2, 0], self.COMs[3], 0.0000000000, 0.0000000],
                                       [(np.pi / 2) + 0.0000000000, 0.0000000000, 0.0000000000, np.pi / 2],
                                       [0.000000000000000000000000, self.COMs[4], 0.0000000000, 0.0000000]])
      
      """
      # 4 degrees-of-freedom robot
      self.dhParametersCOM = np.array([[0, 0, 0, 0],
                                        [self.jointsPositions[0, 0], self.COMs[0], 0.0000000000, np.pi / 2],
                                        [self.jointsPositions[1, 0], 0.0000000000, self.COMs[1], 0.0000000],
                                        [self.jointsPositions[2, 0], 0.0000000000, 0.0000000000, np.pi / 2],
                                        [self.jointsPositions[3, 0], self.COMs[2], 0.0000000000, 0.0000000]])
      """
     
  def whereIsTheJoint(self, joint : int):
    """This method allows to know in which reference frame is attached any joint based on symbolic Denavit - Hartenberg Parameters Matrix, so this have to be set before calling this method

    Args:
      joint (int): number of joint we want to look for

    Returns:
      row, colum (int, int): Row and column of Denavit - Hartenberg Parameters Matrix where joint is stored
    """        
        
    # Check what frame has the i-th joint attached by iteration through all the rows in Denavit - Hartenberg symbolic matrix
    for row in range(self.dhParameters.shape[0]):
        
      # Get the current row from the symbolic Denavit - Hartenberg parameters
      frame = self.symbolicDHParameters[4 * row : 4 * (row + 1)]
        
      # If joint qi is in current reference frame
      if Symbol('q' + str(joint)) in frame:
        break
    
    # Returns the frame
    return row, frame.index(Symbol('q' + str(joint)))
  
  def whereIsTheCOM(self, COM : int):
    """This method allows to know in which reference frame is attached any Center of Mass based on symbolic Denavit - Hartenberg Parameters Matrix, so this have to be set before calling this method

    Args:
      COM (int): number of Center of Mass we want to look for

    Returns:
      row, colum (int, int): Row and column of Denavit - Hartenberg Parameters Matrix where center of mass is stored
    """        
        
    # Check what frame has the i-th joint attached by iteration through all the rows in Denavit - Hartenberg symbolic matrix
    for row in range(self.dhParameters.shape[0]):
        
      # Get the current row from the symbolic Denavit - Hartenberg parameters
      frame = self.symbolicDHParametersCOM[4 * row : 4 * (row + 1)]
        
      # If Center of Máss Lcomi is in current reference frame
      if Symbol('Lcom' + str(COM)) in frame:
        break
    
    # Returns the frame
    return row, frame.index(Symbol('Lcom' + str(COM)))
  
if __name__ == '_main__':
  
  """
    THIS SECTION IS FOR TESTING PURPOSES ONLY
  """
  
  print("Z")