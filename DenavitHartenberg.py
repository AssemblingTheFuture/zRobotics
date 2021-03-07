import numpy as np

def matrix(robot):
    """
      Denavit - Hartenberg parameters for n - th rigid body
      theta: rotation on «z» axis
      d: translation on «z» axis
      a: translation on «x» axis
      alpha: rotation on «x» axis
    """
    return np.array([[0, 0, 0, 0],
                     [robot.jointsPositions[0, 0], robot.linksLengths[0], 0, np.pi / 2],
                     [robot.jointsPositions[1, 0], 0, robot.linksLengths[1], 0],
                     [robot.jointsPositions[2, 0], 0, 0, np.pi / 2],
                     [robot.jointsPositions[3, 0], robot.linksLengths[2], 0, 0]])

def centersOfMass(robot):
  """
    Denavit - Hartenberg parameters for n - th center of mass
    theta: rotation on «z» axis
    d: translation on «z» axis
    a: translation on «x» axis
    alpha: rotation on «x» axis
  """
  return np.array([[0, 0, 0, 0],
                   [robot.jointsPositions[0, 0], robot.centersOfMass[0], 0, np.pi / 2],
                   [robot.jointsPositions[1, 0], 0, robot.centersOfMass[1], 0],
                   [robot.jointsPositions[2, 0], 0, 0, np.pi / 2],
                   [robot.jointsPositions[3, 0], robot.centersOfMass[2], 0, 0]])
  
def symbolicMatrix(robot):
  """
    Denavit - Hartenberg parameters for n - th rigid body
    theta: rotation on «z» axis
    d: translation on «z» axis
    a: translation on «x» axis
    alpha: rotation on «x» axis
  """
  return np.array([[0, 0, 0, 0],
                   [robot.symbolicJointsPositions[0, 0], robot.symbolicLinksLengths[0], 0, np.pi / 2],
                   [robot.symbolicJointsPositions[1, 0], 0, robot.symbolicLinksLengths[1], 0],
                   [robot.symbolicJointsPositions[2, 0], 0, 0, np.pi / 2],
                   [robot.symbolicJointsPositions[3, 0], robot.symbolicLinksLengths[2], 0, 0]])

def symbolicCentersOfMass(robot):
  """
    Denavit - Hartenberg parameters for n - th center of mass
    theta: rotation on «z» axis
    d: translation on «z» axis
    a: translation on «x» axis
    alpha: rotation on «x» axis
  """
  return np.array([[0, 0, 0, 0],
                   [robot.symbolicJointsPositions[0, 0], robot.symbolicCentersOfMass[0], 0, np.pi / 2],
                   [robot.symbolicJointsPositions[1, 0], 0, robot.symbolicCentersOfMass[1], 0],
                   [robot.symbolicJointsPositions[2, 0], 0, 0, np.pi / 2],
                   [robot.symbolicJointsPositions[3, 0], robot.symbolicCentersOfMass[2], 0, 0]])