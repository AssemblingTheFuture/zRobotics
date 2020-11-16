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