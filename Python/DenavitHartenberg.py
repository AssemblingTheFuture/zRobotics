import numpy as np

def matrix(q, L):
    """
      Denavit - Hartenberg parameters for n - th rigid body
      theta: rotation on «z» axis
      d: translation on «z» axis
      a: translation on «x» axis
      alpha: rotation on «x» axis
    """
    return np.array([[0, 0, 0, 0],
                     [q[0, 0], L[0], 0, np.pi / 2],
                     [q[0, 1], 0, L[1], 0],
                     [q[0, 2], 0, 0, np.pi / 2],
                     [q[0, 3], L[2], 0, 0]])