import numpy as np
from sympy import *

def tx(x = 0, symbolic = False):
  """Translation on «x» axis

  Args:
    x (float or SymPy Symbol, optional): length of displacement in meters. Defaults to zero.
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    H (np.array): Homogeneous Transformation Matrix (numerical)
    H (SymPy Matrix): Homogeneous Transformation Matrix (symbolical)
  """
  
  return Matrix([[1, 0, 0, x],
                 [0, 1, 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]]) if symbolic else np.array([[1, 0, 0, x],
                                                           [0, 1, 0, 0],
                                                           [0, 0, 1, 0],
                                                           [0, 0, 0, 1]])

def ty(y = 0, symbolic = False):
  """Translation on «y» axis

  Args:
    y (float or SymPy Symbol, optional): length of displacement in meters. Defaults to zero.
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    H (np.array): Homogeneous Transformation Matrix (numerical)
    H (SymPy Matrix): Homogeneous Transformation Matrix (symbolical)
  """
  
  return Matrix([[1, 0, 0, 0],
                 [0, 1, 0, y],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]]) if symbolic else np.array([[1, 0, 0, 0],
                                                           [0, 1, 0, y],
                                                           [0, 0, 1, 0],
                                                           [0, 0, 0, 1]])

def tz(z = 0, symbolic = False):
  """Translation on «z» axis

  Args:
    z (float or SymPy Symbol, optional): length of displacement in meters. Defaults to zero.
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    H (np.array): Homogeneous Transformation Matrix (numerical)
    H (SymPy Matrix): Homogeneous Transformation Matrix (symbolical)
  """
  
  return Matrix([[1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 1, z],
                 [0, 0, 0, 1]]) if symbolic else np.array([[1, 0, 0, 0],
                                                           [0, 1, 0, 0],
                                                           [0, 0, 1, z],
                                                           [0, 0, 0, 1]])

def rx(x = 0, symbolic = False):
  """Rotation on «x» axis

  Args:
    x (float or SymPy Symbol, optional): angle of rotation in radians. Defaults to zero.
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    H (np.array): Homogeneous Transformation Matrix (numerical)
    H (SymPy Matrix): Homogeneous Transformation Matrix (symbolical)
  """
  
  return Matrix([[1, 0.0000, 0.00000, 0],
                 [0, cos(x), -sin(x), 0],
                 [0, sin(x), +cos(x), 0],
                 [0, 0.0000, 0.00000, 1]]) if symbolic else np.array([[1, 0, 0, 0],
                                                                      [0, np.cos(x), -np.sin(x), 0],
                                                                      [0, np.sin(x), +np.cos(x), 0],
                                                                      [0, 0, 0, 1]])

def ry(y = 0, symbolic = False):
  """Rotation on «y» axis

  Args:
    y (float or SymPy Symbol, optional): angle of rotation in radians. Defaults to zero.
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    H (np.array): Homogeneous Transformation Matrix (numerical)
    H (SymPy Matrix): Homogeneous Transformation Matrix (symbolical)
  """
  
  return Matrix([[cos(y), 0, +sin(y), 0],
                 [0.0000, 1, 0.00000, 0],
                 [-sin(y), 0, cos(y), 0],
                 [0.00000, 0, 0.0000, 1]]) if symbolic else np.array([[np.cos(y), 0, +np.sin(y), 0],
                                                                      [0.0000000, 1, 0.00000000, 0],
                                                                      [-np.sin(y),0, np.cos(y), 0.],
                                                                      [0.0000000, 0.0, 0.000000, 1]])

def rz(z = 0, symbolic = False):
  """Rotation on «z» axis

  Args:
    z (float or SymPy Symbol, optional): angle of rotation in radians. Defaults to zero.
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    H (np.array): Homogeneous Transformation Matrix (numerical)
    H (SymPy Matrix): Homogeneous Transformation Matrix (symbolical)
  """
  
  return Matrix([[cos(z), -sin(z), 0, 0],
                 [sin(z), +cos(z), 0, 0],
                 [0.0000, 0.00000, 1, 0],
                 [0.0000, 0.00000, 0, 1]]) if symbolic else np.array([[np.cos(z), -np.sin(z), 0, 0],
                                                                      [np.sin(z), +np.cos(z), 0, 0],
                                                                      [0.0000000, 0.00000000, 1, 0],
                                                                      [0.0000000, 0.00000000, 0, 1]])

def crossMatrix(r : np.array, symbolic = False):
  """Cross operator for three dimensional vectors

    Args:
        r (np.array  or SymPy Symbol): 3D vector
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        c (np.array): Cross Operator (numeric)
        c (SymPy Matrix): Cross Operator Matrix (symbolic)
  """
  return Matrix([[0.000, -r[2], +r[1]],
                  [+r[2], 0.000, -r[0]],
                  [-r[1], +r[0], 0.00]]) if symbolic else np.array([[0.0000000000, float(-r[2]), float(+r[1])],
                                                                    [float(+r[2]), 0.0000000000, float(-r[0])],
                                                                    [float(-r[1]), float(+r[0]), 0.0000000000]])

if __name__ == '__main__':

  """
    THIS SECTION IS FOR TESTING PURPOSES ONLY
  """
  
  # Numerical representation of a translation
  H = tx(x = 0.5)
  
  # Symbolical representation of a translation
  symbolicH = tx(x = 1, symbolic = True)
  
  print("Z")