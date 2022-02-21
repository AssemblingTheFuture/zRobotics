import numpy as np
from sympy import *

def tx(x = 0):
  """Translation on «x» axis

  Args:
    x (float): length of displacement in meters

  Returns:
    H (np.array): Homogeneous Transformation Matrix
  """
  
  return np.array([[1, 0, 0, x],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

def symbolicTx(x = 0):
  """Translation on «x» axis

  Args:
    x (float): length of displacement in meters

  Returns:
    H (SymPy Matrix): Homogeneous Transformation Matrix
  """
  
  return Matrix([[1, 0, 0, x],
                 [0, 1, 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

def ty(y = 0):
  """Translation on «y» axis

  Args:
    y (float): length of displacement in meters

  Returns:
    H (np.array): Homogeneous Transformation Matrix
  """
  
  return np.array([[1, 0, 0, 0],
                   [0, 1, 0, y],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

def symbolicTy(y = 0):
  """Translation on «y» axis

  Args:
    y (float): length of displacement in meters

  Returns:
    H (SymPy Matrix): Homogeneous Transformation Matrix
  """
  
  return Matrix([[1, 0, 0, 0],
                 [0, 1, 0, y],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

def tz(z = 0):
  """Translation on «z» axis

  Args:
    z (float): length of displacement in meters

  Returns:
    H (np.array): Homogeneous Transformation Matrix
  """
  
  return np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, z],
                   [0, 0, 0, 1]])

def symbolicTz(z = 0):
  """Translation on «z» axis

  Args:
    z (float): length of displacement in meters

  Returns:
    H (SymPy Matrix): Homogeneous Transformation Matrix
  """
  
  return Matrix([[1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 1, z],
                 [0, 0, 0, 1]])

def rx(x = 0):
  """Rotation on «x» axis

  Args:
    x (float): angle of rotation in radians

  Returns:
    H (np.array): Homogeneous Transformation Matrix
  """
  
  return np.array([[1, 0, 0, 0],
                   [0, np.cos(x), -np.sin(x), 0],
                   [0, np.sin(x), +np.cos(x), 0],
                   [0, 0, 0, 1]])

def symbolicRx(x = 0):
  """Rotation on «x» axis

  Args:
    x (float): angle of rotation in radians

  Returns:
    H (SymPy Matrix): Homogeneous Transformation Matrix
  """
  
  return Matrix([[1, 0.0000, 0.00000, 0],
                 [0, cos(x), -sin(x), 0],
                 [0, sin(x), +cos(x), 0],
                 [0, 0.0000, 0.00000, 1]])

def ry(y = 0):
  """Rotation on «y» axis

  Args:
    y (float): angle of rotation in radians

  Returns:
    H (np.array): Homogeneous Transformation Matrix
  """
  return np.array([[np.cos(y), 0, +np.sin(y), 0],
                   [0.0000000, 1, 0.00000000, 0],
                   [-np.sin(y),0, np.cos(y), 0.],
                   [0.0000000, 0.0, 0.000000, 1]])

def symbolicRy(y = 0):
  """Rotation on «y» axis

  Args:
    y (float): angle of rotation in radians

  Returns:
    H (SymPy Matrix): Homogeneous Transformation Matrix
  """
  return Matrix([[cos(y), 0, +sin(y), 0],
                 [0, 1, 0, 0],
                 [-sin(y), 0, cos(y), 0],
                 [0, 0, 0, 1]])

def rz(z = 0):
  """Rotation on «z» axis

  Args:
    z (float): angle of rotation in radians

  Returns:
    H (np.array): Homogeneous Transformation Matrix
  """
  
  return np.array([[np.cos(z), -np.sin(z), 0, 0],
                   [np.sin(z), +np.cos(z), 0, 0],
                   [0.0000000, 0.00000000, 1, 0],
                   [0.0000000, 0.00000000, 0, 1]])

def symbolicRz(z = 0):
  """Rotation on «z» axis

  Args:
    z (float): angle of rotation in radians

  Returns:
    H (SymPy Matrix): Homogeneous Transformation Matrix
  """
  
  return Matrix([[cos(z), -sin(z), 0, 0],
                 [sin(z), +cos(z), 0, 0],
                 [0.0000, 0.00000, 1, 0],
                 [0.0000, 0.00000, 0, 1]])

def axisAngle(H, symbolic = False):
  """This function computes the axis - angle vector «X» using the Homogeneous Transformation Matrix of a reference frame

  Args:
    H (np.array): Homogeneous Transformation Matrix
    symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

  Returns:
    X (np.array): Axis - Angle vector
    X (SymPy Matrix): Axis - Angle vector
  """
  
  # Calculate angle of rotation
  theta = simplify(acos((H[0 : 3, 0 : 3].trace() - 1) / 2)) if symbolic else np.arccos((np.trace(H[0 : 3, 0 : 3]) - 1)/2)
  
  # Calculate axis of rotation
  n = simplify((1 / (2 * sin(theta))) * Matrix([[H[2, 1] - H[1, 2]],
                                                [H[0, 2] - H[2, 0]],
                                                [H[1 ,0] - H[0, 1]]])) if symbolic else (1/(2 * np.sin(theta))) * np.array([[H[2, 1] - H[1, 2]],
                                                                                                                            [H[0, 2] - H[2, 0]],
                                                                                                                            [H[1 ,0] - H[0, 1]]])
  
  # Append position and orientation in one single vector
  X = H[0 : 3, 3].row_insert(3, simplify(theta * n)) if symbolic else np.append(H[0 : 3, 3], theta * n)
  
  return nsimplify(X, tolerance = 1e-10, rational = False) if symbolic else X.reshape((6, 1))