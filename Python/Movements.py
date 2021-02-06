import numpy as np
from sympy import *

def tx(x = 0):
  """
    Translation on «x» axis. Returns homogeneous transformation matrix
  """
  return np.array([[1, 0, 0, x],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

def symbolicTx(x = 0):
  """
    Translation on «x» axis. Returns homogeneous transformation matrix
  """
  return Matrix([[1, 0, 0, x],
                 [0, 1, 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

def ty(y = 0):
  '''
  Translation on «y» axis. Returns homogeneous transformation matrix
  '''
  return np.array([[1, 0, 0, 0],
                   [0, 1, 0, y],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

def symbolicTy(y = 0):
  """
    Translation on «y» axis. Returns homogeneous transformation matrix
  """
  return Matrix([[1, 0, 0, 0],
                 [0, 1, 0, y],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

def tz(z = 0):
  '''
  Translation on «z» axis. Returns homogeneous transformation matrix
  '''
  return np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, z],
                   [0, 0, 0, 1]])

def symbolicTz(z = 0):
  """
    Translation on «z» axis. Returns homogeneous transformation matrix
  """
  return Matrix([[1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 1, z],
                 [0, 0, 0, 1]])

def rx(x = 0):
  '''
    Rotation on «x» axis. Returns homogeneous transformation matrix
  '''
  return np.array([[1, 0, 0, 0],
                   [0, np.cos(x), -np.sin(x), 0],
                   [0, np.sin(x), +np.cos(x), 0],
                   [0, 0, 0, 1]])

def symbolicRx(x = 0):
  """
    Rotation on «x» axis. Returns homogeneous transformation matrix
  """
  return Matrix([[1, 0, 0, 0],
                 [0, cos(x), -sin(x), 0],
                 [0, sin(x), +cos(x), 0],
                 [0, 0, 0, 1]])

def ry(y = 0):
  '''
    Rotation on «y» axis. Returns homogeneous transformation matrix
  '''
  return np.array([[np.cos(y), 0, +np.sin(y), 0],
                   [0, 1, 0, 0],
                   [-np.sin(y), 0, np.cos(y), 0],
                   [0, 0, 0, 1]])

def symbolicRy(y = 0):
  """
    Rotation on «y» axis. Returns homogeneous transformation matrix
  """
  return Matrix([[cos(y), 0, +sin(y), 0],
                 [0, 1, 0, 0],
                 [-sin(y), 0, cos(y), 0],
                 [0, 0, 0, 1]])

def rz(z = 0):
  '''
    Rotation on «z» axis. Returns homogeneous transformation matrix
  '''
  return np.array([[np.cos(z), -np.sin(z), 0, 0],
                   [np.sin(z), +np.cos(z), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

def symbolicRz(z = 0):
  """
    Rotation on «z» axis. Returns homogeneous transformation matrix
  """
  return Matrix([[cos(z), -sin(z), 0, 0],
                 [sin(z), +cos(z), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

def axisAngle(H):
  """
    This function computes the axis - angle vector «X» using the Homogeneous Transformation Matrix of a reference frame
    H: np.array (two - dimensional)
  """
  theta = np.arccos((np.trace(H[0 : 3, 0 : 3]) - 1)/2)
  n = (1/(2 * np.sin(theta))) * np.array([[H[2, 1] - H[1, 2]],
                                          [H[0, 2] - H[2, 0]],
                                          [H[1 ,0] - H[0, 1]]])
  x = np.append(H[0 : 3, 3], theta * n)
  return x.reshape((6, 1))