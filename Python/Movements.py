import numpy as np

def tx(x = 0):
  """
    Translation on «x» axis. Returns homogeneous transformation matrix
  """
  return np.array([[1, 0, 0, x],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

def ty(y = 0):
  '''
  Translation on «y» axis
  '''
  return np.array([[1, 0, 0, 0],
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

def rx(x = 0):
  '''
  Rotation on «x» axis. Returns homogeneous transformation matrix
  '''
  return np.array([[1, 0, 0, 0],
                   [0, np.cos(x), -np.sin(x), 0],
                   [0, np.sin(x), +np.cos(x), 0],
                   [0, 0, 0, 1]])

def ry(y = 0):
  '''
    Rotation on «y» axis. Returns homogeneous transformation matrix
  '''
  return np.array([[np.cos(y), 0, +np.sin(y), 0],
                   [0, 1, 0, 0],
                   [-np.sin(y), 0, np.cos(y), 0],
                   [0, 0, 0, 1]])

def rz(z = 0):
  '''
    Rotation on «z» axis. Returns homogeneous transformation matrix
  '''
  return np.array([[np.cos(z), -np.sin(z), 0, 0],
                   [np.sin(z), +np.cos(z), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

def dqTx(x = 0):
  '''
    Translation on «x» axis. Returns Dual Quaternion in matrix form
  '''
  return np.array([[1],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0.5 * x],
                   [0],
                   [0]])

def dqTy(y = 0):
  '''
    Translation on «y» axis. Returns Dual Quaternion in matrix form
  '''
  return np.array([[1],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0.5 * y],
                   [0]])

def dqTz(z = 0):
  '''
    Translation on «z» axis. Returns Dual Quaternion in matrix form
  '''
  return np.array([[1],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0.5 * z]])

def dqRx(x = 0):
  '''
    Rotation on «x» axis. Returns Dual Quaternion in matrix form
  '''
  return np.array([[np.cos(x / 2)],
                   [np.sin(x / 2)],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0]])

def dqRy(y = 0):
  '''
    Rotation on «y» axis. Returns Dual Quaternion in matrix form
  '''
  return np.array([[np.cos(y / 2)],
                   [0],
                   [np.sin(y / 2)],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0]])

def dqRz(z = 0):
  '''
    Rotation on «z» axis. Returns Dual Quaternion in matrix form
  '''
  return np.array([[np.cos(z / 2)],
                   [0],
                   [0],
                   [np.sin(z / 2)],
                   [0],
                   [0],
                   [0],
                   [0]])

def axisAngle(H):
  """
    This function computes the axis - angle vector «X» using the Homogeneous Transformation Matrix «H» of a reference frame
  """
  theta = np.arccos((np.trace(H[0 : 2, 0 : 2]) - 1)/2)
  n = (1/(2 * np.sin(theta))) * np.array([[H[2, 1] - H[1, 2]],
                                          [H[0, 2] - H[2, 0]],
                                          [H[1 ,0] - H[0, 1]]])
  x = np.array([H[0 : 2, 3],
                [theta * n]])
  return x