import numpy as np
from sympy import *

def dqTx(x = 0, symbolic = False):
    """Translation on «x» axis

    Args:
        x (float or SymPy Symbol, optional): length of displacement in meters. Defaults to zero.
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        Q (np.array): Dual Quaternion (numeric)
        Q (SymPy Matrix): Dual Quaternion (symbolic)
    """
    
    return Matrix([[1],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0.5 * x],
                   [0],
                   [0]]) if symbolic else np.array([[1],
                                                    [0],
                                                    [0],
                                                    [0],
                                                    [0],
                                                    [0.5 * x],
                                                    [0],
                                                    [0]])

def dqTy(y = 0, symbolic = False):
    """Translation on «y» axis

    Args:
        y (float or SymPy Symbol, optional): length of displacement in meters. Defaults to zero.
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        Q (np.array): Dual Quaternion (numeric)
        Q (SymPy Matrix): Dual Quaternion (symbolic)
    """
    
    return Matrix([[1],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0.5 * y],
                   [0]]) if symbolic else np.array([[1],
                                                    [0],
                                                    [0],
                                                    [0],
                                                    [0],
                                                    [0],
                                                    [0.5 * y],
                                                    [0]])

def dqTz(z = 0, symbolic = False):
    """Translation on «z» axis

    Args:
        z (float or SymPy Symbol, optional): length of displacement in meters. Defaults to zero.
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        Q (np.array): Dual Quaternion (numeric)
        Q (SymPy Matrix): Dual Quaternion (symbolic)
    """
    
    return Matrix([[1],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0.5 * z]]) if symbolic else np.array([[1],
                                                          [0],
                                                          [0],
                                                          [0],
                                                          [0],
                                                          [0],
                                                          [0],
                                                          [0.5 * z]])

def dqRx(x = 0, symbolic = False):
    """Rotation on «x» axis

    Args:
        x (float or SymPy Symbol, optional): angle of rotation in radians. Defaults to zero.
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        Q (np.array): Dual Quaternion (numeric)
        Q (SymPy Matrix): Dual Quaternion (symbolic)
    """
    
    return Matrix([[cos(x / 2)],
                   [sin(x / 2)],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0]]) if symbolic else np.array([[np.cos(x / 2)],
                                                    [np.sin(x / 2)],
                                                    [0],
                                                    [0],
                                                    [0],
                                                    [0],
                                                    [0],
                                                    [0]])

def dqRy(y = 0, symbolic = False):
    """Rotation on «y» axis

    Args:
        y (float or SymPy Symbol, optional): angle of rotation in radians. Defaults to zero.
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        Q (np.array): Dual Quaternion (numeric)
        Q (SymPy Matrix): Dual Quaternion (symbolic)
    """
    
    return Matrix([[cos(y / 2)],
                   [0],
                   [sin(y / 2)],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0]]) if symbolic else np.array([[np.cos(y / 2)],
                                                    [0],
                                                    [np.sin(y / 2)],
                                                    [0],
                                                    [0],
                                                    [0],
                                                    [0],
                                                    [0]])

def dqRz(z = 0, symbolic = False):
    """Rotation on «z» axis

    Args:
        z (float or SymPy Symbol, optional): angle of rotation in radians. Defaults to zero.
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        Q (np.array): Dual Quaternion (numeric)
        Q (SymPy Matrix): Dual Quaternion (symbolic)
    """
    
    return Matrix([[cos(z / 2)],
                   [0],
                   [0],
                   [sin(z / 2)],
                   [0],
                   [0],
                   [0],
                   [0]]) if symbolic else np.array([[np.cos(z / 2)],
                                                    [0],
                                                    [0],
                                                    [np.sin(z / 2)],
                                                    [0],
                                                    [0],
                                                    [0],
                                                    [0]])

def quaternionMultiplication(q1 : np.array, q2 : np.array, symbolic = False):
    """Quaternion multiplication: q = a * b

    Args:
        q1 (np.array) : quaternion
        q2 (np.array) : quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        q (np.array): quaternion result (numeric)
        q (SymPy Matrix): quaternion result (symbolic)
    """
    
    # Real part of quaternion
    r = (q1[0] * q2[0]) - (q1[1] * q2[1]) - (q1[2] * q2[2]) - (q1[3] * q2[3])
    
    # i
    i = (q1[0] * q2[1]) + (q1[1] * q2[0]) + (q1[2] * q2[3]) - (q1[3] * q2[2])
    
    # j
    j = (q1[0] * q2[2]) - (q1[1] * q2[3]) + (q1[2] * q2[0]) + (q1[3] * q2[1])
    
    # k
    k = (q1[0] * q2[3]) + (q1[1] * q2[2]) - (q1[2] * q2[1]) + (q1[3] * q2[0])
    
    return Matrix([r, i, j, k]) if symbolic else np.array([r, i, j, k])

def leftOperator(q : np.array, symbolic = False):
    
    """Left operator for Quaternions multiplication

    Args:
        q (np.array  or SymPy Symbol): Quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        L (np.array): Left Operator (numeric)
        L (SymPy Matrix): Left Operator (symbolic)
    """
    
    # First row of the left operator matrix
    a = Matrix([q[0], -q[1 : , :]]).T if symbolic else np.append(np.array([q[0]]), -q[1 : , :].T, axis = 1)
    
    # Second row of the left operator matrix
    b = q[1 : , :].col_insert(1, (q[0] * eye(3)) + crossOperator(q, symbolic)) if symbolic else np.append(q[1 : , :], (q[0] * np.eye(3)) + crossOperator(q), axis = 1)
    
    return a.row_insert(1, b) if symbolic else np.append(a, b, axis = 0)

def rightOperator(q : np.array, symbolic = False):
    
    """Right operator for Quaternions multiplication

    Args:
        q (np.array  or SymPy Symbol): Quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        R (np.array): Right Operator (numeric)
        R (SymPy Matrix): Right Operator (symbolic)
    """
    
    # First row of the right operator matrix
    a = Matrix([q[0], -q[1 : , :]]).T if symbolic else np.append(np.array([q[0]]), -q[1 : , :].T, axis = 1)
    
    # Second row of the right operator matrix
    b = q[1 : , :].col_insert(1, (q[0] * eye(3)) - crossOperator(q, symbolic)) if symbolic else np.append(q[1 : , :], (q[0] * np.eye(3)) - crossOperator(q), axis = 1)
    
    return a.row_insert(1, b) if symbolic else np.append(a, b, axis = 0)

def dqMultiplication(Qa : np.array, Qb : np.array, symbolic = False):
    """Dual Quaternion multiplication: Q = Qa * Qb

    Args:
        Qa (np.array) : Dual Quaternion
        Qb (np.array) : Dual Quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        Q (np.array): Dual Quaternion result (numeric)
        Q (SymPy Matrix): Dual Quaternion result (symbolic)
    """
    
    # Real part of Dual Quaternion
    r = quaternionMultiplication(Qa[0 : 4, :], Qb[0 : 4, :], symbolic)
    
    # First term of dual part
    x = quaternionMultiplication(Qa[0 : 4, :], Qb[4 : 8, :], symbolic)
    
    # Second term of dual part
    y = quaternionMultiplication(Qa[4 : 8, :], Qb[0 : 4, :], symbolic)
    
    return nsimplify(trigsimp(Matrix([[r], [x + y]])).evalf(), tolerance = 1e-10) if symbolic else np.array([r, x + y]).reshape((8, 1))

def dualLeftOperator(Q : np.array, symbolic = False):
    """Left operator for Dual Quaternions multiplication

    Args:
        Q (np.array  or SymPy Symbol): Dual Quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        L (np.array): Left Operator (numeric)
        L (SymPy Matrix): Left Operator (symbolic)
    """
    
    # 1. Separates real and dual part of Dual Quaternion
    qr = Q[0 : 4, :]
    qd = Q[4 : 8, :]
    
    # 2. Computes Left Operator of Dual Quaternion's real part
    lr = leftOperator(qr, symbolic)
    
    # 3. Computes Left Operator of Dual Quaternion's dual part
    ld = leftOperator(qd, symbolic)
    
    # 4. Append results
    a = lr.col_insert(4, zeros(4)) if symbolic else np.append(lr, np.zeros((4, 4)), axis = 1)
    b = ld.col_insert(4, lr) if symbolic else np.append(ld, lr, axis = 1)
    
    return a.row_insert(4, b) if symbolic else np.append(a, b, axis = 0)

def dualRightOperator(Q : np.array, symbolic = False):
    """Right operator for Dual Quaternions multiplication

    Args:
        Q (np.array  or SymPy Symbol): Dual Quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        R (np.array): Right Operator (numeric)
        R (SymPy Matrix): Right Operator (symbolic)
    """
    
    # 1. Separates real and dual part of Dual Quaternion
    qr = Q[0 : 4, :]
    qd = Q[4 : 8, :]
    
    # 2. Computes Right Operator of Dual Quaternion's real part
    rr = rightOperator(qr, symbolic)
    
    # 3. Computes Right Operator of Dual Quaternion's dual part
    rd = rightOperator(qd, symbolic)
    
    # 4. Append results
    a = rr.col_insert(4, zeros(4)) if symbolic else np.append(rr, np.zeros((4, 4)), axis = 1)
    b = rd.col_insert(4, rr) if symbolic else np.append(rd, rr, axis = 1)
    
    return a.row_insert(4, b) if symbolic else np.append(a, b, axis = 0)

def crossOperator(q : np.array, symbolic = False):
    """Cross operator for quaternions' real part

    Args:
        q (np.array  or SymPy Symbol): quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        c (np.array): Cross Operator (numeric)
        c (SymPy Matrix): Cross Operator Matrix (symbolic)
    """

    return Matrix([[0.000, -q[3], +q[2]],
                   [+q[3], 0.000, -q[1]],
                   [-q[2], +q[1], 0.00]]) if symbolic else np.array([[0.0000000000, float(-q[3]), float(+q[2])],
                                                                     [float(+q[3]), 0.0000000000, float(-q[1])],
                                                                     [float(-q[2]), float(+q[1]), 0.0000000000]])

def dualCrossOperator(Q : np.array, symbolic = False):
    """Dual Cross operator for Dual Quaternions

    Args:
        Q (np.array  or SymPy Symbol): Dual Quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        C (np.array): Cross Operator (numeric)
        C (SymPy Matrix): Cross Operator Matrix (symbolic)
    """
    
    # Cross Operator for real part
    Qr = crossOperatorExtension(Q[0 : 4, :], symbolic)
        
    # Cross Operator for dual part
    Qd = crossOperatorExtension(Q[4 : 8, :], symbolic)
        
    # Auxiliar matrices
    a = Matrix([[Qr, zeros(4)]]) if symbolic else np.append(Qr, np.zeros((4, 4)), axis = 1)
    b = Matrix([[Qd, Qr]]) if symbolic else np.append(Qd, Qr, axis = 1)

    return Matrix([[a], [b]]) if symbolic else np.append(a, b, axis = 0)

def crossOperatorExtension(q : np.array, symbolic = False):
    """Cross operator extension for quaternions' multiplication

    Args:
        q (np.array  or SymPy Symbol): quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        ce (np.array): Cross Operator Extension (numeric)
        ce (SymPy Matrix): Cross Operator Extension (symbolic)
    """
    
    return Matrix([[0, 0.000, 0.000, 0.000],
                   [0, 0.000, -q[3], +q[2]],
                   [0, +q[3], 0.000, -q[1]],
                   [0, -q[2], +q[1], 0.000]]) if symbolic else np.array([[0, 0.0000000000, 0.0000000000, 0.0000000000],
                                                                         [0, 0.0000000000, float(-q[3]), float(+q[2])],
                                                                         [0, float(+q[3]), 0.0000000000, float(-q[1])],
                                                                         [0, float(-q[2]), float(+q[1]), 0.0000000000]])

def conjugateQ(Q : np.array, symbolic = False):
    """Conjugate operator for Quaternions

    Args:
        Q (np.array  or SymPy Symbol): Quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        Q* (np.array): Conjugate Quaternion (numeric)
        Q* (SymPy Matrix): Conjugate Quaternion (symbolic)
    """

    return Matrix([[+ Q[0, 0]],
                   [- Q[1, 0]],
                   [- Q[2, 0]],
                   [- Q[3, 0]]]) if symbolic else np.array([[+ float(Q[0, 0])],
                                                            [- float(Q[1, 0])],
                                                            [- float(Q[2, 0])],
                                                            [- float(Q[3, 0])]])

def conjugateDQ(Q : np.array, symbolic = False):
    """Conjugate operator for Dual Quaternions

    Args:
        Q (np.array  or SymPy Symbol): Dual Quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        Q* (np.array): Conjugate Dual Quaternion (numeric)
        Q* (SymPy Matrix): Conjugate Dual Quaternion (symbolic)
    """

    return Matrix([[+ Q[0, 0]],
                   [- Q[1, 0]],
                   [- Q[2, 0]],
                   [- Q[3, 0]],
                   [+ Q[4, 0]],
                   [- Q[5, 0]],
                   [- Q[6, 0]],
                   [- Q[7, 0]]]) if symbolic else np.array([[+ float(Q[0, 0])],
                                                            [- float(Q[1, 0])],
                                                            [- float(Q[2, 0])],
                                                            [- float(Q[3, 0])],
                                                            [+ float(Q[4, 0])],
                                                            [- float(Q[5, 0])],
                                                            [- float(Q[6, 0])],
                                                            [- float(Q[7, 0])]])
    
def dqToR3(Q : np.array, symbolic = False):
    """Transformation from Dual Quaternion to Euclidian Space Coordinates

    Args:
        Q (np.array  or SymPy Symbol): Dual Quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        r (np.array): Position in R3 coordinates (numeric)
        r (SymPy Matrix): Position in R3 coordinates (symbolic)
    """
    
    # Extract rotation part to a dual quaternion
    qr = Matrix([Q[0 : 4, :], zeros(4, 1)]) if symbolic else np.append(Q[0 : 4, 0].reshape((4, 1)), np.zeros((4, 1)), axis = 0)
    
    # Transformation to R3
    r = 2 * dqMultiplication(Q, conjugateDQ(qr), symbolic)
        
    return nsimplify(trigsimp(r[4 : 8, 0]).evalf(), tolerance = 1e-10) if symbolic else r[4 : 8, 0]

if __name__ == '__main__':
    
  """
    THIS SECTION IS FOR TESTING PURPOSES ONLY
  """
   
  # Numeric representation of a rotation
  Q = dualLeftOperator(dqRz(z = np.pi / 4)).dot(dqTx(x = 0.5))
  
  # Fast dual quaternion multiplication
  Qmult = dqMultiplication(dqRz(z = np.pi / 4), dqTx(x = 0.5))
  
  # Symbolic representation of a rotation
  symbolicQ = dqRz(z = Symbol('z'), symbolic = True)
  
  # Left Operator
  l = leftOperator(Q[0 : 4, :])
  
  # Right Operator
  r = rightOperator(symbolicQ[0 : 4, :], symbolic = True)
  
  # Dual Left Operator
  L = dualLeftOperator(Q)
  
  # Symbolic Dual Right Operator
  R = dualRightOperator(symbolicQ, symbolic = True)
  
  # Dual Cross Operator
  C = dualCrossOperator(Q)
  
  # From Dual Quaternion Space to Euclidian one
  r = dqToR3(Q)
  
  print("Z")