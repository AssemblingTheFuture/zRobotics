import numpy as np
from sympy import *

def Tx(x = 0):
    """Translation on «x» axis

    Args:
        x (float): length of displacement in meters

    Returns:
        Q (np.array): Dual Quaternion
    """
    
    return np.array([[1],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0.5 * x],
                     [0],
                     [0]])

def symbolicTx(x = 0):
    """Translation on «x» axis

    Args:
        x (float): length of displacement in meters

    Returns:
        Q (SymPy Matrix): Dual Quaternion
    """
    
    return Matrix([[1],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0.5 * x],
                   [0],
                   [0]])

def Ty(y = 0):
    """Translation on «y» axis

    Args:
        y (float): length of displacement in meters

    Returns:
        Q (np.array): Dual Quaternion
    """
    
    return np.array([[1],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0.5 * y],
                     [0]])

def symbolicTy(y = 0):
    """Translation on «y» axis

    Args:
        y (float): length of displacement in meters

    Returns:
        Q (SymPy Matrix): Dual Quaternion
    """
    
    return Matrix([[1],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0.5 * y],
                   [0]])

def Tz(z = 0):
    """Translation on «z» axis

    Args:
        z (float): length of displacement in meters

    Returns:
        Q (np.array): Dual Quaternion
    """
    
    return np.array([[1],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0.5 * z]])

def symbolicTz(z = 0):
    """Translation on «z» axis

    Args:
        z (float): length of displacement in meters

    Returns:
        Q (SymPy Matrix): Dual Quaternion
    """
    
    return Matrix([[1],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0.5 * z]])

def Rx(x = 0):
    """Rotation on «x» axis

    Args:
        x (float): angle of rotation in radians

    Returns:
        Q (np.array): Dual Quaternion
    """
    
    return np.array([[np.cos(x / 2)],
                     [np.sin(x / 2)],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0]])
    
def symbolicRx(x = 0):
    """Rotation on «x» axis

    Args:
        x (float): angle of rotation in radians

    Returns:
        Q (SymPy Matrix): Dual Quaternion
    """
    
    return Matrix([[cos(x / 2)],
                   [sin(x / 2)],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0]])

def Ry(y = 0):
    """Rotation on «y» axis

    Args:
        y (float): angle of rotation in radians

    Returns:
        Q (np.array): Dual Quaternion
    """
    
    return np.array([[np.cos(y / 2)],
                     [0],
                     [np.sin(y / 2)],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0]])

def symbolicRy(y = 0):
    """Rotation on «y» axis

    Args:
        y (float): angle of rotation in radians

    Returns:
        Q (SymPy Matrix): Dual Quaternion
    """
    
    return Matrix([[cos(y / 2)],
                   [0],
                   [sin(y / 2)],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0]])

def Rz(z = 0):
    """Rotation on «z» axis

    Args:
        z (float): angle of rotation in radians

    Returns:
        Q (np.array): Dual Quaternion
    """
    
    return np.array([[np.cos(z / 2)],
                     [0],
                     [0],
                     [np.sin(z / 2)],
                     [0],
                     [0],
                     [0],
                     [0]])

def symbolicRz(z = 0):
    """Rotation on «z» axis

    Args:
        z (float): angle of rotation in radians

    Returns:
        Q (SymPy Matrix): Dual Quaternion
    """
    
    return Matrix([[cos(z / 2)],
                   [0],
                   [0],
                   [sin(z / 2)],
                   [0],
                   [0],
                   [0],
                   [0]])

def leftOperator(Q):
    """Left operator for Dual Quaternions multiplication

    Args:
        Q (np.array): Dual Quaternion

    Returns:
        L (np.array): Left Operator
    """
    
    # 1. Separates real and dual part of Dual Quaternion
    qr = Q[0 : 4]
    qd = Q[4 : 8]
    
    # 2. Computes Left Operator of Dual Quaternion's real part
    ar = np.append(qr[0], np.transpose(-qr[1 :])).reshape((1, 4))
    br = np.append(qr[1 :], (qr[0] * np.eye(3)) + crossOperator(qr), axis = 1)
    lr = np.append(ar, br, axis = 0)
    
    # 3. Computes Left Operator of Dual Quaternion's dual part
    ad = np.append(qd[0], np.transpose(-qd[1 :])).reshape((1, 4))
    bd = np.append(qd[1 :], (qd[0] * np.eye(3)) + crossOperator(qd), axis = 1)
    ld = np.append(ad, bd, axis = 0)
    
    # 4. Set zeros matrix
    z = np.zeros((4, 4))
    
    # 5. Build matrix
    a = np.append(lr, z, axis = 1)
    b = np.append(ld, lr, axis = 1)
    
    return np.append(a, b, axis = 0)

def symbolicLeftOperator(Q):
    """Left operator for Dual Quaternions multiplication

    Args:
        Q (np.array): Dual Quaternion

    Returns:
        L (SymPy Matrix): Left Operator
    """
    
    # 1. Separates real and dual part of Dual Quaternion
    qr = Q[0 : 4]
    qd = Q[4 : 8]
    
    # 2. Computes Left Operator of Dual Quaternion's real part
    ar = Matrix([qr[0]]).row_insert(1, Matrix([[-value, ] for value in qr[1 :]])).T
    br = Matrix(qr[1 :]).col_insert(1, (qr[0] * eye(3)) + symbolicCrossOperator(qr))
    lr = ar.row_insert(1, br)
    
    # 3. Computes Left Operator of Dual Quaternion's dual part
    ad = Matrix([qd[0]]).row_insert(1, Matrix([[-value, ] for value in qd[1 :]])).T
    bd = Matrix(qd[1 :]).col_insert(1, (qd[0] * eye(3)) + symbolicCrossOperator(qd))
    ld = ad.row_insert(1, bd)
    
    # 4. Set zeros matrix
    z = zeros(4)
    
    # 5. Build matrix
    a = lr.col_insert(4, z)
    b = ld.col_insert(4, lr)
    
    return a.row_insert(4, b)

def rightOperator(Q):
    """Right operator for Dual Quaternions multiplication

    Args:
        Q (np.array): Dual Quaternion

    Returns:
        R (np.array): Right Operator
    """
    
    # 1. Separates real and dual part of Dual Quaternion
    qr = Q[0 : 4]
    qd = Q[4 : 8]
    
    # 2. Computes Right Operator of Dual Quaternion's real part
    ar = np.append(qr[0], np.transpose(-qr[1 :])).reshape((1, 4))
    br = np.append(qr[1 :], (qr[0] * np.eye(3)) - crossOperator(qr), axis = 1)
    rr = np.append(ar, br, axis = 0)
    
    # 3. Computes Right Operator of Dual Quaternion's dual part
    ad = np.append(qd[0], np.transpose(-qd[1 :])).reshape((1, 4))
    bd = np.append(qd[1 :], (qd[0] * np.eye(3)) - crossOperator(qd), axis = 1)
    rd = np.append(ad, bd, axis = 0)
    
    # 4. Set zeros matrix
    z = np.zeros((4, 4))
    
    # 5. Build matrix
    a = np.append(rr, z, axis = 1)
    b = np.append(rd, rr, axis = 1)
    
    return np.append(a, b, axis = 0)

def symbolicRightOperator(Q):
    """Left operator for Dual Quaternions multiplication

    Args:
        Q (np.array): Dual Quaternion

    Returns:
        R (SymPy Matrix): Right Operator
    """
    
    # 1. Separates real and dual part of Dual Quaternion
    qr = Q[0 : 4]
    qd = Q[4 : 8]
    
    # 2. Computes Left Operator of Dual Quaternion's real part
    ar = Matrix([qr[0]]).row_insert(1, Matrix([[-value, ] for value in qr[1 :]])).T
    br = Matrix(qr[1 :]).col_insert(1, (qr[0] * eye(3)) - symbolicCrossOperator(qr))
    lr = ar.row_insert(1, br)
    
    # 3. Computes Left Operator of Dual Quaternion's dual part
    ad = Matrix([qd[0]]).row_insert(1, Matrix([[-value, ] for value in qd[1 :]])).T
    bd = Matrix(qd[1 :]).col_insert(1, (qd[0] * eye(3)) - symbolicCrossOperator(qd))
    ld = ad.row_insert(1, bd)
    
    # 4. Set zeros matrix
    z = zeros(4)
    
    # 5. Build matrix
    a = lr.col_insert(4, z)
    b = ld.col_insert(4, lr)
    
    return a.row_insert(4, b)

def crossOperator(q):
    """Cross operator for quaternions' real part

    Args:
        q (np.array): Quaternion

    Returns:
        C (np.array): Cross Operator Matrix
    """

    return np.array([[0, float(-q[3]), float(+q[2])],
                     [float(+q[3]), 0, float(-q[1])],
                     [float(-q[2]), float(+q[1]), 0]])

def symbolicCrossOperator(q):
    """Cross operator for quaternions' real part

    Args:
        Q (np.array): Dual Quaternion

    Returns:
        C (SymPy Matrix): Cross Operator Matrix
    """
    
    return Matrix([[0, -q[3], +q[2]],
                   [+q[3], 0, -q[1]],
                   [-q[2], +q[1], 0]])

def dualCrossOperator(Q):
    """Dual Cross operator for Dual Quaternions' real part

    Args:
        Q (np.array): Dual Quaternion

    Returns:
        C (np.array): Dual Cross Operator Matrix
    """
    
    # Cross Operator for rotational part
    Qr = crossOperatorExtension(Q[0 : 4])
    
    # Cross Operator for translational part
    Qd = crossOperatorExtension(Q[4 : 8])
    
    # Auxiliar matrices
    a = np.append(Qr, np.zeros((4, 4)), axis = 1)
    b = np.append(Qd, Qr, axis = 1)
    
    
    return np.append(a, b, axis = 0)

def symbolicDualCrossOperator(Q):
    """Dual Cross operator for Dual Quaternions' real part

    Args:
        Q (np.array): Dual Quaternion

    Returns:
        C (SymPy Matrix): Dual Cross Operator Matrix
    """
    
    # Cross Operator for rotational part
    Qr = symbolicCrossOperatorExtension(Q[0 : 4])
    
    # Cross Operator for translational part
    Qd = symbolicCrossOperatorExtension(Q[4 : 8])
    
    # Auxiliar matrices
    a = Qr.column_insert(4, zeros(4))
    b = Qd.column_insert(4, Qr)
    
    return a.row_insert(4, b)

def crossOperatorExtension(q):
    """Cross operator extension for quaternions' multiplication

    Args:
        q (np.array): Quaternion

    Returns:
        C (np.array): Cross Operator Extension Matrix
    """
    
    return np.array([[0, 0, 0, 0],
                     [0, 0, float(-q[3]), float(+q[2])],
                     [0, float(+q[3]), 0, float(-q[1])],
                     [0, float(-q[2]), float(+q[1]), 0]])

def symbolicCrossOperatorExtension(q):
    """Cross operator extension for quaternions' multiplication

    Args:
        q (np.array): Quaternion

    Returns:
        C (SymPy Matrix): Cross Operator Extension Matrix
    """
    
    return Matrix([[0, 0, 0, 0],
                   [0, 0, -q[3], +q[2]],
                   [0, +q[3], 0, -q[1]],
                   [0, -q[2], +q[1], 0]])

def conjugate(Q):
    """Conjugate operator for Dual Quaternions

    Args:
        Q (np.array): Dual Quaternion

    Returns:
        C (np.array): Conjugate Dual Quaternion
    """

    return np.array([[+ float(Q[0, 0])],
                     [- float(Q[1, 0])],
                     [- float(Q[2, 0])],
                     [- float(Q[3, 0])],
                     [+ float(Q[4, 0])],
                     [- float(Q[5, 0])],
                     [- float(Q[6, 0])],
                     [- float(Q[7, 0])]])

def symbolicConjugate(Q):
    """Conjugate operator for Dual Quaternions

    Args:
        Q (np.array): Dual Quaternion

    Returns:
        C (SymPy Matrix): Conjugate Dual Quaternion
    """
    
    return Matrix([[+ Q[0, 0]],
                   [- Q[1, 0]],
                   [- Q[2, 0]],
                   [- Q[3, 0]],
                   [+ Q[4, 0]],
                   [- Q[5, 0]],
                   [- Q[6, 0]],
                   [- Q[7, 0]]])
    
def toR3(Q):
    """Transformation from Dual Quaternion to Euclidian Space Coordinates

    Args:
        Q (np.array): Dual Quaternion

    Returns:
        r (np.array): Position in R3 coordinates
    """
    
    # Auxiliar matrix
    z = np.array([[0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 2, 0, 0, 0],
                  [0, 0, 0, 0, 0, 2, 0, 0],
                  [0, 0, 0, 0, 0, 0, 2, 0],
                  [0, 0, 0, 0, 0, 0, 0, 2]])
    
    # Extract rotation part to a dual quaternion
    qr = np.append(Q[0 : 4, 0].reshape((4, 1)), np.zeros((4, 1)), axis = 0)
    
    # Transformation to R3
    r = z.dot(leftOperator(Q)).dot(conjugate(qr))
    return r[4 : 8, 0]

def symbolicToR3(Q):
    """Transformation from Dual Quaternion to Euclidian Space Coordinates

    Args:
        Q (np.array): Dual Quaternion

    Returns:
        r (SymPy Matrix): Position in R3 coordinates
    """
    
    #Auxiliar matrix
    z = Matrix([[0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 2, 0, 0, 0],
                [0, 0, 0, 0, 0, 2, 0, 0],
                [0, 0, 0, 0, 0, 0, 2, 0],
                [0, 0, 0, 0, 0, 0, 0, 2]])
    
    # Extract rotation part to a dual quaternion
    qr = Q[0 : 4, 0].row_insert(4, zeros(4, 1))
    
    # Transformation to R3
    r = z * symbolicLeftOperator(Q) * symbolicConjugate(qr)
    
    return nsimplify(simplify(r[4 : 8, 0]), tolerance = 1e-10, rational = False)
