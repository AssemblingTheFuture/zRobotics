import numpy as np
from sympy import *

def dqTx(x = 0, symbolic = False):
    """Translation on «x» axis

    Args:
        x (float or SymPy Symbol, optional): length of displacement in meters. Defaults to zero.
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        Q (NumPy Array): Dual Quaternion (numerical)
        Q (SymPy Matrix): Dual Quaternion (symbolical)
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
        Q (NumPy Array): Dual Quaternion (numerical)
        Q (SymPy Matrix): Dual Quaternion (symbolical)
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
        Q (NumPy Array): Dual Quaternion (numerical)
        Q (SymPy Matrix): Dual Quaternion (symbolical)
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
        Q (NumPy Array): Dual Quaternion (numerical)
        Q (SymPy Matrix): Dual Quaternion (symbolical)
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
        Q (NumPy Array): Dual Quaternion (numerical)
        Q (SymPy Matrix): Dual Quaternion (symbolical)
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
        Q (NumPy Array): Dual Quaternion (numerical)
        Q (SymPy Matrix): Dual Quaternion (symbolical)
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

def leftOperator(Q : np.array, symbolic = False):
    """Left operator for Dual Quaternions multiplication

    Args:
        Q (NumPy Array  or SymPy Symbol): Dual Quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        L (NumPy Array): Left Operator (numerical)
        L (SymPy Matrix): Left Operator (symbolical)
    """
    
    if symbolic:
        
        # 1. Separates real and dual part of Dual Quaternion
        qr = Q[0 : 4]
        qd = Q[4 : 8]
        
        # 2. Computes Left Operator of Dual Quaternion's real part
        ar = Matrix([qr[0]]).row_insert(1, Matrix([[-value, ] for value in qr[1 :]])).T
        br = Matrix(qr[1 :]).col_insert(1, (qr[0] * eye(3)) + crossOperator(qr, symbolic = True))
        lr = ar.row_insert(1, br)
        
        # 3. Computes Left Operator of Dual Quaternion's dual part
        ad = Matrix([qd[0]]).row_insert(1, Matrix([[-value, ] for value in qd[1 :]])).T
        bd = Matrix(qd[1 :]).col_insert(1, (qd[0] * eye(3)) + crossOperator(qd, symbolic = True))
        ld = ad.row_insert(1, bd)
        
        # 4. Set zeros matrix
        z = zeros(4)
        
        # 5. Build matrix
        a = lr.col_insert(4, z)
        b = ld.col_insert(4, lr)
        
        return a.row_insert(4, b)
        
    else:
    
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

def rightOperator(Q : np.array, symbolic = False):
    """Right operator for Dual Quaternions multiplication

    Args:
        Q (NumPy Array  or SymPy Symbol): Dual Quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        R (NumPy Array): Right Operator (numerical)
        R (SymPy Matrix): Right Operator (symbolical)
    """
    
    if symbolic:
        
        # 1. Separates real and dual part of Dual Quaternion
        qr = Q[0 : 4]
        qd = Q[4 : 8]
        
        # 2. Computes Left Operator of Dual Quaternion's real part
        ar = Matrix([qr[0]]).row_insert(1, Matrix([[-value, ] for value in qr[1 :]])).T
        br = Matrix(qr[1 :]).col_insert(1, (qr[0] * eye(3)) - crossOperator(qr, symbolic = True))
        lr = ar.row_insert(1, br)
        
        # 3. Computes Left Operator of Dual Quaternion's dual part
        ad = Matrix([qd[0]]).row_insert(1, Matrix([[-value, ] for value in qd[1 :]])).T
        bd = Matrix(qd[1 :]).col_insert(1, (qd[0] * eye(3)) - crossOperator(qd, symbolic = True))
        ld = ad.row_insert(1, bd)
        
        # 4. Set zeros matrix
        z = zeros(4)
        
        # 5. Build matrix
        a = lr.col_insert(4, z)
        b = ld.col_insert(4, lr)
        
        return a.row_insert(4, b)
    
    else:
        
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

def crossOperator(q : np.array, symbolic = False):
    """Cross operator for quaternions' real part

    Args:
        q (NumPy Array  or SymPy Symbol): quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        c (NumPy Array): Cross Operator (numerical)
        c (SymPy Matrix): Cross Operator Matrix (symbolical)
    """

    return Matrix([[0.000, -q[3], +q[2]],
                   [+q[3], 0.000, -q[1]],
                   [-q[2], +q[1], 0.00]]) if symbolic else np.array([[0.0000000000, float(-q[3]), float(+q[2])],
                                                                     [float(+q[3]), 0.0000000000, float(-q[1])],
                                                                     [float(-q[2]), float(+q[1]), 0.0000000000]])

def dualCrossOperator(Q : np.array, symbolic = False):
    """Dual Cross operator for Dual Quaternions' real part

    Args:
        Q (NumPy Array  or SymPy Symbol): Dual Quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        C (NumPy Array): Cross Operator (numerical)
        C (SymPy Matrix): Cross Operator Matrix (symbolical)
    """
    
    if symbolic:
        
        # Cross Operator for rotational part
        Qr = crossOperatorExtension(Q[0 : 4], symbolic = True)
        
        # Cross Operator for translational part
        Qd = crossOperatorExtension(Q[4 : 8], symbolic = True)
        
        # Auxiliar matrices
        a = Qr.column_insert(4, zeros(4))
        b = Qd.column_insert(4, Qr)
        
        return a.row_insert(4, b)
    
    else:
    
        # Cross Operator for rotational part
        Qr = crossOperatorExtension(Q[0 : 4])
        
        # Cross Operator for translational part
        Qd = crossOperatorExtension(Q[4 : 8])
        
        # Auxiliar matrices
        a = np.append(Qr, np.zeros((4, 4)), axis = 1)
        b = np.append(Qd, Qr, axis = 1)
        
        
        return np.append(a, b, axis = 0)

def crossOperatorExtension(q : np.array, symbolic = False):
    """Cross operator extension for quaternions' multiplication

    Args:
        q (NumPy Array  or SymPy Symbol): quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        ce (NumPy Array): Cross Operator Extension (numerical)
        ce (SymPy Matrix): Cross Operator Extension (symbolical)
    """
    
    return Matrix([[0, 0.000, 0.000, 0.000],
                   [0, 0.000, -q[3], +q[2]],
                   [0, +q[3], 0.000, -q[1]],
                   [0, -q[2], +q[1], 0.000]]) if symbolic else np.array([[0, 0.0000000000, 0.0000000000, 0.0000000000],
                                                                         [0, 0.0000000000, float(-q[3]), float(+q[2])],
                                                                         [0, float(+q[3]), 0.0000000000, float(-q[1])],
                                                                         [0, float(-q[2]), float(+q[1]), 0.0000000000]])

def conjugateDQ(Q : np.array, symbolic = False):
    """Conjugate operator for Dual Quaternions

    Args:
        Q (NumPy Array  or SymPy Symbol): Dual Quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        Q* (NumPy Array): Conjugate Dual Quaternion (numerical)
        Q* (SymPy Matrix): Conjugate Dual Quaternion (symbolical)
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
        Q (NumPy Array  or SymPy Symbol): Dual Quaternion
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        r (NumPy Array): Position in R3 coordinates (numerical)
        r (SymPy Matrix): Position in R3 coordinates (symbolical)
    """
    
    # Extract rotation part to a dual quaternion
    qr = Q[0 : 4, 0].row_insert(4, zeros(4, 1)) if symbolic else np.append(Q[0 : 4, 0].reshape((4, 1)), np.zeros((4, 1)), axis = 0)
    
    # Transformation to R3
    r = 2 * leftOperator(Q, symbolic = True) * conjugateDQ(qr, symbolic = True)  if symbolic else 2 * (leftOperator(Q).dot(conjugateDQ(qr)))
        
    return r[4 : 8, 0]

if __name__ == '__main__':
    
  """
    THIS SECTION IS FOR TESTING PURPOSES ONLY
  """
  
  # Numerical representation of a rotation
  Q = leftOperator(dqRz(z = np.pi / 4)).dot(dqTx(x = 0.5))
  
  # Symbolical representation of a rotation
  symbolicQ = dqRz(z = 1, symbolic = True)
  
  # Dual Left Operator
  L = leftOperator(Q)
  
  # Symbolic Dual Right Operator
  R = rightOperator(symbolicQ, symbolic = True)
  
  # Dual Cross Operator
  C = dualCrossOperator(Q)
  
  # From Dual Quaternion Space to Euclidian one
  r = dqToR3(Q)
  
  print("Z")