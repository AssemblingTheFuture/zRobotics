import numpy as np

def Tx(x = 0):
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

def Ty(y = 0):
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

def Tz(z = 0):
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

def Rx(x = 0):
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

def Ry(y = 0):
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

def Rz(z = 0):
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

def leftOperator(Q):
    '''
        Left operator for Dual Quaternions multiplication
        Q: np.array (two - dimensional)
    '''
    
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

def rightOperator(Q):
    '''
        Right operator for Dual Quaternions multiplication
        Q: np.array (two - dimensional)
    '''
    
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

def crossOperator(q):
    '''
        Cross operator for quaternions' real part multiplication
        q: np.array (two - dimensional)
    '''
    return np.array([[0, float(-q[3]), float(+q[2])],
                     [float(+q[3]), 0, float(-q[1])],
                     [float(-q[2]), float(+q[1]), 0]])

def crossOperatorExtension(q):
    '''
        Cross operator extension for quaternions' multiplication
        q: np.array (two - dimensional)
    '''
    return np.array([[0, 0, 0, 0],
                     [0, 0, float(-q[3]), float(+q[2])],
                     [0, float(+q[3]), 0, float(-q[1])],
                     [0, float(-q[2]), float(+q[1]), 0]])

def conjugate(Q):
    '''
        Conjugate operator for Dual Quaternions
        Q: np.array (two - dimensional)
    '''
    return np.array([[+ float(Q[0, 0])],
                     [- float(Q[1, 0])],
                     [- float(Q[2, 0])],
                     [- float(Q[3, 0])],
                     [+ float(Q[4, 0])],
                     [- float(Q[5, 0])],
                     [- float(Q[6, 0])],
                     [- float(Q[7, 0])]])
    
def toR3(Q):
    '''
        Transformation from Dual Quaternion to Euclidian Space Coordinates
        Q: np.array (two - dimensional)
    '''
    z = np.array([[0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 2, 0, 0, 0],
                  [0, 0, 0, 0, 0, 2, 0, 0],
                  [0, 0, 0, 0, 0, 0, 2, 0],
                  [0, 0, 0, 0, 0, 0, 0, 2]])
    qr = np.append(Q[0 : 4, 0].reshape((4, 1)), np.zeros((4, 1)), axis = 0)
    r = z.dot(leftOperator(Q)).dot(conjugate(qr))
    return r[4 : 8, 0]
