import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def solver(f, F, dt):
    """
        This function solves numerically the differential equation using Runge - Kutta (4th order)
        f: np.array (two - dimensional)
        F: np.array (two - dimensional)
        dt: float
    """
    x, y = f.shape
    for i in range(x):
        for j in range(y):
            k1 = f[i, j]
            k2 = f[i, j] + (0.5 * k1 * dt)
            k3 = f[i, j] + (0.5 * k2 * dt)
            k4 = f[i, j] + (k3 * dt)
            F[i, j] = F[i, j] + ((1 / 6) * (k1 + (2 * k2) + (2 * k3) + k4) * dt)
    return F

def trajectory(P, steps, grade = 5, h = 0.003):
    
    def equation(A, t, grade):
        """
            This function computes numerically the n - th grade equation X(t) = SUM(Ai * (t ** i))
            X: np.array (two - dimensional)
            Xd: np.array (two - dimensional)
            Xdd: np.array (two - dimensional)
            t: np.array (one - dimensional)
            grade: int
        """
        X = 0
        Xd = 0
        Xdd = 0
        for i in range(grade):
            X += (A[i, :] * (t ** i))
            Xd += (i * A[i, :] * (t ** (i - 1)))
            Xdd += ((i ** 2 - i) * A[i, :] * (t ** (i - 2)))
        return X, Xd, Xdd
    
    def parameters(M, a, B, K):
        A = a
        for i in range(15000):
            e = B - (M.dot(A[:, -1].reshape(a.shape)))
            A = np.append(A, solver(f = np.linalg.pinv(M).dot(K).dot(e), F = A[:, -1].reshape(a.shape), dt = 0.003), axis = 1)
            if all(value <= 0.001 for value in abs(e)):
                break
        return A
    
    """
        For «M» matrix
    """
    pose = lambda t, grade : [0 if np.isnan(t ** i) else t ** i for i in range(grade)]
    velocity = lambda t, grade : [0 if np.isnan(i * (t ** (i - 1))) else i * (t ** (i - 1)) for i in range(grade)]
    acceleration = lambda t, grade : [0 if np.isnan((i ** 2 - i) * t ** (i - 2)) else (i ** 2 - i) * (t ** (i - 2)) for i in range(grade)]
    
    """
        System's parameters
    """
    A = np.zeros((grade + 1, 1))
    m, n = P.shape
    t = np.cumsum(steps)
    X = []
    Xd = []
    Xdd = []
    for row in range(m):
        B = np.zeros((n, 1))
        M = np.zeros((n, grade + 1))
        for column in range(n):
            M[(column), :] = np.array(pose(t[column], grade + 1))
            B[(column), :] = P[row, column]
        A = np.append(A, parameters(M = M, B = B, a = A[:, -1].reshape((grade + 1, 1)), K = np.eye(n))[:, -1].reshape((grade + 1, 1)), axis = 1)
        x, xd, xdd = equation(A = A[:, -1].reshape((grade + 1, 1)), t = np.linspace(t[0], t[-1], int(t[-1] / h)), grade = grade + 1)
        X.append(x)
        Xd.append(xd)
        Xdd.append(xdd)
    return np.array(X), np.array(Xd), np.array(Xdd), A