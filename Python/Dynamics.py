import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from Movements import *
import numpy as np
import random

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

def path(P, steps, h = 0.003, plot = False):
    """
        This function solves numerically the n - th grade equation X(t) = SUM(Ai * (t ** i))
        P: np.array (two - dimensional)
        t: np.array (one - dimensional)
    """
    
    def equation(A, t, n):
        """
            This function computes numerically the n - th grade equation X(t) = SUM(Ai * (t ** i))
            X: np.array (two - dimensional)
            t: np.array (one - dimensional)
        """
        X = 0
        for i in range(n):
            X += A[i, :] * (t ** i)
        return X
    
    """
        Check if «P» is a Homogeneous Transformation Matrix or Dual Quaternion
    """
    try:
        m, n = P.shape
    except:
        n = len(P)
        Z = []
        for matrix in P:
            Z.append(axisAngle(matrix))
        P = np.array(Z).reshape((6, n))
        m, n = P.shape
            
    t = np.cumsum(steps)
    pose = lambda t : [t ** i for i in range(n)]
    
    """
        To find each Ai, it's necessary to solve M * v = B
    """
    Z = []
    for row in range(m):
        B = []
        M = []
        for column in range(n):
            M.append(pose(t[column]))
            B.append(P[row, column])
        Z.append(equation(A = np.linalg.pinv(np.array(M)).dot(np.array(B).reshape((len(B), 1))), t = np.linspace(t[0], t[-1], int(t[-1] / h)), n = n))

    if plot:
        i = 0
        for function in Z:
            plt.plot(function)
            plt.scatter(x = t / h, y = P[i, :], c = "red")
            i += 1
        plt.grid()
        plt.show()
    return Z