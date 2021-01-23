from Kinematics import *
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

def path(P, steps, title = r"", variable = r"", h = 0.003, plot = False):
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
            X += (A[i, :] * (t ** i))
        return X
    
    """
        Check «P» dimensions
    """
    m, n = P.shape
    
    t = np.cumsum(steps)
    pose = lambda t, n : [0 if np.isnan(t ** i) else t ** i for i in range(n)]
    
    """
        To find each Ai, it's necessary to solve M * v = B
    """
    Z = []
    for row in range(m):
        B = np.zeros((1 * n, 1))
        M = np.zeros((1 * n, 6))
        for column in range(n):
            M[column, :] = np.array(pose(t[column], 6))
            B[column, :] = P[row, column]
        Z.append(equation(A = np.linalg.pinv(M).dot(B), t = np.linspace(t[0], t[-1], int(t[-1] / h)), n = 6))
    
    Z = np.array(Z)
    
    if plot:
        for function in range(Z.shape[0]):
            plt.plot(Z[function, :], label = variable + str(function + 1) + r"$")
            plt.scatter(x = t / h, y = P[function, :], c = "red")
        plt.title(title)
        plt.xlabel(r"Time [miliseconds]")
        plt.ylabel(r"Amplitude")
        plt.legend(loc = "best")
        plt.grid()
        plt.show()
    return Z
