import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import csv

def solver(f, F, dt = 0.003):
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

def trajectory(P, steps, absolut = True, h = 0.003):
    
    def equation(A, t, grade, h = h):
        """
            This function computes numerically the n - th grade equation X(t) = SUM(Ai * (t ** i))
            X: np.array (two - dimensional)
            Xd: np.array (two - dimensional)
            Xdd: np.array (two - dimensional)
            t: np.array (one - dimensional)
            grade: int
        """
        m, n = A.shape
        X = []
        Xd = []
        Xdd = []
        for interval in range(1, int(m / grade)):
            time = np.linspace(t[interval - 1], t[interval], int((t[interval] - t[interval - 1]) / h))
            x = 0
            xd = 0
            xdd = 0
            Z = A[grade * (interval - 1) : grade * interval, :]
            for i in range(grade):
                x += Z[i, :] * (time ** i)
                xd += i * Z[i, :] * (time ** (i - 1))
                xdd += (i ** 2 - i) * Z[i, :] * (time ** (i - 2))
            X += list(x)
            Xd += list(xd)
            Xdd += list(xdd)
        return np.nan_to_num(np.array(X)), np.nan_to_num(np.array(Xd)), np.nan_to_num(np.array(Xdd))
    
    def parameters(M, a, B, K):
        A = a
        for i in range(15000):
            e = B - (M.dot(A))
            if all(value <= 0.001 for value in abs(e)):
                print("Done!")
                break
            else:
                A = solver(f = np.linalg.pinv(M).dot(K).dot(e), F = A)
        return A
    
    def spline(P, t, absolut):
        """
            For «M» matrix
        """
        pose = lambda t, grade : [0 if np.isnan(t ** i) else t ** i for i in range(grade)]
        velocity = lambda t, grade : [0 if np.isnan(i * (t ** (i - 1))) else i * (t ** (i - 1)) for i in range(grade)]
        acceleration = lambda t, grade : [0 if np.isnan(((i ** 2) - i) * (t ** (i - 2))) else ((i ** 2) - i) * (t ** (i - 2)) for i in range(grade)]
        
        grade = 3
        m, n = P.shape
        t = t if absolut == True else np.cumsum(t)
        v = np.zeros(((n) * (grade + 1), 1))
        
        X = []
        Xd = []
        Xdd = []
        for function in range(m):
            M = np.zeros((4 * (n), (n) * (grade + 1)))
            
            B = np.zeros((4 * (n), 1))
            
            # Initial point
            M[0 : 2, 0 : grade + 1] = np.array([pose(t[0], grade = grade + 1),
                                                acceleration(t[0], grade = grade + 1)])
            B[0, :] = np.array([[P[function, 0]]])
            
            # Intermediate points
            for interval in range(1, n):
                # «M» matrix
                M[(4 * interval) - 2 : (4 * interval) + 2, (grade + 1) * (interval - 1) : (grade + 1) * (interval + 1)] = np.array([pose(t[interval], grade = grade + 1) + [0 for i in range(grade + 1)],
                                                                                                                                    [0 for i in range(grade + 1)] + pose(t[interval], grade = grade + 1),
                                                                                                                                    velocity(t[interval], grade = grade + 1) + list(-np.array(velocity(t[interval], grade = grade + 1))),
                                                                                                                                    acceleration(t[interval], grade = grade + 1) + list(-np.array(acceleration(t[interval], grade = grade + 1)))])
                
                # Solution vector
                B[(4 * interval) - 2 : (4 * interval) + 2, :] = np.array([[P[function, interval]],
                                                                          [P[function, interval]],
                                                                          [0],
                                                                          [0]])
                
            # Final point
            M[-2 : , -(grade + 1) : ] = np.array([pose(t[-1], grade = grade + 1),
                                                  acceleration(t[-1], grade = grade + 1)])
            B[-2 : , :] = np.array([[P[function, -1]],
                                    [0]])
            
            # System's parameters
            z = np.linalg.pinv(M).dot(B)
            v = np.append(v, z if all(value <= 1e-3 for value in abs(M.dot(z) - B)) else parameters(M = M, a = v[:, -1].reshape(z.shape), B = B, K = np.eye(4 * (n))), axis = 1)
            x, xd, xdd = equation(A = v[:, -1].reshape(z.shape), t = t, grade = grade + 1)
            X.append(x)
            Xd.append(xd)
            Xdd.append(xdd)
        return np.array(X), np.array(Xd), np.array(Xdd), v
    
    return spline(P = P, t = steps, absolut = absolut)