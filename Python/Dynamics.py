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