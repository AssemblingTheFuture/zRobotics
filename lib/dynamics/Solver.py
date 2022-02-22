import numpy as np

def rungeKutta4(f, F, dt = 0.001):
    """This function solves numerically a differential equation using Runge - Kutta (4th order)

    Args:
        f (np.array): differential equation (numerical)
        F (np.array): initial conditions (value already known by user)
        dt (float, optional): step size for calculation

    Returns:
        F (np.array): solution of differential equation
    """
    
    # Get the shape of the differential equation
    x, y = f.shape
    
    # Iterates through all the rows (i)
    for i in range(x):
        
        # Iterates through all the columns (j)
        for j in range(y):
            
            # First term
            k1 = f[i, j]
            
            # Second term
            k2 = f[i, j] + (0.5 * k1 * dt)
            
            # Third term
            k3 = f[i, j] + (0.5 * k2 * dt)
            
            # Fourth term
            k4 = f[i, j] + (k3 * dt)
            
            # Numerical solution of current cell (i, j) of differential equation "f"
            F[i, j] = F[i, j] + ((1 / 6) * (k1 + (2 * k2) + (2 * k3) + k4) * dt)

    return F

if __name__ == '__main__':
    
    """
        THIS SECTION IS FOR TESTING PURPOSES ONLY
    """
    
    print("Z")