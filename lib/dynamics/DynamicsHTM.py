# Access to parent folder to get its files
import sys, os
sys.path.append(sys.path[0].replace(r'/lib/dynamics', r''))

# Libraries
import numpy as np
from lib.kinematics.HTM import *
from sympy import *

def inertiaMatrix(robot, symbolic = False):
    """This function calculates the inertia matrix, with respect to each center of mass, given joints positions for dynamic model D(q) * q''(t) + C(q, q') * q'(t) + g(q) = τ

    Args:
        robot (object): serial robot (this won't work with other type of robots)
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        D (NumPy Matrix): inertia matrix (numerical)
        D (SymPy Matrix): inertia matrix (symbolical)
    """
    
    # Calculate forward kinematics
    fkHTM = forwardHTM(robot, symbolic)
    
    # Kinetic matrix initialization
    D = zeros(robot.jointsPositions.shape[0]) if symbolic else np.zeros((robot.jointsPositions.shape[0], robot.jointsPositions.shape[0]))
    
    # Iteration through each center of mass
    for j in range(len(robot.COMs)):
        
        # Velocity of each Center of Mass using Geometric Jacobian Matrix
        JgCOM = geometricJacobianCOM(robot, COM = j + 1, symbolic = symbolic)

        # Linear velocity mapping of current center of mass
        Jv = JgCOM[0 : 3, :]
        
        # Angular velocity mapping of current center of mass
        Jw = JgCOM[3 : 6, :]
        
        # Check in what row of Denavit Hartenberg Parameters Matrix is the current Center of Mass (the sum is because of the way Python indexes arrays)
        rowCOM = robot.whereIsTheCOM(j + 1)
        
        # Rotation Matrix of the reference frame related to the current Center of Mass
        R = fkHTM[rowCOM][0 : 3, 0 : 3]
        
        # Inertia with respect to center of mass: Icom = R^T * I * R
        Icom = R.T * robot.symbolicInertia[j] * R if symbolic else R.T.dot(robot.inertia[j]).dot(R)
        
        # (m * Jv^T * JV) + (m * Jw^T * Icom * Jw)
        D += (robot.symbolicMass[j] * (Jv.T * Jv) )+ ((Jw.T * Icom * Jw)) if symbolic else (robot.mass[j] * (Jv.T).dot(Jv)) + ((Jw.T).dot(Icom).dot(Jw))

    return D

def kineticEnergyCOM(robot, symbolic = False):
    """This function calculates the total kinetic energy, with respect to each center of mass, given linear and angular velocities

    Args:
        robot (object): serial robot (this won't work with other type of robots)
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        K (SymPy Matrix): kinetic matrix (symbolical)
    """
    
    # Kinetic Matrix calculation
    D = inertiaMatrix(robot, symbolic)
    
    return 0.5 * (robot.qdSymbolic.T * D * robot.qdSymbolic)

def potentialEnergyCOM(robot, g = np.array([[0], [0], [-9.80665]]), symbolic = False):
    """This function calculates the potential energy, with respect to each center of mass, given linear and angular velocities

    Args:
        robot (object): serial robot (this won't work with other type of robots)
        g (np.array, optional): gravity acceleration in Euclidian Space (R3)
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        P (NumPy Array): potential energy (numerical)
        P (SymPy Matrix): potential energy (symbolical)
    """
    
    # Pose of each Center of Mass
    fkCOMHTM = forwardCOMHTM(robot, symbolic)
    
    # Potential energy initialization
    P = zeros(1) if symbolic else 0
    
    # Iteration through each center of mass
    for j in range(len(robot.COMs)):
        
        # Get the row of Denavit - Hartenberg Parameters Matrix where current COM is stored
        COM = robot.whereIsTheCOM(j + 1)
        
        # Position of current Center of Mass
        r = fkCOMHTM[COM][0 : 3, -1]
        
        # m * g^T * r
        P += robot.symbolicMass[j] * ((g.T) * r) if symbolic else robot.mass[j] * ((g.T).dot(r))

    return P

def dPdq(robot, g = np.array([[0], [0], [-9.80665]]), dq = 0.001, symbolic = False):
    """This function calculates the derivative of COMs' potential energy with respect to joints positions for dynamic model D(q) * q''(t) + C(q, q') * q'(t) + g(q) = τ

    Args:
        robot (object): serial robot (this won't work with other type of robots)
        g (np.array, optional): gravity acceleration in Euclidian Space (R3). Defaults to np.array([[0], [0], [-9.80665]]).
        dq (float, optional): step size for numeric derivative calculation. Defaults to 0.001
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        dP/dq (SymPy Matrix): derivative of total energy with respect to joints positions
    """
    
    # Potential Energy
    p = potentialEnergyCOM(robot, g, symbolic)
    
    if symbolic:
        
        # Total Derivative
        return p.jacobian(robot.qSymbolic).T
    
    else:
        
        # Get number of joints (generalized coordinates)
        n = robot.jointsPositions.shape[0]
    
        # Initializes Gradient with zeros
        V = np.zeros((n, 1))
        
        # Auxiliar variable to keep original joints positions
        q = robot.jointsPositions.copy()
        
        # Iterates through all colums (generalized coordinates)
        for j in range(n):
            
            # Set increment to current generalized coordinate: z[j] = q[j] + dq
            robot.jointsPositions[j] += dq
            
            # Calculate potential energy with current step size
            P = potentialEnergyCOM(robot, g)
            
            # Calculate gradient: [P(q + dq) - P(q)] / dq
            V[j, :] = ((P - p) / dq)
            
            # Eliminates step size by copying original values from auxiliar variable
            robot.jointsPositions[:, :] = q
        
        return V

def centrifugalCoriolis(robot, dq = 0.001, symbolic = False):
    """This function calculates the Centrifugal and Coriolis matrix for dynamic model

    Args:
        robot (object): serial robot (this won't work with other type of robots)
        dq (float, optional): step size for numeric derivative calculation. Defaults to 0.001
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        C(q, q') (SymPy Matrix): Centrifugal and Coriolis matrix
    """
    
    # Inertia Matrix
    d = inertiaMatrix(robot, symbolic)
    
    # Get number of joints (generalized coordinates)
    n = robot.jointsPositions.shape[0]
    
    # Initializes derivtive matrix with zeros
    V = zeros(n)
        
    # Initialize Centrifugal and Coriolis matrix with zeros 
    C = zeros(n)
        
    # Auxiliar variable to keep original joints positions
    q = robot.jointsPositions.copy()
        
    # Iterates through all colums of inertia matrix
    for j in range(n):
        
        # If symbolic calculation was requested
        if symbolic:
            
            # Differentiates current column with respect to joints positions
            V = d[:, j].jacobian(robot.qSymbolic)

        # Else, calculate derivative numerically
        else:
            
            # Iterates through all the generalized coordinates to calculate the derivative of current column
            for k in range(n):
                
                # Set increment to current generalized coordinate: z[j] = q[j] + dq
                robot.jointsPositions[k] += dq
                    
                # Calculate inertia matrix with current step size
                D = inertiaMatrix(robot)
                    
                # Calculate derivative: [D[:, j](q + dq) - d[:, j](q)] / dq
                V[:, k] = ((D[:, j] - d[:, j]) / dq)
                    
                # Eliminates step size by copying original values from auxiliar variable
                robot.jointsPositions[:, :] = q

        # Sum the previous derivative to get the "C" matrix and multiply it by qi'(t)
        C += (V - (0.5 * V.T)) * robot.qdSymbolic[j]
            
    return C

if __name__ == '__main__':
    
    """
        THIS SECTION IS FOR TESTING PURPOSES ONLY
    """
    
    print("Z")