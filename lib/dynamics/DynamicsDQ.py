# Access to parent folder to get its files
import sys, os
sys.path.append(sys.path[0].replace(r'/lib/dynamics', r''))

# Libraries
import numpy as np
from lib.kinematics.DQ import *
from lib.dynamics.Solver import *
from sympy import *

def dqInertiaMatrixCOM(robot : object, symbolic = False):
    """This function calculates the inertia matrix, with respect to each center of mass, given joints positions for dynamic model D(q) * q''(t) + C(q, q') * q'(t) + g(q) = τ

    Args:
        robot (object): serial robot (this won't work with other type of robots)
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        D (NumPy Matrix): inertia matrix (numerical)
        D (SymPy Matrix): inertia matrix (symbolical)
    """
    
    # Calculate forward kinematics to each center of mass
    fkCOMDQ = forwardCOMDQ(robot, symbolic)
    
    # Inertia matrix initialization
    D = zeros(robot.jointsPositions.shape[0]) if symbolic else np.zeros((robot.jointsPositions.shape[0], robot.jointsPositions.shape[0]))
    
    # Iteration through each center of mass
    for j in range(len(robot.COMs)):
        
        # Velocity of each Center of Mass using Geometric Jacobian Matrix
        JdqCOM = jacobianVelocityDQCOM(robot, COM = j + 1, symbolic = symbolic)

        # Linear velocity mapping of current center of mass
        Jv = JdqCOM[4 : , :]
        
        # Angular velocity mapping of current center of mass
        Jw = JdqCOM[0 : 4, :]
        
        # Extended rotation matrix of the current Center of Mass (the sum is because of the way Python indexes)
        R = leftOperator(fkCOMDQ[j + 1][0 : 4, :], symbolic) * rightOperator(conjugateDQ(fkCOMDQ[j + 1][0 : 4, :], symbolic), symbolic) if symbolic else leftOperator(fkCOMDQ[j + 1][0 : 4, :]).dot(rightOperator(conjugateQ(fkCOMDQ[j + 1][0 : 4, :])))
        
        # Inertia with respect to center of mass: Icom = R^T * I * R
        Icom = R.T * robot.symbolicQuaternionInertia[j] * R if symbolic else R.T.dot(robot.quaternionInertia[j]).dot(R)
        
        # (m * Jv^T * JV) + (m * Jw^T * Icom * Jw)
        D += (robot.symbolicMass[j] * (Jv.T * Jv)) + ((Jw.T * Icom * Jw)) if symbolic else (robot.mass[j] * (Jv.T.dot(Jv))) + ((Jw.T.dot(Icom).dot(Jw)))
        
    return nsimplify(trigsimp(D).evalf(), tolerance = 1e-10) if symbolic else D

def dqKineticEnergyCOM(robot : object, symbolic = False):
    """This function calculates the total kinetic energy, with respect to each center of mass, given linear and angular velocities

    Args:
        robot (object): serial robot (this won't work with other type of robots)
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        K (SymPy Matrix): kinetic matrix (symbolical)
    """
    
    # Kinetic Matrix calculation
    D = dqInertiaMatrixCOM(robot, symbolic)
    
    return nsimplify(trigsimp(0.5 * (robot.qdSymbolic.T * D * robot.qdSymbolic)).evalf(), tolerance = 1e-10) if symbolic else 0.5 * (robot.jointsVelocities.T.dot(D).dot(robot.jointsVelocities))

def dqPotentialEnergyCOM(robot : object, g = np.array([[0], [0], [0], [-9.80665]]), symbolic = False):
    """This function calculates the potential energy, with respect to each center of mass, given linear and angular velocities

    Args:
        robot (object): serial robot (this won't work with other type of robots)
        g (np.array, optional): gravity acceleration as pure quaternion (its real part equals zero). Defaults to np.array([[0], [0], [0], [-9.80665]]).
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        P (NumPy Array): potential energy (numerical)
        P (SymPy Matrix): potential energy (symbolical)
    """
    
    # Pose of each Center of Mass
    fkCOMDQ = forwardCOMDQ(robot, symbolic)
    
    # Potential energy initialization
    P = zeros(1) if symbolic else 0
    
    # Iteration through each center of mass
    for j in range(len(robot.COMs)):
        
        # Position of current Center of Mass (the sum is because of the way Python indexes)
        r = dqToR3(fkCOMDQ[j + 1], symbolic)
        
        # m * g^T * r
        P += robot.symbolicMass[j] * ((g.T) * r) if symbolic else robot.mass[j] * ((g.T).dot(r))

    return nsimplify(trigsimp(P).evalf(), tolerance = 1e-10) if symbolic else P

def dqGravitationalCOM(robot : object, g = np.array([[0], [0], [0], [-9.80665]]), symbolic = False):
    """This function calculates the derivative of COMs' potential energy with respect to joints positions for dynamic model D(q) * q''(t) + C(q, q') * q'(t) + g(q) = τ

    Args:
        robot (object): serial robot (this won't work with other type of robots)
        g (np.array, optional): gravity acceleration as pure quaternion (its real part equals zero). Defaults to np.array([[0], [0], [0], [-9.80665]])
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        G (NumPy Array): Gravitational effects' vector
        G (SymPy Matrix): Gravitational effects' vector
    """
    
    # Get number of joints
    n = robot.jointsPositions.shape[0]
        
    # Initialize differential term with zeros
    G = zeros(n, 1) if symbolic else np.zeros((n, 1))
        
    # Iteration through each center of mass
    for j in range(len(robot.COMs)):
            
        # Jacobian matrix to current center of mass
        JdqCOM = jacobianVelocityDQCOM(robot, COM = j + 1, symbolic = symbolic)
            
        # m * (JdqCOM)^T * g
        G += robot.symbolicMass[j] * JdqCOM[4 : , :].T * g if symbolic else robot.mass[j] * (JdqCOM[4 : , :].T).dot(g)
        
    return nsimplify(trigsimp(G).evalf(), tolerance = 1e-10) if symbolic else G

def dqCentrifugalCoriolisCOM(robot : object, dq = 0.001, symbolic = False):
    """This function calculates the Centrifugal and Coriolis matrix for dynamic model

    Args:
        robot (object): serial robot (this won't work with other type of robots)
        dq (float, optional): step size for numeric derivative calculation. Defaults to 0.001
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        C(q, q') (SymPy Matrix): Centrifugal and Coriolis matrix
    """
    
    # Inertia Matrix
    d = dqInertiaMatrixCOM(robot, symbolic)
    
    # Get number of joints (generalized coordinates)
    n = robot.jointsPositions.shape[0]
    
    # Initializes derivative matrix with zeros
    V = zeros(n) if symbolic else np.zeros((n, n))
        
    # Initialize Centrifugal and Coriolis matrix with zeros 
    C = zeros(n) if symbolic else np.zeros((n, n))
        
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
                D = dqInertiaMatrixCOM(robot)
                    
                # Calculate derivative: [D[:, j](q + dq) - d[:, j](q)] / dq
                V[:, k] = ((D[:, j] - d[:, j]) / dq)
                    
                # Eliminates step size by copying original values from auxiliar variable
                robot.jointsPositions[:, :] = q

        # Sum the previous derivative to get the "C" matrix and multiply it by qi'(t)
        C += (V - (0.5 * V.T)) * robot.qdSymbolic[j] if symbolic else (V - (0.5 * V.T)) * robot.jointsVelocities[j]
            
    return nsimplify(trigsimp(C).evalf(), tolerance = 1e-10) if symbolic else C

if __name__ == '__main__':
    
    """
        THIS SECTION IS FOR TESTING PURPOSES ONLY
    """
    
    print("Z")