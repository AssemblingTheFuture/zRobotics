# Access to parent folder to get its files
import sys, os
sys.path.append(sys.path[0].replace(r'/lib/dynamics', r''))

# Libraries
import numpy as np
from lib.kinematics.HTM import *
from lib.kinematics.DifferentialHTM import *
from sympy import *

def inertiaMatrixCOM(robot : object, symbolic = False):
    """This function calculates the inertia matrix, with respect to each center of mass, given joints positions for dynamic model D(q) * q''(t) + C(q, q') * q'(t) + g(q) = τ

    Args:
        robot (object): serial robot (this won't work with other type of robots)
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        D (NumPy Matrix): inertia matrix (numerical)
        D (SymPy Matrix): inertia matrix (symbolical)
    """
    
    # Calculate forward kinematics to each center of mass
    fkCOMHTM = forwardCOMHTM(robot, symbolic = symbolic)
    
    # Inertia matrix initialization
    D = zeros(robot.jointsPositions.shape[0]) if symbolic else np.zeros((robot.jointsPositions.shape[0], robot.jointsPositions.shape[0]))
    
    # Iteration through each center of mass
    for j in range(len(robot.COMs)):
        
        # Velocity of each Center of Mass using Geometric Jacobian Matrix
        JgCOM = geometricJacobianCOM(robot, COM = j + 1, symbolic = symbolic)

        # Linear velocity mapping of current center of mass
        Jv = JgCOM[0 : 3, :]
        
        # Angular velocity mapping of current center of mass
        Jw = JgCOM[3 : 6, :]
        
        # Rotation matrix of the current Center of Mass (the sum is because of the way Python indexes arrays)
        R = fkCOMHTM[j + 1][0 : 3, 0 : 3]
        
        # Inertia with respect to center of mass: Icom = R^T * I * R
        Icom = R.T * robot.symbolicInertia[j] * R if symbolic else R.T.dot(robot.inertia[j]).dot(R)
        
        # (m * Jv^T * JV) + (m * Jw^T * Icom * Jw)
        D += (robot.symbolicMass[j] * (Jv.T * Jv) )+ ((Jw.T * Icom * Jw)) if symbolic else (robot.mass[j] * (Jv.T.dot(Jv))) + ((Jw.T.dot(Icom).dot(Jw)))
        
    return D

def kineticEnergyCOM(robot : object, symbolic = False):
    """This function calculates the total kinetic energy, with respect to each center of mass, given linear and angular velocities

    Args:
        robot (object): serial robot (this won't work with other type of robots)
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        K (SymPy Matrix): kinetic matrix (symbolical)
    """
    
    # Kinetic Matrix calculation
    D = inertiaMatrixCOM(robot, symbolic)
    
    return 0.5 * (robot.qdSymbolic.T * D * robot.qdSymbolic) if symbolic else 0.5 * (robot.jointsVelocities.T.dot(D).dot(robot.jointsVelocities))

def potentialEnergyCOM(robot : object, g = np.array([[0], [0], [-9.80665]]), symbolic = False):
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
        
        # Position of current Center of Mass (the sum is because of the way Python indexes arrays)
        r = fkCOMHTM[j + 1][0 : 3, -1]
        
        # m * g^T * r
        P += robot.symbolicMass[j] * ((g.T) * r) if symbolic else robot.mass[j] * ((g.T).dot(r))

    return P

def dPdqCOM(robot : object, g = np.array([[0], [0], [-9.80665]]), dq = 0.001, symbolic = False):
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
        
        # Get number of joints
        n = robot.jointsPositions.shape[0]
        
        # Initialize differential term with zeros
        dP_dq = np.zeros((n, 1))
        
        # Iteration through each center of mass
        for j in range(len(robot.COMs)):
            
            # Jacobian matrix to current center of mass
            JgCOM = geometricJacobianCOM(robot, COM = j + 1, symbolic = symbolic)
            
            # m * (JvCOM)^T * g
            dP_dq += robot.symbolicMass[j] * JgCOM[0 : 3, :].T * g if symbolic else robot.mass[j] * (JgCOM[0 : 3, :].T).dot(g)
        
        return dP_dq

def centrifugalCoriolisCOM(robot : object, dq = 0.001, symbolic = False):
    """This function calculates the Centrifugal and Coriolis matrix for dynamic model

    Args:
        robot (object): serial robot (this won't work with other type of robots)
        dq (float, optional): step size for numeric derivative calculation. Defaults to 0.001
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        C(q, q') (SymPy Matrix): Centrifugal and Coriolis matrix
    """
    
    # Inertia Matrix
    d = inertiaMatrixCOM(robot, symbolic)
    
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
                D = inertiaMatrixCOM(robot)
                    
                # Calculate derivative: [D[:, j](q + dq) - d[:, j](q)] / dq
                V[:, k] = ((D[:, j] - d[:, j]) / dq)
                    
                # Eliminates step size by copying original values from auxiliar variable
                robot.jointsPositions[:, :] = q

        # Sum the previous derivative to get the "C" matrix and multiply it by qi'(t)
        C += (V - (0.5 * V.T)) * robot.qdSymbolic[j] if symbolic else (V - (0.5 * V.T)) * robot.jointsVelocities[j]
            
    return C

def newtonEuler(robot : object, w0 = np.zeros((3, 1)), dw0 = np.zeros((3, 1)), dv0 = np.array([[0], [0], [-9.80665]]), f0 = np.zeros((3, 1)), T0 = np.zeros((3, 1)), symbolic = False):
    """This function calculates the Newton - Euler algorithm using Homogeneous Transformation Matrices

    Args:
        robot (object): serial robot (this won't work with other type of robots)
        w0 (np.array): initial angular velocity of the system (equals to zero if the robot's base is not mobile). Defaults to np.zeros((3, 1)).
        dw0 (np.array): initial angular acceleration of the system (equals to zero if the robot's base is not mobile). Defaults to np.zeros((3, 1)).
        dv0 (np.array): initial linear acceleration of the system (gravitational effects have to be considered). Defaults to np.array([[0], [0], [-9.80665]]).
        f0 (np.array): force in the end effector. Defaults to np.zeros((3, 1))
        T0 (np.array): torque in the end effector. Defaults to np.zeros((3, 1))
        symbolic (bool, optional): used to calculate symbolic equations. Defaults to False.

    Returns:
        f (NumPy Array): forces in each link
        f (SymPy Matrix): forces in each link
    """
    
    # Pose of each moving frame
    fkHTM = forwardHTM(robot, symbolic)
    
    # Pose of each center of mass
    fkCOMHTM = forwardCOMHTM(robot, symbolic)
    
    # Get number of reference frames
    m = robot.dhParameters.shape[0]
    
    # Create empty list of wrenches
    Wrench = [zeros(6, 1)] * m if symbolic else [np.zeros((6, 1))] * m
    
    # Initialize the force calculation with the end-effector force
    Wrench[-1][0 : 3, :] = f0
    
    # Initialize the torque calculation with the end-effector force
    Wrench[-1][3 : 6, :] = T0
    
    # Calculate velocities and accelerations
    if symbolic:
        
        # Inertial angular velocity propagation to each reference frame 
        W = angularVelocityPropagation(robot, w0 = w0, qd = robot.qdSymbolic, symbolic = symbolic)
        
        # Inertial angular velocity propagation to each center of mass
        Wcom = angularVelocityPropagationCOM(robot, wCOM0 = w0, W = W, qd = robot.qdSymbolic, symbolic = symbolic)
        
        # Inertial angular acceleration propagation to each reference frame
        dW = angularAccelerationPropagation(robot, dw0 = dw0, W = W, qd = robot.qdSymbolic, qdd = robot.qddSymbolic, symbolic = symbolic)
        
        # Inertial linear acceleration propagation to each reference frame attached to joints
        dV = linearAccelerationPropagation(robot, dv0 = dv0, W = W, dW = dW, symbolic = symbolic)
        
        # Inertial angular velocity propagation to each center of mass
        Wcom = angularVelocityPropagationCOM(robot, wCOM0 = w0, W = W, qd = robot.qdSymbolic, symbolic = symbolic)
        
        # Inertial angular acceleration propagation to each reference frame
        dWcom = angularAccelerationPropagationCOM(robot, dwCOM0 = dw0, Wcom = Wcom, dW = dW, qd = robot.qdSymbolic, qdd = robot.qddSymbolic, symbolic = symbolic)
        
        # Inertial linear acceleration propagation to each center of mass
        dVcom = linearAccelerationPropagationCOM(robot, dvCOM0 = np.zeros((3, 1)), Wcom = Wcom, dWcom = dWcom, dV = dV, symbolic = symbolic)
        
    else:
    
        # Inertial angular velocity propagation to each reference frame 
        W = angularVelocityPropagation(robot, w0 = w0, qd = robot.jointsVelocities)
        
        # Inertial angular velocity propagation to each center of mass
        Wcom = angularVelocityPropagationCOM(robot, wCOM0 = w0, W = W, qd = robot.jointsVelocities)
        
        # Inertial angular acceleration propagation to each reference frame
        dW = angularAccelerationPropagation(robot, dw0 = dw0, W = W, qd = robot.jointsVelocities, qdd = robot.jointsAccelerations)
        
        # Inertial linear acceleration propagation to each reference frame attached to joints
        dV = linearAccelerationPropagation(robot, dv0 = dv0, W = W, dW = dW)
        
        # Inertial angular velocity propagation to each center of mass
        Wcom = angularVelocityPropagationCOM(robot, wCOM0 = w0, W = W, qd = robot.jointsVelocities)
        
        # Inertial angular acceleration propagation to each reference frame
        dWcom = angularAccelerationPropagationCOM(robot, dwCOM0 = dw0, Wcom = Wcom, dW = dW, qd = robot.jointsVelocities, qdd = robot.jointsAccelerations)
        
        # Inertial linear acceleration propagation to each center of mass
        dVcom = linearAccelerationPropagationCOM(robot, dvCOM0 = np.zeros((3, 1)), Wcom = Wcom, dWcom = dWcom, dV = dV)
    
    # Iterates through all reference frames (excepting inertial one)
    for k in range(m - 1, 0, -1):
    
        # Get Denavit - Hartenberg Parameters Matrix of current frame
        frame = robot.symbolicDHParametersCOM[k, :]
        
        # Check if this frame contains any of the "n" joints
        containedJoints = np.in1d(robot.qSymbolic, frame)
        
        # Check if current frame contains any of the centers of mass
        containedCOMs = np.in1d(robot.symbolicCOMs, frame)
        
        # If any joint is in the current reference frame and also there is a center of mass
        if any(element == True for element in containedJoints) and any(element == True for element in containedCOMs):
        
            # Get the number of the center of mass (the sum is because of the way Python indexes arrays)
            COM = np.where(containedCOMs == True)[0][-1] + 1
            
            # Rotation matrix of the current Center of Mass
            Rcom = fkCOMHTM[COM][0 : 3, 0 : 3]
            
            # Distances to center of mass from i-1 and i-th frames
            r = Rcom.T * (fkHTM[k - 1][0 : 3, -1] - fkCOMHTM[COM][0 : 3, -1]) if symbolic else Rcom.T.dot(fkHTM[k - 1][0 : 3, -1] - fkCOMHTM[COM][0 : 3, -1])
            ri = Rcom.T * (fkHTM[k][0 : 3, -1] - fkCOMHTM[COM][0 : 3, -1]) if symbolic else Rcom.T.dot(fkHTM[k][0 : 3, -1] - fkCOMHTM[COM][0 : 3, -1])
            
            # Wrench matrix
            M = Matrix([[Rcom, zeros(3, 3)], [-vectorCrossOperator(r + ri, symbolic) * Rcom, Rcom]]) if symbolic else np.append(np.append(Rcom, np.zeros((3, 3)), axis = 1), np.append(-vectorCrossOperator(r + ri).dot(Rcom), Rcom, axis = 1), axis = 0)
            
            # Acceleration matrix
            A = Matrix([[robot.symbolicMass[COM - 1] * Rcom.T, zeros(3, 3)], [-robot.symbolicMass[COM - 1] * vectorCrossOperator(r, symbolic) * Rcom.T, Rcom.T * robot.symbolicInertia[COM - 1]]]) if symbolic else np.append(np.append(robot.mass[COM - 1] * Rcom.T, np.zeros((3, 3)), axis = 1), np.append(-vectorCrossOperator(r, symbolic).dot(robot.mass[COM - 1] * Rcom.T), Rcom.T.dot(robot.inertia[COM - 1]), axis = 1), axis = 0)
            
            # Non-linear vector
            S = Matrix([[zeros(3, 1)], [(Rcom.T * Wcom[COM]).cross(Rcom.T * robot.symbolicInertia[COM - 1] * Wcom[COM])]]) if symbolic else np.append(np.zeros((3, 1)), np.cross(Rcom.T.dot(Wcom[COM]), Rcom.T.dot(robot.inertia[COM - 1]).dot(Wcom[COM]), axis = 0), axis = 0)
            
            # Wrench calculation
            Wrench[k - 1] = trigsimp((M * Wrench[k]) + (A * Matrix([[dVcom[COM]], [dWcom[COM]]])) + S) if symbolic else (M.dot(Wrench[k])) + (A.dot(np.append(dVcom[COM], dWcom[COM], axis = 0))) + S
        
        # Else, if there's no center of mass but a joint only
        elif any(element == True for element in containedJoints):
                    
            # Wrench assignation
            Wrench[k - 1] = Wrench[k]
        
    return Wrench

if __name__ == '__main__':
    
    """
        THIS SECTION IS FOR TESTING PURPOSES ONLY
    """
    
    print("Z")