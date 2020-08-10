function [H] = forwardKinematicsDH(DH, m)
% This function computes robot's forward kinematics using Homogeneous Transformation Matrices (see instructions inside)
%{
    This function computes robot's forward kinematics using the 
    Denavit - Hartenberg matrix of the system and the number of reference 
    frames to be analyzed
%}
    H = eye(4);
    for i = 1 : m
        H = H * (Rz(DH(i, 1)) * Tz(DH(i, 2)) * Tx(DH(i, 3)) * Rx(DH(i, 4)));
    end
end