function [Q] = forwardKinematicsDQ(DH, m)
% This function computes robot's forward kinematics using Dual Quaternions (see instructions inside)
%{
    This function computes robot's forward kinematics using the 
    Denavit - Hartenberg matrix of the system and the number of reference 
    frames to be analyzed
%}
    Q = transpose([1 0 0 0 0 0 0 0]);
    for i = 1 : m
        Q = leftDualOperator(Q) * (rightDualOperator(dqRx(DH(i, 4))) * leftDualOperator(dqRz(DH(i, 1))) * rightDualOperator(dqTx(DH(i, 3))) * dqTz(DH(i, 2)));
    end
end