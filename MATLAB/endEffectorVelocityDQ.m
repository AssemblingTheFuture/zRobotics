function [W] = endEffectorVelocityDQ(q, qd, DH, m)
% This function computes the end - effector velocity (see instructions inside)
%{
    This function computes the end - effector velocity using the generalized
    coordinates «q» and its respective time derivative «qd»; it also needs 
    the Denavit - Hartenberg matrix of the system and the number of reference 
    frames to be analyzed
%}
    Q = forwardKinematicsDQ(DH, m);
    r = positionDQ(Q);
    s = quaternionsCrossOperator(r(5 : 8));
    J = velocityJacobianMatrixDQ(DH, q);
    M = [eye(4) zeros(4)
         s eye(4)];
    W = (M \ J) * qd;
end