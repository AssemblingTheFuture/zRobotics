function [qdd] = jointsAccelerationsDQ(q, qd, Wd, DH, m)
% This function computes the joints' accelerations (see instructions inside)
%{
    This function computes the joints' accelerations using the generalized
    coordinates «q» and its respective time derivative «qd»; it also needs
    the acceleration of the end - effector in dual form «Wd». Also, it needs
    the Denavit - Hartenberg matrix of the system and the number of reference
    frames to be analyzed
%}
    Q = forwardKinematicsDQ(DH, m);
    r = positionDQ(Q);
    s = quaternionsCrossOperator(r(5 : 8));
    M = [eye(4) zeros(4)
         s eye(4)];
    J = velocityJacobianMatrixDQ(DH, q);
    K = accelerationMatrixDQ(DH, q, qd);
    W = endEffectorVelocityDQ(q, qd, DH, m);
    w = W(1 : 4);
    x = [zeros(4, 1)
         (quaternionsCrossOperator(w)) * (quaternionsCrossOperator(w) * r(5 : 8))];
    qdd = pinv(J) * ((M * Wd) - x - (K * qd));
end