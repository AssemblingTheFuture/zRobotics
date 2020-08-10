function [Wd] = endEffectorAccelerationDQ(q, qd, qdd, DH, m)
% This function computes the end - effector acceleration (see instructions inside)
%{
    This function computes the end - effector acceleration using the generalized
    coordinates «q» and its respective time derivatives «qd» and «qdd»; it
    also needs the Denavit - Hartenberg matrix of the system and the number
    of reference frames to be analyzed
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
	g = [0 0 0 0 0 0 0 -9.80665]';
    Wd = M \ ((J * qdd) + (K * qd) + x + g);
end