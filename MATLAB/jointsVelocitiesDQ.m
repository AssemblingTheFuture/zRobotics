function [qd] = jointsVelocitiesDQ(q, W, DH, m)
% This function computes the joints' velocities (see instructions inside)
%{
    This function computes the joints' velocities using the generalized
    coordinates «q»; it also needs the velocity of the end - effector in
    dual form «A». Also, it needs the Denavit - Hartenberg matrix of the 
    system and the number of reference frames to be analyzed
%}
    Q = forwardKinematicsDQ(DH, m);
    r = positionDQ(Q);
    s = quaternionsCrossOperator(r(5 : 8));
    J = velocityJacobianMatrixDQ(DH, q);
    M = [eye(4) zeros(4)
         s eye(4)];
    qd = pinv(J) * M * W;
end