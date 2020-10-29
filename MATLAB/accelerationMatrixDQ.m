function [K] = accelerationMatrixDQ(DH, q, qd, xi)
% This function computes the inertial acceleration matrix «K» (see instructions inside)
%{
    This function computes the inertial acceleration matrix «K» using 
    Denavit - Hartenberg matrix «DH», generalized coordinates «q» and its 
    time derivative «qd»
%}
    [n, ~] = size(q);
    K = zeros(8, n);
    for j = 1 : n
        Q = forwardKinematicsDQ(DH, j);
        W = leftDualOperator(dualConjugate(Q)) * rightDualOperator(Q) * (leftDualOperator(Q) * rightDualOperator(dualConjugate(Q)) * xi(:, j) * qd(j));
        K(:, j) = leftDualOperator(Q) * rightDualOperator(dualConjugate(Q)) * crossOperatorDQ(W) * xi(:, j);
    end
end