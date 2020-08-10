function [K] = accelerationMatrixDQ(DH, q, qd)
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
        W = leftDualOperator(dualConjugate(Q)) * rightDualOperator(Q) * (leftDualOperator(Q) * rightDualOperator(dualConjugate(Q)) * [0 0 0 1 0 0 0 0]' * qd(j));
        K(:, j) = leftDualOperator(Q) * rightDualOperator(dualConjugate(Q)) * crossOperatorDQ(W) * [0 0 0 1 0 0 0 0]';
    end
end