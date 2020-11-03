function [J] = jacobianMatrixDQ(DH, q, xi)
% This function computes robot's Jacobian matrix using Dual Quaternions (see instructions inside)
%{
    This function computes robot's Jacobian matrix using the generalized
    coordinates «q» and the Denavit - Hartenberg matrix
%}
    [n, ~] = size(q);
    F = forwardKinematicsDQ(DH, n + 1);
    J = zeros(8, n);
    for j = 1 : n
        Q = forwardKinematicsDQ(DH, j);
        J(:, j) = 0.5 * leftDualOperator(Q) * rightDualOperator(F) * rightDualOperator(dualConjugate(Q)) * xi(:, j);
    end
end