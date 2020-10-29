function [J] = velocityJacobianMatrixDQ(DH, q, xi)
% This function computes robot's Velocity Jacobian matrix using Dual Quaternions (see instructions inside)
%{
    This function computes robot's Velocity Jacobian matrix using the
    generalized coordinates «q» and the Denavit - Hartenberg matrix
%}
    [n, ~] = size(q);
    % J = zeros(8, n);
    for j = 1 : n
        Q = forwardKinematicsDQ(DH, j);
        J(:, j) = leftDualOperator(Q) * rightDualOperator(dualConjugate(Q)) * xi(:, j);
    end
end