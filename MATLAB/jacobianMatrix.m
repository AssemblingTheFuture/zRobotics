function [J] = jacobianMatrix(DH, q)
% This function computes robot's Jacobian matrix using Homogeneous Transformation Matrices (see instructions inside)
%{
    This function computes robot's Jacobian matrix using the generalized
    coordinates «q» and the Denavit - Hartenberg matrix
%}
    [n, ~] = size(q);
    F = forwardKinematicsDH(DH, n + 1);
    J = zeros(6, n);
    for j = 1 : n
        H = forwardKinematicsDH(DH, j);
        z = H(1 : 3, 3);
        r = F(1 : 3, 4) - H(1 : 3, 4);
        J(1 : 3, j) = cross(z, r);
        J(4 : 6, j) = z;
    end
end