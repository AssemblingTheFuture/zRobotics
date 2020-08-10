function [qi] = inverseKinematics(q0, L, Hd, K, m)
% This function computes robot's inverse kinematics using Homogeneous Transformation Matrices (see instructions inside)
%{
    This function computes robot's inverse kinematics using the generalized
    coordinates «q», the length of the rigid bodies, the desired pose
    represented as Homogeneous Transformation Matrix «Hd», a constant gain
    matrix «K» and the number of reference frames to be analyzed
%}
    Xd = axisAngle(Hd);
    i = 1;
    qi(:, i) = q0;
    while i <= 10000
        DH = denavitHartenberg(qi(:, i), L);
        H = forwardKinematicsDH(DH, m);
        X = axisAngle(H);
        e = Xd - X;
        if abs(e) <= 1e-3
            break
        else
            qi(:, i + 1) = solver(pinv(jacobianMatrix(DH, qi(:, i))) * K * e, qi(:, i), 3/1000);
%             qi(:, i + 1) = qi(:, i) + (pinv(jacobianMatrix(DH, qi(:, i))) * K * e);
        end
        i = i + 1;
    end
end