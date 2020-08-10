function [qi] = inverseKinematicsDQ(q0, L, Qd, K, m)
% This function computes robot's inverse kinematics using Dual Quaternions (see instructions inside)
%{
    This function computes robot's inverse kinematics using the generalized
    coordinates «q», the length of the rigid bodies, the desired pose
    represented as Dual Quaternion «Qd», a constant gain matrix «K» and the
    number of reference frames to be analyzed
%}
    i = 1;
    qi(:, i) = q0;
    while i <= 10000
        DH = denavitHartenberg(qi(:, i), L);
        Q = forwardKinematicsDQ(DH, m);
        e = Qd - Q;
        if (abs(e) <= 1e-3)
            break
        else
            qi(:, i + 1) = solver(pinv(jacobianMatrixDQ(DH, qi(:, i))) * K * e, qi(:, i), 3/1000);
%             qi(:, i + 1) = qi(:, i) + (pinv(jacobianMatrixDQ(DH, qi(:, i))) * K * e);
        end
        i = i + 1;
    end
end