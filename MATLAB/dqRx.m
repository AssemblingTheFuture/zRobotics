function [Q] = dqRx(a)
%% This function computes a rotation «a» along «x» axis in Dual Quaternion form
    Q = transpose([cos(a/2) sin(a/2) 0 0 0 0 0 0]);
end