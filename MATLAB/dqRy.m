function [Q] = dqRy(a)
%% This function computes a rotation «a» along «y» axis in Dual Quaternion form
    Q = transpose([cos(a/2) 0 sin(a/2) 0 0 0 0 0]);
end