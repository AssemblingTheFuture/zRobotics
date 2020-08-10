function [Q] = dqTz(a)
%% This function computes a translation «a» along «z» axis in Dual Quaternion form
    Q = transpose([1 0 0 0 0 0 0 a/2]);
end