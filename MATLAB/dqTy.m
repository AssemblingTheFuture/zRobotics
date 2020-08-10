function [Q] = dqTy(a)
%% This function computes a translation «a» along «y» axis in Dual Quaternion form
    Q = transpose([1 0 0 0 0 0 a/2 0]);
end