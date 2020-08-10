function [Q] = dqTx(a)
%% This function computes a translation «a» along «x» axis in Dual Quaternion form
    Q = transpose([1 0 0 0 0 a/2 0 0]);
end