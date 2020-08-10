function [T] = Tx(a)
% This function computes a translation «a» along «x» axis with Homogeneous
% Transformation Matrix
    T = [[1 0 0 a];
         [0 1 0 0];
         [0 0 1 0];
         [0 0 0 1]];
end