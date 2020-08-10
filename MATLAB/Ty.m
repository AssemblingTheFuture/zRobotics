function [T] = Ty(a)
% This function computes a translation «a» along «y» axis with Homogeneous
% Transformation Matrix
    T = [[1 0 0 0];
         [0 1 0 a];
         [0 0 1 0];
         [0 0 0 1]];
end