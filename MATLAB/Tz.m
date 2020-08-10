function [T] = Tz(a)
% This function computes a translation «a» along «z» axis with Homogeneous
% Transformation Matrix
    T = [[1 0 0 0];
         [0 1 0 0];
         [0 0 1 a];
         [0 0 0 1]];
end