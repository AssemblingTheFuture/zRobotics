function [R] = Rx(a)
% This function computes a rotation «a» along «x» axis with Homogeneous
% Transformation Matrix
    R = [[1 0 0 0];
         [0 cos(a) -sin(a) 0];
         [0 sin(a) cos(a) 0];
         [0 0 0 1]];
end