function [R] = Ry(a)
% This function computes a rotation «a» along «y» axis with Homogeneous
% Transformation Matrix
    R = [[cos(a) 0 sin(a) 0];
         [0 1 0 0];
         [-sin(a) 0 cos(a) 0];
         [0 0 0 1]];
end