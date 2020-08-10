function [R] = Rz(a)
% This function computes a rotation «a» along «z» axis with Homogeneous
% Transformation Matrix
    R = [[cos(a) -sin(a) 0 0];
         [sin(a) cos(a) 0 0];
         [0 0 1 0];
         [0 0 0 1]];
end