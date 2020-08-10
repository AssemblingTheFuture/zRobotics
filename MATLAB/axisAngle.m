function [x] = axisAngle(H)
% This function computes the axis - angle vector «X» (see instructions inside)
%{
    This function computes the axis - angle vector «X» using the 
    Homogeneous Transformation Matrix «H» of a reference frame
%}
    theta = acos((trace(H(1 : 3, 1 : 3)) - 1)/2);
    n = (1/(2 * sin(theta))) * [H(3,2) - H(2,3)
                                H(1,3) - H(3,1)
                                H(2,1) - H(1,2)];
    x = [H(1 : 3, 4)
         theta * n];
end