function [L] = leftDualOperator(Q)
% This function computes the left dual operator of Dual Quaternion «Q»
    L = [leftOperator(Q(1 : 4)) zeros(4, 4)
         leftOperator(Q(5 : 8)) leftOperator(Q(1 : 4))];
end