function [R] = rightDualOperator(q)
% This function computes the left dual operator of Dual Quaternion «Q»
    R = [rightOperator(q(1 : 4)) zeros(4, 4)
         rightOperator(q(5 : 8)) rightOperator(q(1 : 4))];
end