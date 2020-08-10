function [x] = quaternionsCrossOperator(q)
% This function creates the cross product operator of a quaternion «q»
    x = [0 zeros(1, 3);
         zeros(3, 1) symmetricMatrix(q)];
end