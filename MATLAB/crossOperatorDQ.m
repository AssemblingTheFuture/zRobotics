function [x] = crossOperatorDQ(Q)
%% This function creates the cross product operator of a Dual Quaternion «Q»
    x = [quaternionsCrossOperator(Q(1 : 4)) zeros(4, 4);
         quaternionsCrossOperator(Q(5 : 8)) quaternionsCrossOperator(Q(1 : 4))];
end