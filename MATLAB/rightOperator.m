function [R] = rightOperator(q)
% This function computes the left operator of quaternion «q»
    R = [q(1) -transpose(q(2 : 4))
         q(2 : 4) (q(1) * eye(3) - symmetricMatrix(q))];
end