function [L] = leftOperator(q)
% This function computes the left operator of quaternion «q»
    L = [q(1) -transpose(q(2 : 4))
         q(2 : 4) (q(1) * eye(3) + symmetricMatrix(q))];
end