function [r] = positionDQ(Q)
% This function computes the position «r», in dual for, of Dual Quaternion «Q»
    z = [zeros(4, 4) zeros(4, 4)
         zeros(4, 4) (2 * eye(4))];
    rotationDQ = [Q(1 : 4)
                  zeros(4, 1)];
    r = z * leftDualOperator(Q) * dualConjugate(rotationDQ);
end