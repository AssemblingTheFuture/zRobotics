function [Q] = dualConjugate(Q)
%% This function computes the conjugate form of a Dual Quaternion «Q»
    Q = [conjugate(Q(1 : 4));
         conjugate(Q(5 : 8))];
end
