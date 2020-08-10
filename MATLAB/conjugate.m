function [q] = conjugate(q)
%% This function computes the conjugate form of a quaternion «q»
    q = transpose([q(1) -q(2) -q(3) -q(4)]);
end