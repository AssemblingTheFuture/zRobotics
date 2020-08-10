function [DH] = denavitHartenberg(q, L)
% This function computes the Denavit - Hartenberg matrix (see instructinons inside)
%{
    This function computes the Denavit - Hartenberg matrix based on the 
    generalized coordinates «q» and the length of the rigid bodies «L» 
%}
    DH = [0 0 0 0
          q(1) L(1) 0 pi/2
          q(2) 0 L(2) 0
          q(3) 0 0 pi/2
          q(4) L(3) 0 0];
end