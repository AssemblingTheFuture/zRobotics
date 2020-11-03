function [S] = signal(P, t, dt)
% This function computes the time - space diagram of the point(s) to be
% reached by the system
%{
    This function computes the time - space diagram of the point(s) to be
    reached by the system. «P» represents the point(s), «t» the time it
    will take to perform that action and «dt» is the time interval of the
    processor (usually, milliseconds)
%}
    [~, n] = size(P);
    S(:, 1) = P(1);
    
    % Points' iterator
    for i = 2 : n
        % Number of samples to reach the current point
        k = int32(t(i)/dt);
        
        % Slope or instant velocity
        m = (P(i) - P(i - 1)) / t(i);
        
        % Signal iterator
        [~, p] = size(S);
        for j = p + 1 : p + k
            S(:, j) = S(:, j - 1) + (m * dt);
        end
    end
end