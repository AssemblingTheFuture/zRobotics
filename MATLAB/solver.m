function [F] = solver(f, F, h)
% This function solves numerically the differential equation «f» using «RK4»
    [m, n] = size(f);
    for i = 1 : m
        for j = 1 : n
            k1 = f(i, j);
            k2 = f(i, j) + (0.5 * k1 * h);
            k3 = f(i, j) + (0.5 * k2 * h);
            k4 = f(i, j) + (k3 * h);
            F(i, j) = F(i, j) + ((1/6) * (k1 + (2 * k2) + (2 * k3) + k4) * h);
        end
    end
end