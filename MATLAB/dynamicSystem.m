function [e, q, u, V] = dynamicSystem(q0, qd, K, t)
% This function computes the robot's dynamic system as it would be unkown
    [m, ~] = size(q0);
    q = zeros(m, fix(t/0.003) + 1);
    e = zeros(m, fix(t/0.003) + 1);
    u = zeros(m, fix(t/0.003) + 1);
    V = zeros(m, fix(t/0.003) + 1);
    q(:, 1) = q0;
    e(:, 1) = q0 - qd;
    V(:, 1) = solver(K * e(:, 1), V(:, 1), 3/1000);
    u(:, 1) = - (q0 + V(:, 1) + (K * e(:, 1)));
    for i = 1 : fix(t / 0.003)
        q(:, i + 1) = solver((randn(m, m) * q(:, i)) + sin(q(:, i)) + u(:, i), q(:, i), 3/1000);
        e(:, i + 1) = q(:, i + 1) - qd;
        V(:, i + 1) = solver(K * e(:, i + 1), V(:, i), 3/1000);
        u(:, i + 1) = - (q(:, i + 1) + V(:, i + 1) + (K * e(:, i + 1)));
    end
end