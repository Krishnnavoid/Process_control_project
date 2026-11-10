function [q, info] = mpc_central(s, scn, cfg)
% MPC_CENTRAL Centralized MPC baseline optimizing all 10 x Nu = 100 loop flows simultaneously.
%
% Decision vector u [100 x 1]: reshape(Q, [Nl * Nu, 1]) where Q is [10 x 10].
% Subject to:
%   - Box bounds: qmin <= q_j(n) <= qmax
%   - Linear inequality at each step n = 1...Nu: sum_{j=1}^{10} q_j(n) <= qTmax
% Solved via SQP from the 4 standard initial points.

    persistent Q_last
    Nl    = cfg.Nl;
    Nu    = cfg.Nu;
    nVars = Nl * Nu;

    Dp    = get_dni_preview(scn, s.t, cfg); % [Nl x 29 x Np]
    q_cap = cfg.qTmax / Nl;                 % 0.65 l/s per-loop cap

    % DNI-proportional feasible loop allocation strictly inside bounds and sum(q) < qTmax
    loop_dni = mean(Dp(:, :, 1), 2); % [Nl x 1]
    q_prop   = (cfg.qTmax - 0.02) * (loop_dni / sum(loop_dni));
    q_prop   = min(max(q_prop, cfg.qmin + 0.02), cfg.qmax - 0.05);
    if sum(q_prop) >= cfg.qTmax
        q_prop = q_prop * ((cfg.qTmax - 0.02) / sum(q_prop));
    end

    % 1. Construct the 4 initial guess matrices [Nl x Nu] strictly inside feasible interior
    if isempty(Q_last) || s.t == 0 || size(Q_last, 2) ~= Nu
        Q_shift = repmat(q_prop, 1, Nu);
    else
        Q_shift = [Q_last(:, 2:end), Q_last(:, end)];
        % Ensure shifted warm-start is strictly inside bounds and linear constraint
        Q_shift = min(max(Q_shift, cfg.qmin + 1e-3), cfg.qmax - 1e-3);
        col_sums = sum(Q_shift, 1);
        over_idx = col_sums >= cfg.qTmax;
        if any(over_idx)
            Q_shift(:, over_idx) = Q_shift(:, over_idx) .* ((cfg.qTmax - 1e-2) ./ col_sums(over_idx));
        end
    end

    rng(2000 + round(s.t)); % Fixed seed for reproducible random starting point
    Q_lb   = (cfg.qmin + 0.01) * ones(Nl, Nu);
    Q_ub   = (q_cap - 0.005) * ones(Nl, Nu);
    Q_rand = (cfg.qmin + 0.01) + rand(Nl, Nu) * (q_cap - cfg.qmin - 0.02);

    x0 = {Q_shift(:), Q_lb(:), Q_ub(:), Q_rand(:)};

    % 2. Box bounds: qmin <= q_j(n) <= qmax
    lb = cfg.qmin * ones(nVars, 1);
    ub = cfg.qmax * ones(nVars, 1);

    % 3. Linear inequality constraint: sum_{j=1}^{Nl} q_j(n) <= qTmax for each n = 1...Nu
    A_ineq = kron(eye(Nu), ones(1, Nl)); % [Nu x 100]
    b_ineq = cfg.qTmax * ones(Nu, 1);    % [Nu x 1]

    opts = optimoptions('fmincon', ...
        'Algorithm', 'sqp', ...
        'HonorBounds', true, ...
        'Display', 'off', ...
        'MaxIterations', 250, ...
        'MaxFunctionEvaluations', 25000, ...
        'StepTolerance', 1e-5, ...
        'OptimalityTolerance', 1e-4);

    f = @(u) cost_field(reshape(u, Nl, Nu), s, Dp, cfg, 'central');

    % 4. Solve from all 4 starting points in parallel
    u_cand  = zeros(nVars, 4);
    fv_cand = inf(1, 4);

    parfor i = 1:4
        try
            [u_sol, fv_sol] = fmincon(f, x0{i}, A_ineq, b_ineq, [], [], lb, ub, [], opts);
            u_cand(:, i) = u_sol;
            fv_cand(i)   = fv_sol;
        catch
            u_cand(:, i) = x0{i};
            fv_cand(i)   = f(x0{i});
        end
    end

    [best, best_idx] = min(fv_cand);
    ubest            = u_cand(:, best_idx);
    Q_best           = reshape(ubest, Nl, Nu);

    Q_last          = Q_best;
    q               = Q_best(:, 1);
    info.time.local = 0;
    info.time.coord = 0;
    info.fval       = best;
end
