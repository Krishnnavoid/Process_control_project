function [q, info] = mpc_central(s, scn, cfg)
% MPC_CENTRAL Centralized MPC baseline optimizing all 10 x Nu = 100 loop flows simultaneously.
%
% Decision vector u [100 x 1]: reshape(Q, [Nl * Nu, 1]) where Q is [10 x 10].
% Subject to:
%   - Box bounds: qmin <= q_j(n) <= qmax
%   - Linear inequality at each step n = 1...Nu: sum_{j=1}^{10} q_j(n) <= qTmax
% Solved via SQP from the 4 standard initial points.

    persistent Q_last
    Nl = cfg.Nl;
    Nu = cfg.Nu;
    nVars = Nl * Nu;

    Dp    = get_dni_preview(scn, s.t, cfg); % [Nl x 29 x Np]
    q_cap = cfg.qTmax / Nl;                 % Feasible uniform upper bound (0.65 l/s)

    % 1. Construct the 4 initial guess matrices [Nl x Nu]
    if isempty(Q_last) || s.t == 0 || size(Q_last, 2) ~= Nu
        % Ensure initial point satisfies sum_j q_j(n) <= qTmax
        q0_proj = min(s.q_prev(:), q_cap);
        Q_shift = repmat(q0_proj, 1, Nu);
    else
        Q_shift = [Q_last(:, 2:end), Q_last(:, end)];
    end

    rng(2000 + round(s.t)); % Fixed seed for reproducible random starting point
    Q_lb   = cfg.qmin * ones(Nl, Nu);
    Q_ub   = q_cap * ones(Nl, Nu);
    Q_rand = cfg.qmin + rand(Nl, Nu) * (q_cap - cfg.qmin);

    x0 = {Q_shift(:), Q_lb(:), Q_ub(:), Q_rand(:)};

    % 2. Box bounds: qmin <= q_j(n) <= qmax
    lb = cfg.qmin * ones(nVars, 1);
    ub = cfg.qmax * ones(nVars, 1);

    % 3. Linear inequality constraint: sum_{j=1}^{Nl} q_j(n) <= qTmax for each n = 1...Nu
    % Since u = reshape(Q, Nl*Nu, 1) stacks column-by-column (step-by-step),
    % A_ineq is block-diagonal with 1x10 row vectors of ones: kron(eye(Nu), ones(1, Nl))
    A_ineq = kron(eye(Nu), ones(1, Nl)); % [Nu x 100]
    b_ineq = cfg.qTmax * ones(Nu, 1);    % [Nu x 1]

    opts = optimoptions('fmincon', ...
        'Algorithm', 'sqp', ...
        'Display', 'off', ...
        'MaxFunctionEvaluations', 15000);

    f = @(u) cost_field(reshape(u, Nl, Nu), s, Dp, cfg, 'central');

    % 4. Solve from all 4 starting points (supports parfor if Parallel Toolbox is active)
    u_cand  = zeros(nVars, 4);
    fv_cand = inf(1, 4);

    parfor i = 1:4
        [u_sol, fv_sol] = fmincon(f, x0{i}, A_ineq, b_ineq, [], [], lb, ub, [], opts);
        u_cand(:, i) = u_sol;
        fv_cand(i)   = fv_sol;
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