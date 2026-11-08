function [q, info] = mpc_novalves(s, scn, cfg)
% MPC_NOVALVES Baseline MPC controller with a single common flow rate for all 10 loops.
% Uses fmincon (SQP algorithm) with the 4 standard initial points:
%   1. Previous solution shifted by one step (or held from s.q_prev(1))
%   2. Lower bound (qmin)
%   3. Upper bound (qTmax / Nl)
%   4. Random point within bounds (fixed seed for reproducibility)

    persistent u_last
    if isempty(u_last) || s.t == 0
        u_shift = s.q_prev(1) * ones(1, cfg.Nu);
    else
        u_shift = [u_last(2:end), u_last(end)];
    end

    Dp    = get_dni_preview(scn, s.t, cfg); % [Nl x 29 x Np]
    q_cap = cfg.qTmax / cfg.Nl;             % 0.65 l/s per loop

    % Fix random seed per time step for reproducibility
    rng(1000 + round(s.t));

    % 4 starting points [1 x Nu]
    x0 = { ...
        u_shift, ...
        cfg.qmin * ones(1, cfg.Nu), ...
        q_cap * ones(1, cfg.Nu), ...
        cfg.qmin + rand(1, cfg.Nu) * (q_cap - cfg.qmin) ...
    };

    lb   = cfg.qmin * ones(cfg.Nu, 1);
    ub   = q_cap * ones(cfg.Nu, 1);
    opts = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off');

    best  = inf;
    ubest = x0{1}(:);

    % Objective replicates candidate vector u across all 10 loops
    f = @(u) cost_field(repmat(u(:).', cfg.Nl, 1), s, Dp, cfg, 'central');

    for i = 1:numel(x0)
        [u, fv] = fmincon(f, x0{i}(:), [], [], [], [], lb, ub, [], opts);
        if fv < best
            best  = fv;
            ubest = u;
        end
    end

    u_last          = ubest(:).';
    q               = ubest(1) * ones(cfg.Nl, 1);
    info.time.local = 0;
    info.time.coord = 0;
    info.fval       = best;
end