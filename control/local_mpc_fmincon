function [q, qseq] = local_mpc_fmincon(ls, dni_prev, q_prev, Tin, Tout, Ta, qmax_eff, cfg)
% First-step flow of the local MPC for one loop, with flow cap qmax_eff.
    Nu = cfg.Nu;
    lb = cfg.qmin * ones(Nu, 1);
    ub = qmax_eff * ones(Nu, 1);

    rs = RandStream('mt19937ar', 'Seed', 1);        % fixed seed, global rng untouched
    q0 = min(max(q_prev, cfg.qmin), qmax_eff);
    x0 = { q0*ones(Nu,1), lb, ub, lb + rand(rs, Nu, 1).*(ub - lb) };

    opts = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off');
    f    = @(u) cost_local(u(:).', ls, dni_prev, q_prev, Tin, Tout, Ta, cfg);

    best  = inf;  ubest = x0{1};
    for i = 1:4
        [u, fv] = fmincon(f, x0{i}, [], [], [], [], lb, ub, [], opts);
        if fv < best, best = fv; ubest = u; end
    end

    qseq = ubest(:).';     % whole planned sequence (l/s)
    q    = qseq(1);        % first move is what gets applied
end
