function J = cost_local(q_seq, ls, dni_prev, q_prev, Tin, Tout_hold, Ta, cfg)
% Cost of one loop over the horizon (same units as baselines/cost_field.m)
    Np = size(dni_prev, 2);
    Nu = numel(q_seq);

    [Tout, W] = simulate_loop_ctrl(ls, q_seq, dni_prev, Tin, Tout_hold, Ta, cfg);

    q_ext = [q_seq(:).', q_seq(end) * ones(1, Np - Nu)];   % hold last move
    dq    = q_ext - [q_prev, q_ext(1:end-1)];              % flow changes

    viol  = max(max((Tout - cfg.Tmax)/cfg.Tmax, (cfg.Tmin - Tout)/cfg.Tmax), 0);

    J = sum(-W/1e3 + cfg.psi_l * viol.^2 + cfg.eps_l * dq.^2);
end
