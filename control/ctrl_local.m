function [q, info] = ctrl_local(s, scn, cfg)
% Local MPC with per-loop cap qTmax/Nl (paper's "Local MPC" baseline)
    Dp = get_dni_preview(scn, s.t, cfg);
    q  = zeros(cfg.Nl, 1);
    tl = tic;
    for j = 1:cfg.Nl
        ls = coarsen_state(s, j, cfg);
        q(j) = local_mpc_fmincon(ls, squeeze(Dp(j,:,:)), s.q_prev(j), ...
                   s.Tin, s.Tout, cfg.Ta, cfg.qTmax/cfg.Nl, cfg);
    end
    info.time.local = toc(tl);
    info.time.coord = 0;
end
