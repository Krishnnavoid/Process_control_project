% Quick check of the local MPC on all 10 loops at t = 0
addpath(genpath(pwd));
cfg = config();
scn = make_scn(cfg);
s   = initial_state(cfg);

Dp = get_dni_preview(scn, 0, cfg);        % [10 x 29 x 12]

fprintf('Loop | q (cap 1.5) | q (cap 0.65) | time (s)\n');
for j = 1:cfg.Nl
    ls     = coarsen_state(s, j, cfg);
    dni_j  = squeeze(Dp(j, :, :));        % [29 x 12]
    tic
    qA = local_mpc_fmincon(ls, dni_j, s.q_prev(j), s.Tin, s.Tout, cfg.Ta, cfg.qmax, cfg);
    tA = toc;
    qB = local_mpc_fmincon(ls, dni_j, s.q_prev(j), s.Tin, s.Tout, cfg.Ta, cfg.qTmax/cfg.Nl, cfg);
    fprintf('%4d | %11.3f | %12.3f | %8.2f\n', j, qA, qB, tA);
end
