function [Kp, Ki, sweep_tbl] = pi_tune()
% PI_TUNE Tunes the PI controller using Smith's two-point FOPDT method and IMC rules,
% then runs the setpoint sweep Tref in {220, 240, 260, 280} degC and exports logs/log_PI.mat.

    % Lock Current Folder to root 'project/' directory regardless of where user calls pi_tune
    base_dir  = fileparts(mfilename('fullpath'));
    proj_root = fileparts(base_dir);
    cd(proj_root);
    addpath(genpath(proj_root));

    cfg  = config();
    s    = initial_state(cfg);
    Tin0 = s.Tin; % Hold Tin = 155 degC fixed during open-loop collector step test
    D    = 800 * ones(cfg.Nl, cfg.Ns);
    D(:, cfg.passive) = 0;

    %% 1. Settle Plant for 40 min at q0 = 0.50 l/s, then Step +10% to 0.55 l/s for 40 min
    N_settle = round(40 * 60 / cfg.dT);
    N_step   = round(40 * 60 / cfg.dT);
    q0       = 0.50e-3 * ones(cfg.Nl, 1);
    q1       = 0.55e-3 * ones(cfg.Nl, 1);

    for k = 1:N_settle
        [s, ~] = plant_step(s, q0, D, cfg);
        s.Tin  = Tin0;
    end
    T_initial = s.Tout;

    Tout_step = zeros(1, N_step);
    t_step    = (1:N_step) * cfg.dT;
    for k = 1:N_step
        [s, ~]       = plant_step(s, q1, D, cfg);
        s.Tin        = Tin0;
        Tout_step(k) = s.Tout;
    end

    %% 2. Fit First-Order Plus Dead-Time (FOPDT) Model via Smith's Two-Point Method
    dT_out = Tout_step(end) - T_initial;
    dq_lps = 0.05;
    K_proc = dT_out / dq_lps;

    y_norm = (Tout_step - T_initial) / dT_out;
    t28    = t_step(find(y_norm >= 0.283, 1, 'first'));
    t63    = t_step(find(y_norm >= 0.632, 1, 'first'));

    tau   = 1.5 * (t63 - t28);
    theta = max(0, t63 - tau);

    %% 3. IMC-PI Tuning Rules (Step 8)
    lambda = max(theta, 0.2 * tau);
    Kp     = tau / (abs(K_proc) * (lambda + theta));
    Ti     = tau;
    Ki     = Kp / Ti;

    fprintf('FOPDT Fit: K = %.2f degC/(l/s), tau = %.1f s, theta = %.1f s\n', K_proc, tau, theta);
    fprintf('IMC-PI Gains: Kp = %.5f l/s/degC, Ki = %.6f l/s/(degC*s)\n', Kp, Ki);

    %% 4. Setpoint Sweep {220, 240, 260, 280} degC in Simulink & Export Best Log
    Trefs     = [220, 240, 260, 280];
    mean_W_kW = zeros(size(Trefs));
    best_W    = -inf;
    best_log  = struct();

    % Build the Simulink model once and load into memory via absolute path
    build_baseline_pi(Kp, Ki, Trefs(1));
    fprintf('Successfully generated simulink/baseline_pi.slx\n');
    mdl     = 'baseline_pi';
    slx_pth = fullfile(proj_root, 'simulink', 'baseline_pi.slx');
    load_system(slx_pth);

    for idx = 1:numel(Trefs)
        Tref_val = Trefs(idx);
        set_param([mdl '/Tref'], 'Value', num2str(Tref_val));
        save_system(mdl, slx_pth);

        out = sim(mdl);

        W_sim   = reshape(out.o_W.signals.values, 1, []);
        Tf_sim  = reshape(out.o_Tout.signals.values, 1, []);
        Tin_sim = reshape(out.o_Tin.signals.values, 1, []);

        q_raw  = squeeze(out.o_q.signals.values);
        if size(q_raw, 1) ~= cfg.Nl, q_raw = q_raw.'; end
        Tl_raw = squeeze(out.o_ToutLoop.signals.values);
        if size(Tl_raw, 1) ~= cfg.Nl, Tl_raw = Tl_raw.'; end

        % Format into 120 controller steps (1 minute each = 120 steps of 0.5 s)
        K_steps  = 120;
        nsub     = 120;
        sample_k = (1:K_steps) * nsub;

        log_cand.name       = 'PI';
        log_cand.scenario   = 'frejo2h';
        log_cand.t          = 1:K_steps;
        log_cand.q          = q_raw(:, sample_k);
        log_cand.Tout_loop  = Tl_raw(:, sample_k);
        log_cand.Tout_field = Tf_sim(sample_k);
        log_cand.Tin        = Tin_sim(sample_k);
        log_cand.W_kW       = mean(reshape(W_sim(1:K_steps*nsub), nsub, K_steps), 1);
        log_cand.time.total = zeros(1, K_steps);
        log_cand.time.local = zeros(1, K_steps);
        log_cand.time.coord = zeros(1, K_steps);

        mean_W_kW(idx) = mean(log_cand.W_kW);
        fprintf('  Setpoint Tref = %d degC -> Mean Power = %.2f kW\n', Tref_val, mean_W_kW(idx));

        if mean_W_kW(idx) > best_W
            best_W   = mean_W_kW(idx);
            best_log = log_cand;
        end
    end

    close_system(mdl, 0);

    sweep_tbl = table(Trefs(:), mean_W_kW(:), 'VariableNames', {'Tref_degC', 'MeanPower_kW'});
    disp(sweep_tbl);

    logs_dir = fullfile(proj_root, 'logs');
    if ~exist(logs_dir, 'dir'), mkdir(logs_dir); end
    log = best_log; %#ok<NASGU>
    save(fullfile(logs_dir, 'log_PI.mat'), 'log', 'sweep_tbl');
end
