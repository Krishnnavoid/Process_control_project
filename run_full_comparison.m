% RUN_FULL_COMPARISON Executes all missing cases (Local MPC and Centralized MPC)
% to completion, and runs compare_results to generate full 5-way benchmarks.
clear; clc;
proj_root = fileparts(mfilename('fullpath'));
cd(proj_root);
addpath(genpath(proj_root));

cfg = config();
scn = make_scn(cfg);
logs_dir = fullfile(proj_root, 'logs');
if ~exist(logs_dir, 'dir'), mkdir(logs_dir); end

fprintf('=================================================================\n');
fprintf('  PROCESS CONTROL BENCHMARK: FULL RUN OF ALL CASES\n');
fprintf('=================================================================\n');

%% 1. Local MPC
local_file = fullfile(logs_dir, 'log_local.mat');
need_local = true;
if exist(local_file, 'file')
    S = load(local_file);
    if isfield(S, 'log') && any(strcmpi(S.log.name, {'local', 'ctrl_local'}))
        W = S.log.W_kW(:).';
        if ~any(W <= 0)
            fprintf('\n[1/2] Local MPC is already complete in %s. Skipping.\n', local_file);
            need_local = false;
        end
    end
end

if need_local
    fprintf('\n[1/2] Running Local MPC (2 hours, 120 steps)...\n');
    log_local = run_controller_resumable(@ctrl_local, 'local', scn, cfg, logs_dir);
    log = log_local; %#ok<NASGU>
    save(local_file, 'log');
    fprintf('  Local MPC finished and saved to %s\n', local_file);
end

%% 2. Centralized MPC
central_file = fullfile(logs_dir, 'log_central.mat');
need_central = true;
if exist(central_file, 'file')
    S = load(central_file);
    if isfield(S, 'log') && strcmpi(S.log.name, 'central')
        W = S.log.W_kW(:).';
        if ~any(W <= 0)
            fprintf('\n[2/2] Centralized MPC is already complete in %s. Skipping.\n', central_file);
            need_central = false;
        end
    end
end

if need_central
    fprintf('\n[2/2] Running Centralized MPC (2 hours, 120 steps)...\n');
    log_central = run_controller_resumable(@mpc_central, 'central', scn, cfg, logs_dir);
    log = log_central; %#ok<NASGU>
    save(central_file, 'log');
    fprintf('  Centralized MPC finished and saved to %s\n', central_file);
end

%% 3. Full Comparison
fprintf('\n=================================================================\n');
fprintf('  ALL CASES COMPLETE -> Running compare_results\n');
fprintf('=================================================================\n');
compare_results('noValves');
fprintf('\nDone! Metrics and figures updated.\n');


% =========================================================================
function log = run_controller_resumable(ctrl_fn, name, scn, cfg, logs_dir)
    Tend = 7200;
    K    = Tend / cfg.Tc;   % 120 steps
    nsub = cfg.Tc / cfg.dT; % 120 sub-steps

    ckpt_file = fullfile(logs_dir, sprintf('checkpoint_%s.mat', name));
    s = initial_state(cfg);
    k_start = 1;

    % Check if a valid checkpoint can be resumed
    if exist(ckpt_file, 'file')
        C = load(ckpt_file);
        if isfield(C, 'log') && isfield(C, 'k') && C.k >= 1 && C.k < K ...
           && any(strcmpi(C.log.name, {name, ['ctrl_' name]}))
            k_done = C.k;
            log = C.log;
            fprintf('  Found valid checkpoint for "%s" at step %d / %d.\n', name, k_done, K);
            fprintf('  Replaying plant forward to step %d (t = %d min)...\n', k_done, round(k_done * cfg.Tc / 60));
            t_rep = tic;
            for k = 1:k_done
                s.t = (k - 1) * cfg.Tc;
                q = log.q(:, k);
                for m = 1:nsub
                    D = get_dni(scn, s.t + (m - 1) * cfg.dT, cfg);
                    [s, ~] = plant_step(s, q * 1e-3, D, cfg);
                end
                s.q_prev = q;
            end
            fprintf('  Plant state replayed in %.2f s. Resuming execution from step %d to %d...\n', toc(t_rep), k_done + 1, K);
            k_start = k_done + 1;
        else
            log = init_log(name, cfg, K);
        end
    else
        log = init_log(name, cfg, K);
    end

    for k = k_start:K
        s.t = (k - 1) * cfg.Tc;
        t_step = tic;
        [q, info] = ctrl_fn(s, scn, cfg);
        tc = toc(t_step);

        q = min(max(q(:), cfg.qmin), cfg.qmax);

        Wacc = 0;
        for m = 1:nsub
            D = get_dni(scn, s.t + (m - 1) * cfg.dT, cfg);
            [s, W] = plant_step(s, q * 1e-3, D, cfg);
            Wacc = Wacc + W;
        end
        s.q_prev = q;

        log.q(:, k)         = q;
        log.Tout_loop(:, k) = s.Tf(:, end);
        log.Tout_field(k)   = s.Tout;
        log.Tin(k)          = s.Tin;
        log.W_kW(k)         = Wacc / nsub;
        log.time.total(k)   = tc;

        if isstruct(info) && isfield(info, 'time')
            if isfield(info.time, 'local'), log.time.local(k) = info.time.local; end
            if isfield(info.time, 'coord'), log.time.coord(k) = info.time.coord; end
        end

        % Save checkpoint every step so crashes or interruptions lose zero work
        save(ckpt_file, 'log', 'k');

        fprintf('  [%s] step %3d / %d (t=%3d min): W = %7.1f kW, Tout = %5.1f C, time = %5.1f s\n', ...
                name, k, K, round(s.t / 60), log.W_kW(k), log.Tout_field(k), tc);
    end
end

function log = init_log(name, cfg, K)
    log.name       = name;
    log.scenario   = 'frejo2h';
    log.t          = 1:K;
    log.q          = zeros(cfg.Nl, K);
    log.Tout_loop  = zeros(cfg.Nl, K);
    log.Tout_field = zeros(1, K);
    log.Tin        = zeros(1, K);
    log.W_kW       = zeros(1, K);
    log.time.total = zeros(1, K);
    log.time.local = zeros(1, K);
    log.time.coord = zeros(1, K);
end

