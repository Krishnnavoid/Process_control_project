% RUN_PART1_ALL Executes all Part 1 verification tests and generates baseline logs.
clear; clc;

% Automatically lock Current Folder to the root 'project/' directory
proj_root = fileparts(mfilename('fullpath'));
cd(proj_root);
addpath(genpath(proj_root));

% Ensure parallel pool is active, has no idle timeout, and shares all project paths
if license('test', 'Distrib_Computing_Toolbox')
    pool = gcp('nocreate');
    if isempty(pool)
        pool = parpool('Processes');
    end
    pool.IdleTimeout = 240; % Prevent pool from shutting down during long runs
    pctRunOnAll(sprintf('addpath(genpath(''%s''));', proj_root));
end

cfg = config();
scn = make_scn(cfg);
logs_dir = fullfile(proj_root, 'logs');
if ~exist(logs_dir, 'dir'), mkdir(logs_dir); end

%% 1. Run PI Tuning, Simulink Build, Setpoint Sweep & Export logs/log_PI.mat
fprintf('--- Step 1: Tuning PI & Running Simulink Baseline ---\n');
[Kp, Ki, sweep_tbl] = pi_tune();

%% 2. Run No-Valves MPC Baseline & Export logs/log_noValves.mat
fprintf('--- Step 2: Running No-Valves MPC Baseline (2h scenario) ---\n');
log = run_closed_loop(@mpc_novalves, 'noValves', scn, cfg, 7200);
save(fullfile(logs_dir, 'log_noValves.mat'), 'log');
fprintf('No-Valves Mean Thermal Power: %.2f kW\n', mean(log.W_kW));

%% 3. Run Centralized MPC Baseline & Export logs/log_central.mat
fprintf('--- Step 3: Running Centralized MPC Baseline (2h scenario) ---\n');
log = run_closed_loop(@mpc_central, 'central', scn, cfg, 7200);
save(fullfile(logs_dir, 'log_central.mat'), 'log');
fprintf('Centralized MPC Mean Thermal Power: %.2f kW\n', mean(log.W_kW));
