% RUN_PART1_ALL Executes all Part 1 verification tests and generates baseline logs.
clear; clc;
addpath(genpath(pwd));

cfg = config();
scn = make_scn(cfg);
if ~exist('logs', 'dir'), mkdir('logs'); end

%% 1. Run PI Tuning, Simulink Build, Setpoint Sweep & Export logs/log_PI.mat
fprintf('--- Step 1: Tuning PI & Running Simulink Baseline ---\n');
[Kp, Ki, sweep_tbl] = pi_tune();

%% 2. Run No-Valves MPC Baseline & Export logs/log_noValves.mat
fprintf('--- Step 2: Running No-Valves MPC Baseline (2h scenario) ---\n');
log = run_closed_loop(@mpc_novalves, 'noValves', scn, cfg, 7200);
save(fullfile('logs', 'log_noValves.mat'), 'log');
fprintf('No-Valves Mean Thermal Power: %.2f kW\n', mean(log.W_kW));

%% 3. Run Centralized MPC Baseline & Export logs/log_central.mat
fprintf('--- Step 3: Running Centralized MPC Baseline (2h scenario) ---\n');
log = run_closed_loop(@mpc_central, 'central', scn, cfg, 7200);
save(fullfile('logs', 'log_central.mat'), 'log');
fprintf('Centralized MPC Mean Thermal Power: %.2f kW\n', mean(log.W_kW));