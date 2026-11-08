% TEST_COORDINATOR  Hand-made two-loop unit test of the coordinator.
% Uses mock local solvers (no fmincon) so the logic is tested on its own.
% Run from the repo root:  test_coordinator
addpath(genpath(pwd));
cfg = config();  scn = make_scn(cfg);  s = initial_state(cfg);

% ---- two-loop mini field: loop 1 sunny (eff 1.0), loop 2 weak (eff 0.15) ----
cfg.Nl = 2;
scn.eff = ones(2, cfg.Ns);  scn.eff(2, :) = 0.15;  scn.eff(:, cfg.passive) = 0;
scn.cloud.t0 = 999;  scn.cloud.t1 = 998;                  % no cloud
s.Tf = s.Tf(1:2, :);  s.Tm = s.Tm(1:2, :);  s.T1f = s.T1f(1:2, :);
s.q_prev = s.q_prev(1:2);

tol = 1e-6;

% ---- Mock solvers: return [q, qseq], respecting the cap ----
want  = @(dni) max(0.2, mean(dni(:))/1000);               % sunny ~0.66, weak -> 0.2
mockA = @(ls,dni,qp,Tin,Tout,Ta,cap,c) deal(min(want(dni),cap), min(want(dni),cap)*ones(1,c.Nu));
mockB = @(ls,dni,qp,Tin,Tout,Ta,cap,c) deal(min(0.6,cap), min(0.6,cap)*ones(1,c.Nu));

%% Test 1: Step 3 pass-through (sum of Step-2 flows already feasible)
cfg.qTmax = 1.0;
[q, info] = coordinator(s, scn, cfg, mockA);
assert(strcmp(info.exit, 'step3_feasible'), 'Test 1: expected exit at Step 3');
assert(abs(sum(q) - sum(info.q_step2)) < tol, 'Test 1: flows should be unchanged');
fprintf('Test 1 OK  (Step 3)           q = [%.3f %.3f]\n', q);

%% Test 2: cap applies, spare margin (Steps 7-8): sum(Step 4) < qTmax
cfg.qTmax = 0.7;                                          % cap = 0.35 per loop
[q, info] = coordinator(s, scn, cfg, mockA);
assert(sum(q) <= cfg.qTmax + tol,               'Test 2: total flow above qTmax');
assert(all(q >= cfg.qmin - tol) && all(q <= 1.2 + tol), 'Test 2: flow out of bounds');
assert(q(1) >= info.q_step4(1) - tol,           'Test 2: saturated loop should not decrease');
fprintf('Test 2 OK  (Steps 7-8)        q = [%.3f %.3f]  total = %.3f  exit = %s\n', ...
        q, sum(q), info.exit);

%% Test 3: total at maximum (Steps 9-10): both loops saturated at the cap
cfg.qTmax = 0.7;
[q, info] = coordinator(s, scn, cfg, mockB);
assert(sum(q) <= cfg.qTmax + tol,               'Test 3: total flow above qTmax');
assert(all(q >= cfg.qmin - tol) && all(q <= 1.2 + tol), 'Test 3: flow out of bounds');
fprintf('Test 3 OK  (Steps 9-10)       q = [%.3f %.3f]  total = %.3f  exit = %s\n', ...
        q, sum(q), info.exit);
fprintf('          flow moved to the sunny loop? %d\n', q(1) > q(2));

%% Test 4: real fmincon local MPC on the same two loops (prints, no assertion)
cfg.qTmax = 0.7;
[q, info] = coordinator(s, scn, cfg, @local_mpc_fmincon);
assert(sum(q) <= cfg.qTmax + tol, 'Test 4: total flow above qTmax');
fprintf('Test 4 OK  (fmincon)          q = [%.3f %.3f]  total = %.3f  exit = %s\n', ...
        q, sum(q), info.exit);
fprintf('          time: local %.2f s, coordinator %.2f s\n', info.time.local, info.time.coord);

fprintf('\nAll coordinator tests passed.\n');
