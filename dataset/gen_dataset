function gen_dataset(nDays, seed, nSteps, firstDay)
% GEN_DATASET  Closed-loop data generation for the neural-network local MPC.
%
%   gen_dataset(nDays, seed, nSteps)
%   gen_dataset(nDays, seed, nSteps, firstDay)
%
% For each randomised training day (make_training_days, never the 2 h Frejo case):
%   - the plant is driven in closed loop by the distributed MPC (dmpc_fmincon), so the
%     visited states are realistic;
%   - at every controller step, for every loop, the local MPC (fmincon) is solved with a
%     RANDOM flow cap and the first move is stored as the label Y (and the whole planned
%     sequence as Yseq);
%   - the 411-column feature vector is built with build_features (Part 3) if it exists,
%     otherwise with build_features_stub (same layout).
%
% Each day is saved to data/day_XXX.mat as soon as it finishes, so an overnight run can be
% stopped and resumed (days that already exist are skipped). Then run assemble_dataset.
%
% Pilot (about 4 min):          gen_dataset(1, 123, 20)
% Real run (days run in parallel if the Parallel Computing Toolbox is available):
%                               gen_dataset(25, 123, 120)

    if nargin < 3 || isempty(nSteps),   nSteps   = 120; end
    if nargin < 4 || isempty(firstDay), firstDay = 1;   end

    root   = fileparts(which('config'));
    outDir = fullfile(root, 'data');
    if ~exist(outDir, 'dir'), mkdir(outDir); end
    addpath(genpath(root));

    cfg  = config();
    scns = make_training_days(nDays, seed, cfg);       % randomised days only (held-out case excluded)

    % Start a parallel pool if possible; otherwise parfor simply runs serially
    try
        if license('test', 'Distrib_Computing_Toolbox') && isempty(gcp('nocreate'))
            parpool;
        end
        if ~isempty(gcp('nocreate'))
            pctRunOnAll(sprintf('addpath(genpath(''%s''))', root));
        end
    catch ME
        warning('gen_dataset:pool', 'No parallel pool (%s). Running serially.', ME.message);
    end

    t_all = tic;
    parfor d = 1:nDays
        dayId = firstDay + d - 1;
        f = fullfile(outDir, sprintf('day_%03d.mat', dayId));
        if exist(f, 'file')
            fprintf('day %d: already exists, skipped\n', dayId);
        else
            t_day = tic;
            R = run_day(scns{d}, config(), seed + 1000*dayId, nSteps, dayId);
            R.seed = seed;
            R.scn  = scns{d};
            save_day(f, R);
            fprintf('day %d done: %d samples, %d failed solves, %.1f min\n', ...
                    dayId, size(R.X, 1), R.nFail, toc(t_day)/60);
        end
    end
    fprintf('gen_dataset finished in %.1f min\n', toc(t_all)/60);
end


% =====================================================================
function R = run_day(scn, cfg, dayseed, nSteps, dayId)
% One closed-loop day: DMPC drives the plant, local MPC labels every loop at every step.

    if exist('build_features', 'file') == 2
        fb = @build_features;
    else
        fb = @build_features_stub;
    end

    s    = initial_state(cfg);
    nsub = cfg.Tc / cfg.dT;                       % 120 plant steps per controller step
    rs   = RandStream('mt19937ar', 'Seed', dayseed);

    nMax = nSteps * cfg.Nl;
    X     = zeros(nMax, 411);
    Y     = zeros(nMax, 1);
    Yseq  = zeros(nMax, cfg.Nu);
    capv  = zeros(nMax, 1);
    loopv = zeros(nMax, 1);
    kv    = zeros(nMax, 1);
    r = 0;  nFail = 0;  t_label = 0;

    for k = 1:nSteps
        s.t = (k - 1) * cfg.Tc;
        Dp  = get_dni_preview(scn, s.t, cfg);

        % ---- label every loop at this state with a random cap ----
        for j = 1:cfg.Nl
            ls  = coarsen_state(s, j, cfg);
            dni = squeeze(Dp(j, :, :));            % [29 x 12]
            cap = draw_cap(rs, cfg);
            try
                t0 = tic;
                [q, qs] = local_mpc_fmincon(ls, dni, s.q_prev(j), s.Tin, s.Tout, cfg.Ta, cap, cfg);
                t_label = t_label + toc(t0);
                if ~isfinite(q), error('non-finite label'); end
                x = fb(ls, dni, s.q_prev(j), s.Tin, s.Tout, cfg.Ta, cap, cfg);
                r = r + 1;
                X(r, :)    = x;
                Y(r)       = q;
                Yseq(r, :) = qs;
                capv(r)    = cap;
                loopv(r)   = j;
                kv(r)      = k;
            catch
                nFail = nFail + 1;                 % skip this sample, keep going
            end
        end

        % ---- advance the plant with the distributed MPC ----
        [qc, ~] = dmpc_fmincon(s, scn, cfg);
        qc = min(max(qc(:), cfg.qmin), cfg.qmax);
        for m = 1:nsub
            D      = get_dni(scn, s.t + (m - 1) * cfg.dT, cfg);
            [s, ~] = plant_step(s, qc * 1e-3, D, cfg);
        end
        s.q_prev = qc;
    end

    R.X      = X(1:r, :);
    R.Y      = Y(1:r);
    R.Yseq   = Yseq(1:r, :);
    R.cap    = capv(1:r);
    R.loop   = loopv(1:r);
    R.k      = kv(1:r);
    R.day    = dayId * ones(r, 1);
    R.nFail  = nFail;
    R.tLabel = t_label / max(r, 1);                % mean seconds per label
end


function cap = draw_cap(rs, cfg)
% Runtime only uses caps of 1.5 (Step 2) and 0.65 (Step 4); intermediate caps add smoothness.
    u = rand(rs);
    if u < 0.40
        cap = cfg.qmax;                            % 1.5
    elseif u < 0.80
        cap = cfg.qTmax / cfg.Nl;                  % 0.65
    else
        cap = 0.3 + 0.9 * rand(rs);                % uniform in [0.3, 1.2]
    end
end


function save_day(f, R) %#ok<INUSD>
% save inside parfor must go through a function
    save(f, '-struct', 'R');
end
