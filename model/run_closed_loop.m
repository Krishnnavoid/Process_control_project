function log = run_closed_loop(ctrl, name, scn, cfg, Tend)
% RUN_CLOSED_LOOP Generic closed-loop runner for any controller matching the contract:
%   [q, info] = ctrl(s, scn, cfg)
% Executes the 0.5 s plant for Tend seconds (default 7200 s) and returns the standard log.

    if nargin < 5
        Tend = 7200; % Default 2-hour benchmark run [s]
    end

    s    = initial_state(cfg);
    K    = Tend / cfg.Tc;      % Total number of 60 s controller steps (120)
    nsub = cfg.Tc / cfg.dT;    % 120 plant steps (0.5 s) per controller step

    % Preallocate standard log structure
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

    for k = 1:K
        s.t = (k - 1) * cfg.Tc;

        % Invoke controller and measure execution time
        t_start   = tic;
        [q, info] = ctrl(s, scn, cfg);
        tc        = toc(t_start);

        % Safety clip to per-loop physical bounds [l/s]
        q = min(max(q(:), cfg.qmin), cfg.qmax);

        % Advance the high-resolution plant for 60 s (120 steps of 0.5 s)
        Wacc = 0;
        for m = 1:nsub
            D       = get_dni(scn, s.t + (m - 1) * cfg.dT, cfg);
            [s, W]  = plant_step(s, q * 1e-3, D, cfg);
            Wacc    = Wacc + W;
        end
        s.q_prev = q;

        % Record step data into shared log format
        log.q(:, k)          = q;
        log.Tout_loop(:, k)  = s.Tf(:, end);
        log.Tout_field(k)    = s.Tout;
        log.Tin(k)           = s.Tin;
        log.W_kW(k)          = Wacc / nsub;
        log.time.total(k)    = tc;

        if isstruct(info) && isfield(info, 'time')
            if isfield(info.time, 'local'), log.time.local(k) = info.time.local; end
            if isfield(info.time, 'coord'), log.time.coord(k) = info.time.coord; end
        end

        % Save periodic checkpoint every 10 steps for long MPC runs
        if mod(k, 10) == 0 && exist('logs', 'dir')
            save(fullfile('logs', sprintf('checkpoint_%s.mat', name)), 'log', 'k');
        end
    end
end