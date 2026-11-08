function scns = make_training_days(N, seed, cfg)
% MAKE_TRAINING_DAYS Generates N randomised disturbance scenarios for Part 2 dataset generation.
% Keeps the 2-hour Frejo & Camacho (2020) scenario strictly held out.

    if nargin < 3, cfg = config(); end
    if nargin >= 2 && ~isempty(seed), rng(seed); end

    scns = cell(1, N);
    for idx = 1:N
        s_day.D0  = 500 + 400 * rand(); % Random base DNI in [500, 900] W/m
        s_day.eff = ones(cfg.Nl, cfg.Ns);

        % Randomly soil a subset of collector loops (efficiency in [0.5, 1.0])
        num_dirty  = randi([1, 5]);
        dirty_lps  = randperm(cfg.Nl, num_dirty);
        for lp = dirty_lps
            seg_start = randi([1, 4]) * 36 - 35;
            seg_range = seg_start:min(seg_start + 35, cfg.Ns);
            s_day.eff(lp, seg_range) = 0.5 + 0.45 * rand();
        end
        s_day.eff(:, cfg.passive) = 0; % Zero out passive segments

        % Include cloud-free days (~15% of scenarios) and passing cloud days
        if rand() < 0.15
            s_day.cloud = struct('t0', 999, 't1', 998, 'seg0', 10, ...
                'speed', 10, 'len', 50, 'loops', 1:10, 'depth', 0, 'edge', 4);
        else
            t0    = 15 + 75 * rand();
            dur   = 8 + 18 * rand();
            n_hit = randi([4, cfg.Nl]);
            l_st  = randi([1, cfg.Nl - n_hit + 1]);
            s_day.cloud = struct( ...
                't0',    t0, ...
                't1',    t0 + dur, ...
                'seg0',  randi([1, 30]), ...
                'speed', 5 + 15 * rand(), ...
                'len',   30 + 60 * rand(), ...
                'loops', l_st:(l_st + n_hit - 1), ...
                'depth', 0.3 + 0.65 * rand(), ...
                'edge',  4 ...
            );
        end
        scns{idx} = s_day;
    end
end