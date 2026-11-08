function scn = make_scn(cfg)
% MAKE_SCN Creates the 2-hour benchmark scenario struct from Frejo & Camacho (2020).
% Includes base DNI (800 W/m), dirty collector segments, passive joints, and moving cloud.

    scn.D0  = 800; % Base effective DNI [W/m] (~1.15 MW total field thermal power)
    scn.eff = ones(cfg.Nl, cfg.Ns);

    % Dirty / defocused collectors matching Frejo & Camacho (2020) Fig. 3
    scn.eff([1, 9], 1:36)    = 0.70; % First 36 segments of loops 1 and 9
    scn.eff(10, 43:132)      = 0.35; % Degraded middle collectors on loop 10
    scn.eff(3:5, 97:132)     = 0.15; % Severely defocused collectors on loops 3-5
    scn.eff(3, 139:174)      = 0.70; % Slight degradation at end of loop 3
    scn.eff(8, 139:174)      = 0.85; % Slight degradation at end of loop 8

    % Zero out passive pipe joints (segments 37:42, 79:96, 133:138)
    scn.eff(:, cfg.passive)  = 0;

    % Moving cumulus cloud parameters (Minutes 63 to 76, Fig. 4)
    scn.cloud = struct( ...
        't0',    63, ...    % Cloud appearance time [min]
        't1',    76, ...    % Cloud exit time [min]
        'seg0',  10, ...    % Initial segment affected at t0 [-]
        'speed', 11, ...    % Cloud front propagation speed [segments/min]
        'len',   60, ...    % Cloud width [segments]
        'loops', 3:9, ...   % Collector loops affected by the cloud
        'depth', 0.8, ...   % Fractional reduction in DNI at cloud core [-]
        'edge',  4 ...      % Sigmoid softness parameter for cloud edges [segments]
    );
end