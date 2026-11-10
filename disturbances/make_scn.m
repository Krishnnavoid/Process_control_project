function scn = make_scn(cfg)
% MAKE_SCN Creates the 2-hour benchmark scenario struct from Frejo & Camacho (2020).
% Uses D0 = 800 W/m and matches the Table 3 mean power targets (~1118.6 kW No-Valves, ~1130.8 kW Central).

    scn.D0  = 800; % Base effective DNI [W/m] (Section 8.1 of Part 1 Guide)
    scn.eff = ones(cfg.Nl, cfg.Ns);

    % Dirty / defocused collector segments matching Frejo & Camacho (2020) Fig. 3
    scn.eff([1, 9], 1:36)    = 0.68; % Soiled first collector on loops 1 and 9
    scn.eff(10, 43:132)      = 0.42; % Degraded middle collectors on loop 10
    scn.eff(3:5, 97:132)     = 0.20; % Defocused 3rd collector group on loops 3, 4, 5
    scn.eff(3, 139:174)      = 0.65; % Degraded end of loop 3
    scn.eff(8, 139:174)      = 0.82; % Slight degradation at end of loop 8

    % Zero out passive pipe joints (segments 37:42, 79:96, 133:138 -> 30 m passive, 144 m active)
    scn.eff(:, cfg.passive)  = 0;

    % Moving cumulus cloud parameters (Minutes 63 to 76, Fig. 4)
    scn.cloud = struct( ...
        't0',    63, ...    % Cloud appearance time [min]
        't1',    76, ...    % Cloud exit time [min]
        'seg0',  10, ...    % Initial segment affected at t0 [-]
        'speed', 11, ...    % Cloud front propagation speed [segments/min]
        'len',   60, ...    % Cloud width [segments]
        'loops', 3:9, ...   % Collector loops affected by the cloud
        'depth', 0.80, ...  % Fractional reduction in DNI at cloud core
        'edge',  4 ...      % Sigmoid softness parameter for cloud edges [segments]
    );
end
