function s = initial_state(cfg)
% INITIAL_STATE Initializes the 10-loop solar field state struct at t = 0 s.
% Matches initial conditions from Frejo & Camacho (2020) Fig. 8.

    Tin0  = 155; % Initial field inlet temperature [degC]
    Tout0 = 255; % Initial field outlet temperature [degC]

    % Linearly distributed initial fluid temperature profile along the 174 segments
    Tf = repmat(linspace(Tin0, Tout0, cfg.Ns), cfg.Nl, 1);

    s.Tf     = Tf;                      % Fluid temperature [Nl x 174] [degC]
    s.Tm     = Tf + 5;                  % Metal tube temperature [Nl x 174] [degC]
    s.T1f    = Tf;                      % Auxiliary temperature kept for contract compatibility [Nl x 174]
    s.Tin    = Tin0;                    % Scalar field inlet temperature [degC]
    s.Tout   = Tout0;                   % Scalar field outlet temperature [degC]
    s.q_prev = 0.5 * ones(cfg.Nl, 1);   % Previous applied loop flow rates [Nl x 1] [l/s]
    s.t      = 0;                       % Current simulation time [s]
end