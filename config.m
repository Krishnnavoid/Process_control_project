function cfg = config()
% CONFIG Shared configuration parameters and operational constraints for ACUREX.
% Strictly adheres to Frejo & Camacho (2020), Ruiz-Moreno et al. (2021),
% and the CHE381 Part 1 Step-by-Step Guide.

    %% 1. Plant Discretisation & Geometry
    cfg.Nl = 10;                            % Number of parallel collector loops [-]
    cfg.Ns = 174;                           % Number of segments per loop [-]
    cfg.dl = 1;                             % Plant segment length [m]
    cfg.dT = 0.5;                           % Plant simulation time step [s]
    cfg.passive = [37:42, 79:96, 133:138];  % Passive pipe segments (30 m total, 144 m active)

    %% 2. Physical Properties (Metal Tube & Pipe Geometry)
    cfg.rhom = 7800;                        % Metal density [kg/m^3]
    cfg.Cm   = 550;                         % Metal specific heat capacity [J/(kg*degC)]
    cfg.Dm   = 0.031;                       % Pipe outside diameter [m]
    cfg.Df   = 0.0254;                      % Pipe inside diameter [m]
    cfg.Am   = 2.48e-4;                     % Metal cross-sectional area [m^2]
    cfg.Af   = 7.55e-4;                     % Fluid cross-sectional area [m^2]
    cfg.Ta   = 25;                          % Ambient temperature [degC]

    %% 3. Operational Constraints (Corrected for swapped labels in paper tables)
    cfg.qmin  = 0.2;                        % Minimum per-loop flow rate [l/s]
    cfg.qmax  = 1.5;                        % Maximum per-loop flow rate [l/s] (CFL limit c <= 0.993)
    cfg.qTmax = 6.5;                        % Maximum total field pump flow rate [l/s] (0.65 l/s per loop cap)
    cfg.Tmin  = 220;                        % Minimum allowed HTF outlet temperature [degC]
    cfg.Tmax  = 300;                        % Maximum allowed HTF outlet temperature [degC]

    %% 4. Inlet Temperature Steam-Generator Dynamics (10 min time constant, 0.5 s step)
    cfg.a_in     = 0.999167;                % Discrete AR coefficient exp(-0.5/600) [-]
    cfg.b_in     = 0.000833;                % Discrete input gain 1 - exp(-0.5/600) [-]
    cfg.dT_steam = 90;                      % Temperature drop across steam generator [degC]
    cfg.tau_in   = 600;                     % Continuous time constant [s]

    %% 5. Controller & Coarsened Control Model Parameters
    cfg.Tc    = 60;                         % Controller sampling interval [s]
    cfg.dTc   = 3;                          % Control model integration step [s] (alpha = 20 sub-steps/min)
    cfg.dlc   = 6;                          % Control model segment length [m]
    cfg.Nsc   = 29;                         % Number of control segments per loop (174 / 6) [-]
    cfg.Np    = 12;                         % Prediction horizon [steps / minutes]
    cfg.Nu    = 10;                         % Control horizon [steps / minutes]

    %% 6. MPC Cost Function Weights (Calibrated for W in MW)
    cfg.psi_c = 450;                        % Centralized soft temperature constraint weight [-]
    cfg.eps_c = 30;                         % Centralized flow slew-rate penalty weight [1/(l/s)^2]
    cfg.psi_l = 45;                         % Local/Distributed soft temperature constraint weight [-]
    cfg.eps_l = 3;                          % Local/Distributed flow slew-rate penalty weight [1/(l/s)^2]
end
