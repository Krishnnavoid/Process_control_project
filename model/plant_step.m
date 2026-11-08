function [s, W_kW] = plant_step(s, q, D, cfg)
% PLANT_STEP Advances the 10-loop collector field by one 0.5 s step (vectorised).
% Uses the standard distributed-parameter upwind PDE discretization to prevent steady-state drift.
%
% Inputs:
%   s   : Plant state struct
%   q   : Loop volumetric flow rates [Nl x 1] in [m^3/s]
%   D   : Effective concentrated solar irradiance (eta*G*I) [Nl x Ns] in [W/m]
%   cfg : Configuration struct from config()
%
% Outputs:
%   s    : Updated plant state struct
%   W_kW : Instantaneous net field thermal power [kW]

    q = q(:); % Ensure column vector [Nl x 1] in m^3/s
    dT = cfg.dT;

    % 1. Evaluate fluid properties and heat transfer coefficients at k-1
    [rho, Cf, Hl, Ht] = fluid_props(s.Tf, q, cfg.Ta);

    % 2. Compute Courant-Friedrichs-Lewy (CFL) transport number c <= 1
    c = (q * dT) / (cfg.Af * cfg.dl); % [Nl x 1]
    if any(c > 1 + 1e-6)
        warning('plant_step:CFLViolation', ...
            'Transport condition c = %.3f > 1 violated! Ensure q <= 1.5 l/s.', max(c));
    end

    % 3. Upwind spatial differences (boundary condition T_0^f = s.Tin)
    Tprev = [s.Tin * ones(cfg.Nl, 1), s.Tf(:, 1:end-1)];

    % 4. Energy balance on HTF (Standard Upwind Form)
    Tf = s.Tf - c .* (s.Tf - Tprev) ...
       + (pi * cfg.Df * dT) * Ht ./ (rho .* Cf * cfg.Af) .* (s.Tm - s.Tf);

    % 5. Energy balance on Metal Tube
    Tm = s.Tm + (dT / (cfg.rhom * cfg.Cm * cfg.Am)) * ...
         (D - pi * cfg.Dm * Hl .* (s.Tm - cfg.Ta) - pi * cfg.Df * Ht .* (s.Tm - s.Tf));

    % 6. Field outlet mixing (flow-weighted average of previous loop outlets) & Inlet filter
    ToutLoop = s.Tf(:, end);
    s.Tout   = sum(q .* ToutLoop) / sum(q);
    s.Tin    = cfg.a_in * s.Tin + cfg.b_in * (s.Tout - cfg.dT_steam);

    % 7. Commit updated states
    s.Tf  = Tf;
    s.Tm  = Tm;
    s.T1f = Tf; % Retained strictly for interface contract compatibility

    % 8. Compute net thermal power in kW: sum_j(W_out,j - W_in,j)
    [rN, CN] = fluid_props(Tf(:, end), q, cfg.Ta);
    [r1, C1] = fluid_props(s.Tin, q, cfg.Ta);
    W_kW     = sum(rN .* CN .* q .* Tf(:, end) - r1 .* C1 .* q .* s.Tin) / 1e3;
end