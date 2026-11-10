function [Tout_traj, W_traj] = simulate_loop_ctrl(ls, q_seq, dni_prev, Tin0, Tout_hold, Ta, cfg)
% SIMULATE_LOOP_CTRL Simulates a single collector loop over the prediction horizon Np
% using the coarsened control grid (3 s step, 6 m segments, 29 segments).
%
% Inputs:
%   ls        : Coarsened loop state struct (ls.Tf [1x29], ls.Tm [1x29]) [degC]
%   q_seq     : Candidate control sequence [1 x Nu] or [Nu x 1] in [l/s]
%   dni_prev  : Predicted effective DNI for this loop [29 x Np] in [W/m]
%   Tin0      : Current measured field inlet temperature at start of horizon [degC]
%   Tout_hold : Last measured field outlet temperature, held constant over horizon [degC]
%   Ta        : Ambient temperature [degC]
%   cfg       : Configuration struct
%
% Outputs:
%   Tout_traj : Predicted loop outlet temperature at end of each minute [1 x Np] [degC]
%   W_traj    : Predicted loop net thermal power at end of each minute [1 x Np] [kW]

    Np    = size(dni_prev, 2);
    Nu    = numel(q_seq);
    % Hold the last control move constant for steps Nu+1 ... Np
    q_seq = [q_seq(:).', q_seq(end) * ones(1, max(0, Np - Nu))];
    nsub  = cfg.Tc / cfg.dTc; % 60 s / 3 s = 20 sub-steps per minute

    Tf  = ls.Tf(:).'; % Row vector [1 x 29]
    Tm  = ls.Tm(:).'; % Row vector [1 x 29]
    Tin = Tin0;

    Tout_traj = zeros(1, Np);
    W_traj    = zeros(1, Np);

    % Exact discrete filter pole for 3 s step
    a = exp(-cfg.dTc / cfg.tau_in);

    for n = 1:Np
        q = q_seq(n) * 1e-3; % Convert flow rate from [l/s] to [m^3/s]
        c = (q * cfg.dTc) / (cfg.Af * cfg.dlc); % Transport number (must be <= 1)

        if c > 1 + 1e-6
            warning('simulate_loop_ctrl:CFLViolation', ...
                'Control model CFL c = %.3f > 1 (q = %.2f l/s).', c, q_seq(n));
        end

        D_n = dni_prev(:, n).'; % [1 x 29] effective DNI in W/m for minute n

        for m = 1:nsub
            [rho, Cf, Hl, Ht] = fluid_props(Tf, q, Ta);

            Tprev = [Tin, Tf(1:end-1)];

            Tf_n = Tf - c * (Tf - Tprev) ...
                 + (pi * cfg.Df * cfg.dTc) * Ht ./ (rho .* Cf * cfg.Af) .* (Tm - Tf);

            Tm   = Tm + (cfg.dTc / (cfg.rhom * cfg.Cm * cfg.Am)) * ...
                   (D_n - pi * cfg.Dm * Hl .* (Tm - Ta) - pi * cfg.Df * Ht .* (Tm - Tf));

            Tf   = Tf_n;

            % Update inlet temperature holding last measured field outlet constant
            Tin  = a * Tin + (1 - a) * (Tout_hold - cfg.dT_steam);
        end

        % Compute net thermal power at the end of minute n [kW]
        [rN, CN]     = fluid_props(Tf(end), q, Ta);
        [r1, C1]     = fluid_props(Tin, q, Ta);
        Tout_traj(n) = Tf(end);
        W_traj(n)    = (rN * CN * q * Tf(end) - r1 * C1 * q * Tin) / 1e3;
    end
end
