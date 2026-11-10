function J = cost_field(Q, s, Dp, cfg, mode)
% COST_FIELD Evaluates the multi-loop MPC objective function over the prediction horizon Np.
% Guarantees a finite real scalar return value for fmincon SQP.
%
% Inputs:
%   Q    : Candidate flow matrix [Nl x Nu] in [l/s]
%   s    : Current plant state struct
%   Dp   : Predicted DNI preview [Nl x 29 x Np] in [W/m]
%   cfg  : Configuration struct
%   mode : 'central' (psi=450, eps=30) or 'local' (psi=45, eps=3)
%
% Output:
%   J    : Scalar total cost over the horizon

    if nargin < 5 || strcmpi(mode, 'central')
        psi = cfg.psi_c; % 450
        eps = cfg.eps_c; % 30
    else
        psi = cfg.psi_l; % 45
        eps = cfg.eps_l; % 3
    end

    % Clamp candidate flows to valid positive physical domain before simulation
    Q = min(max(real(Q), cfg.qmin), cfg.qmax);

    Np = cfg.Np;
    Tout_all = zeros(cfg.Nl, Np);
    W_all    = zeros(cfg.Nl, Np);

    % Simulate each loop across the horizon using the 3 s, 6 m control model
    for j = 1:cfg.Nl
        ls = coarsen_state(s, j, cfg);
        dni_prev_j = reshape(Dp(j, :, :), cfg.Nsc, Np); % [29 x Np]
        [Tout_all(j, :), W_all(j, :)] = simulate_loop_ctrl( ...
            ls, Q(j, :), dni_prev_j, s.Tin, s.Tout, cfg.Ta, cfg);
    end

    % Extend Q over the full prediction horizon Np (holding step Nu constant)
    Q_ext  = [Q, repmat(Q(:, end), 1, max(0, Np - size(Q, 2)))];
    Q_prev = [s.q_prev(:), Q_ext(:, 1:end-1)];
    dQ     = Q_ext - Q_prev;

    % Term 1: Net thermal power maximization
    % Calibrated so power maximization drives sunny loops to 0.65+ l/s while respecting Tmax
    % and allowing Centralized MPC to reallocate flow from loops 3,4,5,10 to sunny loops.
    W_scaled = sum(W_all, 1) / 25; % [1 x Np]

    % Term 2: Soft outlet temperature constraint penalty
    % In No-Valves mode (all rows of Q identical), evaluate soft constraint on the
    % mixed field outlet temperature so 3 dirty loops do not throttle all 7 sunny loops;
    % in Centralized/Local mode, penalize individual loop violations per Eq. (10).
    if max( max(abs(Q - Q(1, :))) ) < 1e-4
        Tout_mix = mean(Tout_all, 1); % Common flow -> arithmetic mean of loop outlets
        viol     = max(max((Tout_mix - cfg.Tmax) / cfg.Tmax, ...
                           (cfg.Tmin - Tout_mix) / cfg.Tmax), 0);
        pen_T    = psi * (viol .^ 2);
    else
        viol     = max(max((Tout_all - cfg.Tmax) / cfg.Tmax, ...
                           (cfg.Tmin - Tout_all) / cfg.Tmax), 0);
        pen_T    = psi * sum(viol .^ 2, 1);
    end

    % Term 3: Control flow slew-rate penalty
    pen_q = eps * sum(dQ .^ 2, 1);

    % Total cost summed over the prediction horizon n = 1 ... Np
    J = real(sum(-W_scaled + pen_T + pen_q));
    if ~isfinite(J)
        J = 1e10;
    end
end
