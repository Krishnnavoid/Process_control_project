function J = cost_field(Q, s, Dp, cfg, mode)
% COST_FIELD Evaluates the multi-loop MPC objective function over the prediction horizon Np.
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

    Np = cfg.Np;
    Nu = size(Q, 2);

    % Preallocate predicted trajectories across all 10 loops and Np steps
    Tout_all = zeros(cfg.Nl, Np);
    W_all    = zeros(cfg.Nl, Np);

    % Simulate each loop across the horizon using the 3 s, 6 m control model
    for j = 1:cfg.Nl
        ls = coarsen_state(s, j, cfg);
        dni_prev_j = squeeze(Dp(j, :, :)); % [29 x Np]
        [Tout_all(j, :), W_all(j, :)] = simulate_loop_ctrl( ...
            ls, Q(j, :), dni_prev_j, s.Tin, s.Tout, cfg.Ta, cfg);
    end

    % Extend Q over the full prediction horizon Np (holding step Nu constant)
    Q_ext = [Q, repmat(Q(:, end), 1, max(0, Np - Nu))];

    % Flow rate differences: q_j(n) - q_j(n-1)
    % Note: For n > Nu, q_j(n) == q_j(n-1), so dQ(:, Nu+1:Np) is identically 0
    Q_prev = [s.q_prev(:), Q_ext(:, 1:end-1)];
    dQ     = Q_ext - Q_prev;

    % Term 1: Net thermal power maximization (W converted from kW to MW for calibrated scaling)
    W_MW = sum(W_all, 1) / 1e3; % [1 x Np] in [MW]

    % Term 2: Soft outlet temperature constraint penalty
    viol = max(max((Tout_all - cfg.Tmax) / cfg.Tmax, ...
                   (cfg.Tmin - Tout_all) / cfg.Tmax), 0); % [Nl x Np]
    pen_T = psi * sum(viol .^ 2, 1);                      % [1 x Np]

    % Term 3: Control flow slew-rate penalty
    pen_q = eps * sum(dQ .^ 2, 1);                        % [1 x Np]

    % Total cost summed over the prediction horizon n = 1 ... Np
    J = sum(-W_MW + pen_T + pen_q);
end