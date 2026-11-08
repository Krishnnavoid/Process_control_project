function D = get_dni(scn, t_sec, cfg)
% GET_DNI Computes the spatial effective DNI map [Nl x Ns] in [W/m] at time t_sec [s].
% Never stores the full 3D time-space array in memory.

    if nargin < 3
        cfg = config();
    end

    D  = scn.D0 * scn.eff;
    c  = scn.cloud;
    tm = t_sec / 60; % Convert time to minutes

    if tm >= c.t0 && tm <= c.t1
        front = c.seg0 + (tm - c.t0) * c.speed; % Moving cloud leading edge
        seg   = 1:cfg.Ns;

        % Smooth double-sigmoid spatial window for cloud shadow
        sh = (1 ./ (1 + exp(-(seg - (front - c.len)) / c.edge))) .* ...
             (1 - 1 ./ (1 + exp(-(seg - front) / c.edge)));

        D(c.loops, :) = D(c.loops, :) .* (1 - c.depth * sh);
    end
end