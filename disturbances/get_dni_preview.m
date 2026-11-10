function Dp = get_dni_preview(scn, t_sec, cfg)
% GET_DNI_PREVIEW Computes the 12-minute DNI preview over the coarsened control grid.
%
% Inputs:
%   scn   : Scenario struct from make_scn(cfg)
%   t_sec : Current simulation time [s]
%   cfg   : Configuration struct
%
% Output:
%   Dp    : Coarsened effective DNI preview [Nl x 29 x Np] in [W/m]

    Dp = zeros(cfg.Nl, cfg.Nsc, cfg.Np);
    for n = 1:cfg.Np
        D = get_dni(scn, t_sec + (n - 1) * cfg.Tc, cfg);
        % Block-average every 6 segments along dimension 2
        Dp(:, :, n) = squeeze(mean(reshape(D, cfg.Nl, cfg.dlc, cfg.Nsc), 2));
    end
end
