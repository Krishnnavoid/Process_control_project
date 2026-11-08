function ls = coarsen_state(s, j, cfg)
% COARSEN_STATE Coarsens the 1 m plant state (174 segments) of loop j
% onto the 6 m control model grid (29 segments) by block averaging.
%
% Inputs:
%   s   : Full field state struct (s.Tf, s.Tm of size [10 x 174])
%   j   : Loop index (1 to 10)
%   cfg : Configuration struct (optional; defaults to config() if omitted)
%
% Output:
%   ls  : Coarsened loop state struct with fields ls.Tf [1 x 29] and ls.Tm [1 x 29]

    if nargin < 3
        cfg = config();
    end

    ratio = cfg.dlc / cfg.dl; % 6 m / 1 m = 6 segments per block
    blk   = @(X) mean(reshape(X(j, :), ratio, []), 1);

    ls.Tf = blk(s.Tf); % [1 x 29] [degC]
    ls.Tm = blk(s.Tm); % [1 x 29] [degC]
end