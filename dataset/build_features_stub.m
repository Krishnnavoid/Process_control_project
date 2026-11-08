function x = build_features_stub(ls, dni_prev, q_prev, Tin, Tout, Ta, qmax_eff, cfg) %#ok<INUSD>
% BUILD_FEATURES_STUB  Temporary feature builder with the agreed 411-column layout.
% Replace by Part 3's build_features.m (same argument order) when it exists.
%
% Inputs (same order as local_mpc_fmincon):
%   ls        : coarse loop state, ls.Tf [1x29], ls.Tm [1x29]  (6 m blocks)
%   dni_prev  : predicted effective DNI of this loop [29 x Np]  (Np = 12)
%   q_prev    : last applied flow of this loop [l/s]
%   Tin, Tout : field inlet / outlet temperature [degC]
%   Ta        : ambient temperature [degC]
%   qmax_eff  : flow cap given to the local MPC [l/s]
%
% Output x [1 x 411]:
%   1        q(k-1)
%   2        Tin
%   3        field Tout
%   4        Ta
%   5-33     Tf at the 29 coarse segments
%   34-62    Tm at the 29 coarse segments
%   63-410   effective DNI, 12 prediction steps, step-major (29 values of step 1, then step 2, ...)
%   411      qmax_eff

    Nsc = numel(ls.Tf);                    % 29
    Np  = size(dni_prev, 2);               % 12
    assert(Nsc == 29 && Np == 12, 'build_features_stub: expected 29 segments and 12 steps');

    x = [ q_prev, Tin, Tout, Ta, ...
          ls.Tf(:).', ls.Tm(:).', ...
          dni_prev(:).', ...               % column-major: step 1 (29 values), step 2, ...
          qmax_eff ];

    assert(numel(x) == 411, 'build_features_stub: feature vector must have 411 entries');
end
