function [rho, Cf, Hl, Ht] = fluid_props(Tf, q, Ta)
% FLUID_PROPS Computes Therminol VP-1 properties and heat transfer coefficients.
%
% Inputs:
%   Tf : Fluid temperature array [degC] (any size, e.g., [Nl x Ns] or [1 x Nsc])
%   q  : Volumetric flow rate [m^3/s] (scalar or column vector [Nl x 1])
%   Ta : Ambient temperature [degC] (scalar)
%
% Outputs:
%   rho : Fluid density [kg/m^3]
%   Cf  : Fluid specific heat capacity [J/(kg*degC)]
%   Hl  : Thermal loss coefficient to ambient [W/(m^2*degC)]
%   Ht  : Metal-to-fluid convective heat transfer coefficient [W/(m^2*degC)]

    % Density and specific heat capacity (Ruiz-Moreno et al. 2021 Eq. 15 & Frejo 2020 Eq. 20-21)
    rho = 903 - 0.672 * Tf;
    Cf  = 1820 + 3.478 * Tf;

    % Thermal loss coefficient (Eq. 18 / Eq. 22)
    Hl  = 0.00249 * (Tf - Ta) - 0.06133;

    % Metal-fluid convective heat transfer coefficient (q MUST be in m^3/s)
    % Uses implicit expansion (.*): [Nl x 1] .* [Nl x Ns] or scalar .* [1 x Nsc]
    Ht  = (q .^ 0.8) .* (2.17e6 - 5.01e4 * Tf + 4.53e2 * (Tf.^2) ...
                         - 1.64 * (Tf.^3) + 2.1e-3 * (Tf.^4));
end