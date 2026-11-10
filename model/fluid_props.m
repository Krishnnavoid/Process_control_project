function [rho, Cf, Hl, Ht] = fluid_props(Tf, q, Ta)
% FLUID_PROPS Computes Therminol VP-1 properties and heat transfer coefficients.
% Protected against negative trial flows (prevents complex numbers in fmincon)
% and extreme temperature excursions outside the empirical polynomial range [100, 320] degC.

    % Clamp temperature to valid physical range of Therminol VP-1 to prevent
    % 4th-order polynomial blow-up and NaN overflow during low-flow trial steps
    Tf_safe = min(max(real(Tf), 100), 320);
    q_safe  = max(real(q), 1e-6); % Prevent complex numbers from negative q^(0.8)

    % Density and specific heat capacity (Frejo & Camacho 2020 Eq. 20-21)
    rho = 903 - 0.672 * Tf_safe;
    Cf  = 1820 + 3.478 * Tf_safe;

    % Thermal loss coefficient (Eq. 22)
    Hl  = max(0, 0.00249 * (Tf_safe - Ta) - 0.06133);

    % Metal-fluid convective heat transfer coefficient (Eq. 23, q in m^3/s)
    Ht  = (q_safe .^ 0.8) .* (2.17e6 - 5.01e4 * Tf_safe + 4.53e2 * (Tf_safe.^2) ...
                              - 1.64 * (Tf_safe.^3) + 2.1e-3 * (Tf_safe.^4));
end
