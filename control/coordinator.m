function [q, info] = coordinator(s, scn, cfg, solver, opts)
% COORDINATOR  Steps 2-11 of the logic-based distributed MPC (Frejo & Camacho 2020).
%
%   [q, info] = coordinator(s, scn, cfg, solver)
%   [q, info] = coordinator(s, scn, cfg, solver, opts)
%
% The coordinator does not know whether the local MPC is fmincon or a neural network.
% It only calls the function handle "solver":
%
%   [q, qseq] = solver(ls, dni_prev, q_prev, Tin, Tout, Ta, qmax_eff, cfg)
%
%     q    : first-step flow of this loop [l/s]
%     qseq : planned flow sequence [1 x Nu]. A scalar is also accepted (e.g. a network
%            that only predicts the first move); the saturation test and the cost
%            perturbation then use the first move only (held over the horizon).
%
% Optional opts fields (defaults in brackets):
%   lambda  [0.01]  flow increment for Steps 7-10 [l/s]
%   qhi     [min(cfg.qmax, 1.2)]  largest flow a loop may be raised to [l/s]
%           (the 3 s / 6 m control model is unstable above ~1.25 l/s)
%   gmin    [1e-4]  smallest marginal cost improvement still worth acting on [cost per l/s]
%   tolq    [1e-3]  tolerance for "saturated" and "total flow at maximum" [l/s]
%   maxIter [500]   safety limit on Steps 6-10 iterations
%
% Output q is [Nl x 1] in l/s. info holds the timing split and diagnostics.

    if nargin < 5 || isempty(opts), opts = struct(); end
    lambda  = getopt(opts, 'lambda',  0.01);
    qhi     = getopt(opts, 'qhi',     min(cfg.qmax, 1.2));
    gmin    = getopt(opts, 'gmin',    1e-4);
    tolq    = getopt(opts, 'tolq',    1e-3);
    maxIter = getopt(opts, 'maxIter', 500);

    Nl  = cfg.Nl;
    qT  = cfg.qTmax;
    cap = qT / Nl;
    t_all = tic;

    % ---- Step 1: data for every loop (true future DNI = perfect preview) ----
    Dp  = get_dni_preview(scn, s.t, cfg);       % [Nl x 29 x Np]
    ls  = cell(Nl, 1);
    dni = cell(Nl, 1);
    for j = 1:Nl
        ls{j}  = coarsen_state(s, j, cfg);
        dni{j} = squeeze(Dp(j, :, :));          % [29 x Np]
    end

    info.time.step2 = 0;
    info.time.step4 = 0;
    info.iters      = 0;

    % ---- Step 2: local MPC per loop, no global constraint ----
    t2 = tic;
    [q2, ~] = solve_all(solver, ls, dni, s, cfg, cfg.qmax);
    info.time.step2 = toc(t2);
    info.q_step2    = q2;

    % ---- Step 3: feasibility ----
    if sum(q2) <= qT + 1e-9
        q = q2;
        info.exit = 'step3_feasible';
        info = finish_info(info, t_all);
        return
    end

    % ---- Step 4: re-solve with cap qT/Nl (feasible, not necessarily optimal) ----
    t4 = tic;
    [q4, S4] = solve_all(solver, ls, dni, s, cfg, cap);
    info.time.step4 = toc(t4);
    info.q_step4    = q4;

    % ---- Step 5: optimality check (saturated at the cap anywhere in the horizon) ----
    sat = false(Nl, 1);
    for j = 1:Nl
        sat(j) = any(S4{j} >= cap - tolq);
    end
    info.saturated = sat;
    if ~any(sat)
        q = q4;
        info.exit = 'step5_no_saturation';
        info = finish_info(info, t_all);
        return
    end

    % ---- Steps 6-10: improve optimality while keeping sum(q) <= qT ----
    cur = q4;                                   % current first-step flows
    J   = @(j, x) loop_cost(j, x, S4, ls, dni, s, cfg);   % cost of loop j if first move = x
    Jc   = nan(Nl, 1);  gain = nan(Nl, 1);  loss = nan(Nl, 1);   % NaN = needs refresh
    info.exit = 'maxIter';

    for it = 1:maxIter
        info.iters = it;
        [Jc, gain, loss] = refresh_cache(Jc, gain, loss, cur, sat, J, cfg.qmin, lambda, qhi);

        % Step 6: is there spare total flow?
        if qT - sum(cur) > tolq
            [gbest, ji] = max(gain);
            if gbest > gmin                       % Step 7: raising some loop helps
                % Step 8: raise the best loop while it stays best / margin / gain remain
                others = gain;  others(ji) = -Inf;
                g2 = max(others);
                while true
                    step = min([lambda, qT - sum(cur), qhi - cur(ji)]);
                    if step <= 1e-9, break; end
                    cur(ji) = cur(ji) + step;
                    Jc(ji) = NaN;  gain(ji) = NaN;  loss(ji) = NaN;
                    [Jc, gain, loss] = refresh_cache(Jc, gain, loss, cur, sat, J, ...
                                                     cfg.qmin, lambda, qhi);
                    if gain(ji) <= g2 || gain(ji) <= gmin, break; end
                end
                continue                          % back to Step 6
            end
        end

        % Steps 9-10: raise one loop, lower another by the same amount
        [gbest, ji] = max(gain);
        lossm = loss;  lossm(ji) = Inf;
        [lbest, jd] = min(lossm);
        if ~isfinite(lbest) || (gbest - lbest) <= gmin
            info.exit = 'converged';
            break
        end
        step = min([lambda, qhi - cur(ji), cur(jd) - cfg.qmin]);
        if step <= 1e-9
            info.exit = 'converged';
            break
        end
        % accept the swap only if the REAL total cost drops (prevents ping-pong)
        dJ = (J(ji, cur(ji) + step) - Jc(ji)) + (J(jd, cur(jd) - step) - Jc(jd));
        if ~(dJ < 0)
            info.exit = 'converged';
            break
        end
        cur(ji) = cur(ji) + step;
        cur(jd) = cur(jd) - step;
        Jc([ji jd]) = NaN;  gain([ji jd]) = NaN;  loss([ji jd]) = NaN;
    end

    % ---- Step 11: apply ----
    q = min(max(cur, cfg.qmin), cfg.qmax);
    if sum(q) > qT + 1e-6
        warning('coordinator:TotalFlow', 'Total flow %.4f exceeds qTmax %.4f', sum(q), qT);
    end
    info = finish_info(info, t_all);
end


% =====================================================================
function [q, S] = solve_all(solver, ls, dni, s, cfg, cap_eff)
% Call the local solver for every loop with flow cap cap_eff.
    Nl = numel(ls);
    q  = zeros(Nl, 1);
    S  = cell(Nl, 1);
    for j = 1:Nl
        args = {ls{j}, dni{j}, s.q_prev(j), s.Tin, s.Tout, cfg.Ta, cap_eff, cfg};
        try
            [qj, sj] = solver(args{:});
        catch ME
            % a solver that only returns the first move (one output)
            if ~isempty(strfind(ME.identifier, 'TooManyOutputs')) || ...
               ~isempty(strfind(ME.message, 'many outputs'))
                qj = solver(args{:});
                sj = qj;
            else
                rethrow(ME);
            end
        end
        if isscalar(sj), sj = qj * ones(1, cfg.Nu); end
        q(j) = qj;
        S{j} = sj(:).';
    end
end


function J = loop_cost(j, first, S, ls, dni, s, cfg)
% Local cost of loop j when its first planned flow is replaced by "first".
    seq    = S{j};
    seq(1) = first;
    J = cost_local(seq, ls{j}, dni{j}, s.q_prev(j), s.Tin, s.Tout, cfg.Ta, cfg);
    if ~isfinite(J), J = Inf; end
end


function [Jc, gain, loss] = refresh_cache(Jc, gain, loss, cur, sat, J, qmin, lambda, qhi)
% Recompute only the stale (NaN) entries.
%   gain(j) = (J(q) - J(q+d)) / d   > 0 means raising loop j lowers its cost (saturated loops only)
%   loss(j) = (J(q-d) - J(q)) / d   = cost increase from lowering loop j
% Both are per l/s, with d = lambda clipped at the flow limits, so they are comparable
% even when a loop is close to qmin or qhi.
    for j = 1:numel(cur)
        if isnan(Jc(j)), Jc(j) = J(j, cur(j)); end
        if isnan(gain(j))
            up = min(cur(j) + lambda, qhi);
            if sat(j) && (up - cur(j)) > 1e-9
                gain(j) = (Jc(j) - J(j, up)) / (up - cur(j));
            else
                gain(j) = -Inf;
            end
            if isnan(gain(j)), gain(j) = -Inf; end
        end
        if isnan(loss(j))
            dn = max(cur(j) - lambda, qmin);
            if (cur(j) - dn) > 1e-9
                loss(j) = (J(j, dn) - Jc(j)) / (cur(j) - dn);
            else
                loss(j) = Inf;
            end
            if isnan(loss(j)), loss(j) = Inf; end
        end
    end
end


function info = finish_info(info, t_all)
    info.time.total = toc(t_all);
    info.time.local = info.time.step2 + info.time.step4;
    info.time.coord = info.time.total - info.time.local;
end


function v = getopt(opts, name, default)
    if isfield(opts, name), v = opts.(name); else, v = default; end
end
