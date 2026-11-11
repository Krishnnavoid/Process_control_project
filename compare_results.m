function compare_results(refKey)
% COMPARE_RESULTS  Loads the closed-loop logs in logs/, prints a metrics table and
% saves comparison figures to figures/ (PNG) and logs/metrics_summary.csv.
%
%   compare_results            % reference controller = 'noValves'
%   compare_results('PI')      % use another controller as the 0 % baseline
%
% Place this file in the repo root (next to config.m) and run it from MATLAB.
%
% Expected logs (all use the standard log struct from model/run_closed_loop.m):
%   noValves : logs/log_noValves.mat   (or checkpoint_noValves.mat if complete)
%   PI       : logs/log_PI.mat
%   local    : logs/log_local.mat      (log.name must be 'local')
%   dmpc     : logs/log_dmpc.mat       (log.name must be 'dmpc')
%   central  : logs/log_central.mat    (or checkpoint_central.mat if complete)
%
% A log is skipped (with a message) if it is missing, incomplete (zeros after
% the last finished step) or if log.name does not match the controller, so a
% copy of one controller's log saved under another name is never plotted.
%
% Figures written to figures/:
%   fig1_power.png          thermal power vs time + difference to reference
%   fig2_flow_temp_<key>.png per-loop flow and outlet temperature, one per controller
%   fig3_field_temps.png    field inlet / outlet temperature
%   fig4_improvement.png    mean power vs reference (overall and cloud window)
%   fig5_comp_time.png      computation time per controller step
%   fig6_flow_alloc.png     mean flow per loop and total flow vs qTmax

    if nargin < 1, refKey = 'noValves'; end

    proj_root = fileparts(mfilename('fullpath'));
    addpath(genpath(proj_root));
    cfg = config();
    scn = make_scn(cfg);
    logs_dir = fullfile(proj_root, 'logs');
    fig_dir  = fullfile(proj_root, 'figures');
    if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

    %% ---- Controller registry ------------------------------------------------
    % key, label, candidate files (in priority order), accepted log.name values, colour, line style
    reg = { ...
        'noValves', 'No-valves MPC',  {'log_noValves.mat', 'checkpoint_noValves.mat'}, {'noValves'},                   [0.50 0.50 0.50], '-'; ...
        'PI',       'PI + feedforward', {'log_PI.mat'},                                  {'PI'},                         [0.49 0.18 0.56], '-'; ...
        'local',    'Local MPC',      {'log_local.mat'},                               {'local', 'Local', 'ctrl_local'}, [0.47 0.67 0.19], '-'; ...
        'dmpc',     'Distributed MPC', {'log_dmpc.mat'},                               {'dmpc'},                         [0.85 0.33 0.10], '-'; ...
        'central',  'Centralized MPC', {'log_central.mat', 'checkpoint_central.mat'},  {'central'},                      [0.00 0.45 0.74], '--' };

    C = struct('key', {}, 'label', {}, 'log', {}, 'color', {}, 'style', {});
    fprintf('\n=== Loading logs ===\n');
    for i = 1:size(reg, 1)
        [L, status] = load_one(logs_dir, reg{i, 3}, reg{i, 4});
        fprintf('  %-18s %s\n', reg{i, 2}, status);
        if ~isempty(L)
            C(end + 1) = struct('key', reg{i, 1}, 'label', reg{i, 2}, 'log', L, ...
                                'color', reg{i, 5}, 'style', reg{i, 6}); %#ok<AGROW>
        end
    end
    if isempty(C)
        error('compare_results:noLogs', 'No usable logs found in %s', logs_dir);
    end

    ref_idx = find(strcmp({C.key}, refKey), 1);
    if isempty(ref_idx)
        warning('compare_results:noRef', ...
            'Reference controller "%s" not available: improvements vs reference will be NaN.', refKey);
    end

    cl0 = scn.cloud.t0;  cl1 = scn.cloud.t1;      % cloud window [min]

    %% ---- Metrics -----------------------------------------------------------
    n = numel(C);
    M = struct('meanW', nan(n,1), 'cloudW', nan(n,1), 'impr', nan(n,1), 'imprCloud', nan(n,1), ...
               'tcomp', nan(n,1), 'aaci', nan(n,1), 'mscv', nan(n,1), 'qTpeak', nan(n,1), ...
               'qTviol', nan(n,1));
    for i = 1:n
        L = C(i).log;
        t = L.t(:).';
        inCloud = t >= cl0 & t <= cl1;
        M.meanW(i)  = mean(L.W_kW);
        M.cloudW(i) = mean(L.W_kW(inCloud));

        tt = L.time.total(:);
        if any(tt > 0), M.tcomp(i) = mean(tt); end

        M.aaci(i) = mean(sum(abs(diff(L.q, 1, 2)), 2));            % l/s per loop
        v = max(max(cfg.Tmin - L.Tout_loop, L.Tout_loop - cfg.Tmax), 0);
        M.mscv(i) = mean(v(:).^2);                                  % degC^2
        qT = sum(L.q, 1);
        M.qTpeak(i) = max(qT);
        M.qTviol(i) = sum(qT > cfg.qTmax + 1e-3);
    end
    if ~isempty(ref_idx)
        M.impr      = 100 * (M.meanW  - M.meanW(ref_idx))  / M.meanW(ref_idx);
        M.imprCloud = 100 * (M.cloudW - M.cloudW(ref_idx)) / M.cloudW(ref_idx);
    end

    fprintf('\n=== Metrics (reference: %s) ===\n', refKey);
    fprintf('%-18s %10s %8s %10s %8s %9s %8s %9s %8s %6s\n', 'Controller', 'Mean[kW]', 'vs ref%', ...
            'Cloud[kW]', 'vs ref%', 'Tcomp[s]', 'AACI', 'MSCV', 'qTpeak', 'qTviol');
    for i = 1:n
        fprintf('%-18s %10.2f %8.2f %10.2f %8.2f %9.2f %8.3f %9.4f %8.3f %6d\n', C(i).label, ...
                M.meanW(i), M.impr(i), M.cloudW(i), M.imprCloud(i), M.tcomp(i), ...
                M.aaci(i), M.mscv(i), M.qTpeak(i), M.qTviol(i));
    end
    fprintf('(AACI = accumulated |dq| per loop [l/s]; MSCV = mean squared outlet-temperature violation [degC^2];\n');
    fprintf(' qTviol = controller steps with total flow above qTmax = %.2f l/s; Tcomp NaN = not timed.)\n\n', cfg.qTmax);

    tbl = table({C.label}', M.meanW, M.impr, M.cloudW, M.imprCloud, M.tcomp, M.aaci, M.mscv, M.qTpeak, M.qTviol, ...
        'VariableNames', {'Controller', 'MeanPower_kW', 'VsRef_pct', 'CloudPower_kW', 'CloudVsRef_pct', ...
                          'CompTime_s', 'AACI_lps', 'MSCV_degC2', 'PeakTotalFlow_lps', 'StepsOverQTmax'});
    writetable(tbl, fullfile(logs_dir, 'metrics_summary.csv'));

    %% ---- Figure 1: thermal power --------------------------------------------
    f1 = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 900 680]);
    ax = subplot(2, 1, 1);  hold on;  grid on;
    for i = 1:n
        plot(C(i).log.t, C(i).log.W_kW, C(i).style, 'Color', C(i).color, 'LineWidth', 1.6, ...
             'DisplayName', C(i).label);
    end
    shade_cloud(ax, cl0, cl1);
    ylabel('Net thermal power [kW]');
    title('Net thermal power of the collector field');
    legend('Location', 'southwest');

    ax = subplot(2, 1, 2);  hold on;  grid on;
    if ~isempty(ref_idx)
        Wref = C(ref_idx).log.W_kW;
        for i = 1:n
            if i == ref_idx, continue; end
            plot(C(i).log.t, C(i).log.W_kW - Wref, C(i).style, 'Color', C(i).color, 'LineWidth', 1.6);
        end
        ylabel(sprintf('\\DeltaW vs %s [kW]', C(ref_idx).label));
        plot(xlim, [0 0], 'k:', 'HandleVisibility', 'off');
        shade_cloud(ax, cl0, cl1);
    else
        text(0.5, 0.5, 'Reference controller not available', 'Units', 'normalized', ...
             'HorizontalAlignment', 'center');
    end
    xlabel('Time [min]');
    save_fig(f1, fig_dir, 'fig1_power');

    %% ---- Figure 2: per-loop flow and outlet temperature (one figure each) ---
    loopLbl = arrayfun(@(j) sprintf('Loop %d', j), 1:cfg.Nl, 'UniformOutput', false);
    cmap = lines(cfg.Nl);
    for i = 1:n
        L = C(i).log;
        f2 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 60 900 650]);
        ax = subplot(2, 1, 1);  hold on;  grid on;
        for j = 1:cfg.Nl
            stairs(L.t, L.q(j, :), 'Color', cmap(j, :), 'LineWidth', 1.1);
        end
        plot([L.t(1) L.t(end)], [cfg.qmin cfg.qmin], 'k--', 'HandleVisibility', 'off');
        plot([L.t(1) L.t(end)], [cfg.qTmax/cfg.Nl cfg.qTmax/cfg.Nl], 'k:', 'HandleVisibility', 'off');
        shade_cloud(ax, cl0, cl1);
        ylabel('Flow [l/s]');
        title(sprintf('%s: flow in each loop (dotted = qTmax/Nl = %.2f l/s)', C(i).label, cfg.qTmax/cfg.Nl));
        legend(loopLbl, 'Location', 'eastoutside');

        ax = subplot(2, 1, 2);  hold on;  grid on;
        for j = 1:cfg.Nl
            plot(L.t, L.Tout_loop(j, :), 'Color', cmap(j, :), 'LineWidth', 1.1);
        end
        plot([L.t(1) L.t(end)], [cfg.Tmin cfg.Tmin], 'k--', 'HandleVisibility', 'off');
        plot([L.t(1) L.t(end)], [cfg.Tmax cfg.Tmax], 'k--', 'HandleVisibility', 'off');
        shade_cloud(ax, cl0, cl1);
        xlabel('Time [min]');  ylabel('Outlet temperature [^{\circ}C]');
        title('Outlet temperature of each loop');
        legend(loopLbl, 'Location', 'eastoutside');
        save_fig(f2, fig_dir, ['fig2_flow_temp_' C(i).key]);
    end

    %% ---- Figure 3: field inlet / outlet temperature ----------------------------
    f3 = figure('Visible', 'off', 'Color', 'w', 'Position', [120 120 900 560]);
    ax = subplot(2, 1, 1);  hold on;  grid on;
    for i = 1:n
        plot(C(i).log.t, C(i).log.Tout_field, C(i).style, 'Color', C(i).color, 'LineWidth', 1.5, ...
             'DisplayName', C(i).label);
    end
    plot([1 numel(C(1).log.t)], [cfg.Tmin cfg.Tmin], 'k--', 'HandleVisibility', 'off');
    shade_cloud(ax, cl0, cl1);
    ylabel('Field outlet T [^{\circ}C]');  title('Field outlet temperature');
    legend('Location', 'northeast');
    ax = subplot(2, 1, 2);  hold on;  grid on;
    for i = 1:n
        plot(C(i).log.t, C(i).log.Tin, C(i).style, 'Color', C(i).color, 'LineWidth', 1.5);
    end
    shade_cloud(ax, cl0, cl1);
    xlabel('Time [min]');  ylabel('Field inlet T [^{\circ}C]');  title('Field inlet temperature');
    save_fig(f3, fig_dir, 'fig3_field_temps');

    %% ---- Figure 4: improvement vs reference ------------------------------------
    if ~isempty(ref_idx)
        f4 = figure('Visible', 'off', 'Color', 'w', 'Position', [140 140 760 480]);
        B = [M.impr, M.imprCloud];
        h = bar(B, 'grouped');
        h(1).FaceColor = [0.20 0.45 0.75];  h(2).FaceColor = [0.85 0.55 0.15];
        grid on;
        set(gca, 'XTick', 1:n, 'XTickLabel', {C.label});
        xtickangle(20);
        ylabel(sprintf('Mean power vs %s [%%]', C(ref_idx).label));
        title('Improvement in mean thermal power');
        legend({'Whole 2 h run', sprintf('Cloud window (min %g-%g)', cl0, cl1)}, 'Location', 'best');
        for i = 1:n
            text(i - 0.15, B(i, 1), sprintf('%.2f', B(i, 1)), 'HorizontalAlignment', 'center', ...
                 'VerticalAlignment', vtxt(B(i, 1)), 'FontSize', 8);
            text(i + 0.15, B(i, 2), sprintf('%.2f', B(i, 2)), 'HorizontalAlignment', 'center', ...
                 'VerticalAlignment', vtxt(B(i, 2)), 'FontSize', 8);
        end
        save_fig(f4, fig_dir, 'fig4_improvement');
    end

    %% ---- Figure 5: computation time -------------------------------------------
    timed = find(~isnan(M.tcomp));
    if ~isempty(timed)
        f5 = figure('Visible', 'off', 'Color', 'w', 'Position', [160 160 820 480]);
        ax = axes(f5);  hold on;  grid on;
        for k = timed(:).'
            tt = C(k).log.time.total(:).';
            tt(tt <= 0) = NaN;
            semilogy(C(k).log.t, tt, C(k).style, 'Color', C(k).color, 'LineWidth', 1.4, ...
                     'DisplayName', sprintf('%s (mean %.1f s)', C(k).label, M.tcomp(k)));
        end
        set(ax, 'YScale', 'log');
        shade_cloud(ax, cl0, cl1);
        xlabel('Time [min]');  ylabel('Computation time per controller step [s]');
        title('Computation time (depends on your machine: compare ratios, not absolute values)');
        legend('Location', 'best');
        save_fig(f5, fig_dir, 'fig5_comp_time');
    end

    %% ---- Figure 6: flow allocation ---------------------------------------------
    f6 = figure('Visible', 'off', 'Color', 'w', 'Position', [180 100 900 620]);
    subplot(2, 1, 1);  hold on;  grid on;
    Q = zeros(cfg.Nl, n);
    for i = 1:n, Q(:, i) = mean(C(i).log.q, 2); end
    h = bar(Q, 'grouped');
    for i = 1:n, h(i).FaceColor = C(i).color; end
    xlabel('Loop');  ylabel('Mean flow [l/s]');
    title('Time-averaged flow per loop (valves re-allocate flow towards well-irradiated loops)');
    legend({C.label}, 'Location', 'eastoutside');
    ax = subplot(2, 1, 2);  hold on;  grid on;
    for i = 1:n
        plot(C(i).log.t, sum(C(i).log.q, 1), C(i).style, 'Color', C(i).color, 'LineWidth', 1.5);
    end
    plot([1 numel(C(1).log.t)], [cfg.qTmax cfg.qTmax], 'k--', 'HandleVisibility', 'off');
    shade_cloud(ax, cl0, cl1);
    xlabel('Time [min]');  ylabel('Total flow [l/s]');
    title(sprintf('Total flow (dashed = qTmax = %.2f l/s)', cfg.qTmax));
    save_fig(f6, fig_dir, 'fig6_flow_alloc');

    fprintf('Figures saved in %s\nMetrics saved in %s\n', fig_dir, fullfile(logs_dir, 'metrics_summary.csv'));
end


% =========================================================================
function [L, status] = load_one(logs_dir, files, expected)
% Return the first complete log whose log.name is accepted; otherwise explain why not.
    L = [];  status = 'missing';
    for i = 1:numel(files)
        f = fullfile(logs_dir, files{i});
        if ~exist(f, 'file'), continue; end
        S = load(f);
        if ~isfield(S, 'log'), continue; end
        lg = S.log;
        if ~any(strcmpi(lg.name, expected))
            status = sprintf('SKIPPED: %s holds a "%s" log', files{i}, lg.name);
            continue;
        end
        W = lg.W_kW(:).';
        z = find(W <= 0, 1, 'first');
        if ~isempty(z)
            status = sprintf('SKIPPED: %s incomplete (%d of %d steps finished)', files{i}, z - 1, numel(W));
            continue;
        end
        L = lg;
        status = sprintf('ok (%s)', files{i});
        return
    end
end

function shade_cloud(ax, t0, t1)
% Grey band for the cloud window, drawn behind the lines and hidden from legends.
    yl = ylim(ax);
    p = patch(ax, [t0 t1 t1 t0], [yl(1) yl(1) yl(2) yl(2)], [0.85 0.85 0.85], ...
              'EdgeColor', 'none', 'FaceAlpha', 0.6, 'HandleVisibility', 'off');
    ylim(ax, yl);
    ch = get(ax, 'Children');
    ch = ch(ch ~= p);
    set(ax, 'Children', [ch; p]);
end

function v = vtxt(x)
    if x >= 0, v = 'bottom'; else, v = 'top'; end
end

function save_fig(f, fig_dir, name)
    print(f, fullfile(fig_dir, [name '.png']), '-dpng', '-r200');
    close(f);
end

