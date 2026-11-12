function plot_dni_maps()
% PLOT_DNI_MAPS Generates spatial, temporal, and spatiotemporal maps of DNI
% and collector optical efficiency matching get_dni.m and make_scn.m.
% Saves PNG figures to figures/.

    proj_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(proj_root));
    cfg = config();
    scn = make_scn(cfg);

    fig_dir = fullfile(proj_root, 'figures');
    if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

    Nl = cfg.Nl;
    Ns = cfg.Ns;
    D0 = scn.D0;

    %% 1. Figure 1: Optical Cleanliness & Efficiency Map (Frejo Fig. 3)
    f1 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1000 500]);
    imagesc(1:Ns, 1:Nl, scn.eff);
    set(gca, 'YDir', 'normal');
    colormap(gca, 'hot');
    c = colorbar;
    c.Label.String = 'Optical Efficiency \eta_j(z) [-]';
    c.Label.FontSize = 11;
    caxis([0 1]);
    xlabel('Collector Segment [m] (Fluid flow from left 0 m to right 174 m)');
    ylabel('Collector Loop Number');
    title('ACUREX Collector Field: Optical Efficiency & Defocus Map \eta_j(z)', 'FontSize', 12);
    set(gca, 'YTick', 1:Nl, 'XTick', 0:20:Ns);
    grid on;

    save_fig(f1, fig_dir, 'matlab_dni_optical_efficiency');

    %% 2. Figure 2: Cloud Snapshots (t = 0, 65, 70, 75 min)
    f2 = figure('Visible', 'off', 'Color', 'w', 'Position', [120 80 1050 800]);
    times_min = [0, 65, 70, 75];
    titles = { ...
        't = 0 min: Clear Sky Baseline (Nominal 800 W/m + Soiling)', ...
        't = 65 min: Cloud Shadow Enters Field (Loops 3-9, Front advancing at 11 m/min)', ...
        't = 70 min: Peak Cloud Shadow Over Central Zone (80% DNI drop)', ...
        't = 75 min: Cloud Shadow Exiting Field Edge'};

    for i = 1:4
        subplot(4, 1, i);
        D_snap = get_dni(scn, times_min(i) * 60, cfg);
        imagesc(1:Ns, 1:Nl, D_snap);
        set(gca, 'YDir', 'normal');
        colormap(gca, 'hot');
        caxis([0 800]);
        title(titles{i}, 'FontWeight', 'bold', 'FontSize', 10);
        ylabel('Loop');
        set(gca, 'YTick', [1 3 5 7 9 10]);
        if i == 4
            xlabel('Collector Segment [m]');
        else
            set(gca, 'XTickLabel', []);
        end
    end
    c = colorbar('Position', [0.92 0.15 0.02 0.7]);
    c.Label.String = 'Effective DNI [W/m]';

    save_fig(f2, fig_dir, 'matlab_dni_cloud_snapshots');

    %% 3. Figure 3: Loop-Integrated Solar Thermal Power Input vs Time
    f3 = figure('Visible', 'off', 'Color', 'w', 'Position', [140 100 950 500]);
    t_min = 0:0.5:120;
    P_loop = zeros(Nl, numel(t_min));
    for k = 1:numel(t_min)
        D_k = get_dni(scn, t_min(k) * 60, cfg);
        P_loop(:, k) = sum(D_k, 2) / 1000; % kW
    end

    hold on; grid on;
    % Shade cloud
    patch([scn.cloud.t0 scn.cloud.t1 scn.cloud.t1 scn.cloud.t0], [30 30 125 125], ...
          [0.88 0.88 0.88], 'EdgeColor', 'none', 'DisplayName', 'Cloud Window (min 63-76)');
    cmap = lines(Nl);
    for j = 1:Nl
        plot(t_min, P_loop(j, :), 'LineWidth', 1.6, 'Color', cmap(j, :), ...
             'DisplayName', sprintf('Loop %d', j));
    end
    xlabel('Time [min]');
    ylabel('Incident Thermal Power [kW]');
    title('Total Incident Solar Heat Input per Collector Loop vs. Time', 'FontSize', 12);
    legend('Location', 'eastoutside');
    xlim([0 120]); ylim([30 125]);

    save_fig(f3, fig_dir, 'matlab_dni_loop_power_timeseries');

    fprintf('DNI maps generated and saved to %s\n', fig_dir);
end

function save_fig(f, fig_dir, name)
    print(f, fullfile(fig_dir, [name '.png']), '-dpng', '-r200');
    close(f);
end

