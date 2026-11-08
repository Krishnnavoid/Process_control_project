function build_baseline_pi(Kp, Ki, Tref_val)
% BUILD_BASELINE_PI Programmatically constructs and saves 'simulink/baseline_pi.slx'.
% Implements the exact architecture from Section 10.3 of the Part 1 Step-by-Step Guide,
% including the Unit Delay (0.5 s) on the plant input (Step 11) to guarantee zero algebraic loops.
%
% Inputs (optional):
%   Kp       : Proportional gain [l/s/degC]
%   Ki       : Integral gain [l/s/(degC*s)]
%   Tref_val : Outlet temperature setpoint [degC] (default: 260 degC)

    if nargin < 1, Kp = 0.01548; end
    if nargin < 2, Ki = 6.3e-5; end
    if nargin < 3, Tref_val = 260; end

    cfg   = config();
    q_cap = cfg.qTmax / cfg.Nl; % 0.65 l/s per-loop cap

    mdl = 'baseline_pi';
    if bdIsLoaded(mdl), close_system(mdl, 0); end

    % Remove any existing baseline_pi files to prevent MATLAB shadowing warnings
    if ~exist('simulink', 'dir'), mkdir('simulink'); end
    target_slx = fullfile('simulink', 'baseline_pi.slx');
    if exist(target_slx, 'file'), delete(target_slx); end
    if exist('baseline_pi.slx', 'file') && ~strcmp(pwd, fullfile(pwd, 'simulink'))
        delete('baseline_pi.slx');
    end

    new_system(mdl);

    %% 1. Configure Solver & Diagnostics (Fixed-step Discrete, 0.5 s step, 7200 s stop)
    set_param(mdl, ...
        'SolverType', 'Fixed-step', ...
        'Solver', 'FixedStepDiscrete', ...
        'FixedStep', '0.5', ...
        'StopTime', '7200', ...
        'SampleTimeConstraint', 'Unconstrained');

    %% 2. Add Core Blocks
    % 2.1 FieldPlant MATLAB System Block (0.5 s)
    add_block('simulink/User-Defined Functions/MATLAB System', [mdl '/FieldPlant'], ...
        'Position', [720, 120, 880, 320], ...
        'System', 'FieldPlant');

    % 2.2 Setpoint Constant Tref (sampled at 5 s for PI, bridged to 0.5 s for ff)
    add_block('simulink/Sources/Constant', [mdl '/Tref'], ...
        'Position', [40, 180, 90, 210], ...
        'Value', num2str(Tref_val), ...
        'SampleTime', '5');

    % 2.3 Zero-Order Hold / Rate Transition on Tout (0.5 s -> 5 s)
    add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_Tout'], ...
        'Position', [120, 115, 160, 145], ...
        'OutPortSampleTime', '5');

    % 2.4 Error Sum Block: e = Tout - Tref (Reverse-acting process: +e requires +flow)
    add_block('simulink/Math Operations/Sum', [mdl '/Sum_Error'], ...
        'Position', [200, 120, 230, 150], ...
        'Inputs', '+-');

    % 2.5 Discrete PID Controller (PI form, 5 s sample time, Clamping anti-windup, saturation off here)
    add_block('simulink/Discrete/Discrete PID Controller', [mdl '/PI_Controller'], ...
        'Position', [270, 110, 330, 160], ...
        'Controller', 'PI', ...
        'SampleTime', '5', ...
        'P', num2str(Kp, '%.8g'), ...
        'I', num2str(Ki, '%.8g'), ...
        'AntiWindupMode', 'clamping', ...
        'LimitOutput', 'off');

    % 2.6 Rate Transition on PI Output (5 s -> 0.5 s)
    add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_uPI'], ...
        'Position', [370, 120, 410, 150], ...
        'OutPortSampleTime', '0.5');

    % 2.7 Rate Transition on Tref for Feedforward (5 s -> 0.5 s)
    add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_Tref_ff'], ...
        'Position', [120, 310, 160, 340], ...
        'OutPortSampleTime', '0.5');

    % 2.8 Feedforward Block 'ff' (Built as a native Subsystem)
    % Exact equations from Section 10.3 Step 4:
    %   rho   = 903 - 0.672*240;
    %   Cf    = 1820 + 3.478*240;
    %   q_tot = 0.9 * sum(Pabs) / (rho * Cf * max(Tref - Tin, 10));
    %   q_ff  = 1e3 * q_tot / 10;
    ff_sub = [mdl '/ff'];
    add_block('simulink/Ports & Subsystems/Subsystem', ff_sub, ...
        'Position', [250, 250, 390, 340]);
    Simulink.SubSystem.deleteContents(ff_sub);

    add_block('simulink/Sources/In1', [ff_sub '/Pabs'], 'Position', [30, 40, 60, 54]);
    add_block('simulink/Sources/In1', [ff_sub '/Tin'],  'Position', [30, 110, 60, 124]);
    add_block('simulink/Sources/In1', [ff_sub '/Tref'], 'Position', [30, 160, 60, 174]);
    add_block('simulink/Sinks/Out1',  [ff_sub '/q_ff'], 'Position', [480, 75, 510, 89]);

    add_block('simulink/Math Operations/Sum of Elements', [ff_sub '/Sum_Pabs'], ...
        'Position', [110, 32, 140, 62]);
    add_block('simulink/Math Operations/Sum', [ff_sub '/dT_diff'], ...
        'Position', [110, 125, 140, 155], ...
        'Inputs', '-+'); % Tref - Tin
    add_block('simulink/Sources/Constant', [ff_sub '/Min_dT'], ...
        'Position', [110, 185, 140, 215], ...
        'Value', '10');
    add_block('simulink/Math Operations/MinMax', [ff_sub '/Max_dT'], ...
        'Position', [190, 130, 225, 170], ...
        'Function', 'max', ...
        'Inputs', '2');
    add_block('simulink/Math Operations/Divide', [ff_sub '/Divide'], ...
        'Position', [280, 60, 310, 100], ...
        'Inputs', '*/');

    rho_240 = 903 - 0.672 * 240;
    Cf_240  = 1820 + 3.478 * 240;
    ff_gain = (1e3 / 10) * 0.9 / (rho_240 * Cf_240);
    add_block('simulink/Math Operations/Gain', [ff_sub '/Scale_qff'], ...
        'Position', [360, 65, 430, 95], ...
        'Gain', num2str(ff_gain, '%.12g'));

    add_line(ff_sub, 'Pabs/1',       'Sum_Pabs/1',  'autorouting', 'on');
    add_line(ff_sub, 'Tin/1',        'dT_diff/1',   'autorouting', 'on');
    add_line(ff_sub, 'Tref/1',       'dT_diff/2',   'autorouting', 'on');
    add_line(ff_sub, 'dT_diff/1',    'Max_dT/1',    'autorouting', 'on');
    add_line(ff_sub, 'Min_dT/1',     'Max_dT/2',    'autorouting', 'on');
    add_line(ff_sub, 'Sum_Pabs/1',   'Divide/1',    'autorouting', 'on');
    add_line(ff_sub, 'Max_dT/1',     'Divide/2',    'autorouting', 'on');
    add_line(ff_sub, 'Divide/1',     'Scale_qff/1', 'autorouting', 'on');
    add_line(ff_sub, 'Scale_qff/1',  'q_ff/1',      'autorouting', 'on');

    % 2.9 Total Flow Sum (q_raw = u_PI + q_ff)
    add_block('simulink/Math Operations/Sum', [mdl '/Sum_Flow'], ...
        'Position', [440, 195, 470, 225], ...
        'Inputs', '++');

    % 2.10 Saturation Block [0.2, 0.65] l/s
    add_block('simulink/Discontinuities/Saturation', [mdl '/Saturation'], ...
        'Position', [500, 195, 530, 225], ...
        'LowerLimit', num2str(cfg.qmin), ...
        'UpperLimit', num2str(q_cap));

    % 2.11 Gain Block (10x1 vector of ones) to expand scalar flow to [10 x 1]
    add_block('simulink/Math Operations/Gain', [mdl '/Expand_10x1'], ...
        'Position', [560, 195, 600, 225], ...
        'Gain', 'ones(10,1)', ...
        'Multiplication', 'Element-wise(K.*u)');

    % 2.12 Unit Delay (0.5 s) on Plant Input (Step 11 of Section 10.3 to break algebraic loops)
    add_block('simulink/Discrete/Unit Delay', [mdl '/Unit_Delay_PlantIn'], ...
        'Position', [640, 195, 675, 225], ...
        'InitialCondition', '0.5*ones(10,1)', ...
        'SampleTime', '0.5');

    %% 3. Add To Workspace Blocks (Structure With Time)
    ws_names = {'o_ToutLoop', 'o_Tout', 'o_Tin', 'o_W', 'o_q'};
    for idx = 1:5
        add_block('simulink/Sinks/To Workspace', [mdl '/' ws_names{idx}], ...
            'Position', [940, 90 + 50*idx, 1020, 115 + 50*idx], ...
            'VariableName', ws_names{idx}, ...
            'SaveFormat', 'Structure With Time', ...
            'SampleTime', '0.5');
    end

    %% 4. Wire Top-Level Connections
    % Error calculation & PI branch
    add_line(mdl, 'FieldPlant/2',         'RT_Tout/1',            'autorouting', 'on');
    add_line(mdl, 'RT_Tout/1',            'Sum_Error/1',          'autorouting', 'on');
    add_line(mdl, 'Tref/1',               'Sum_Error/2',          'autorouting', 'on');
    add_line(mdl, 'Sum_Error/1',          'PI_Controller/1',      'autorouting', 'on');
    add_line(mdl, 'PI_Controller/1',      'RT_uPI/1',             'autorouting', 'on');
    add_line(mdl, 'RT_uPI/1',             'Sum_Flow/1',           'autorouting', 'on');

    % Feedforward branch (inputs: Pabs, Tin, Tref)
    add_line(mdl, 'FieldPlant/5',         'ff/1',                 'autorouting', 'on');
    add_line(mdl, 'FieldPlant/3',         'ff/2',                 'autorouting', 'on');
    add_line(mdl, 'Tref/1',               'RT_Tref_ff/1',         'autorouting', 'on');
    add_line(mdl, 'RT_Tref_ff/1',         'ff/3',                 'autorouting', 'on');
    add_line(mdl, 'ff/1',                 'Sum_Flow/2',           'autorouting', 'on');

    % Saturation, Expand, Unit Delay & Plant Input
    add_line(mdl, 'Sum_Flow/1',           'Saturation/1',         'autorouting', 'on');
    add_line(mdl, 'Saturation/1',         'Expand_10x1/1',        'autorouting', 'on');
    add_line(mdl, 'Expand_10x1/1',        'Unit_Delay_PlantIn/1', 'autorouting', 'on');
    add_line(mdl, 'Unit_Delay_PlantIn/1', 'FieldPlant/1',         'autorouting', 'on');

    % Workspace Loggers
    add_line(mdl, 'FieldPlant/1',         'o_ToutLoop/1',         'autorouting', 'on');
    add_line(mdl, 'FieldPlant/2',         'o_Tout/1',             'autorouting', 'on');
    add_line(mdl, 'FieldPlant/3',         'o_Tin/1',              'autorouting', 'on');
    add_line(mdl, 'FieldPlant/4',         'o_W/1',                'autorouting', 'on');
    add_line(mdl, 'Expand_10x1/1',        'o_q/1',                'autorouting', 'on');

    %% 5. Save Simulink Model
    save_system(mdl, target_slx);
    close_system(mdl, 0);
end