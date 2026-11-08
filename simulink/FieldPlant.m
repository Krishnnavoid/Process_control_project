classdef FieldPlant < matlab.System
% FIELDPLANT Simulink MATLAB System block encapsulating the 10-loop solar field (0.5 s step).
% Uses separate outputImpl and updateImpl methods (non-direct feedthrough) so that
% plant outputs depend strictly on the state and never create a Simulink algebraic loop.

    properties (Access = private)
        s      % Internal plant state struct
        cfg    % Configuration struct
        scn    % Disturbance scenario struct
        k      % Discrete step counter
        W_last % Last computed net thermal power [kW]
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.cfg    = config();
            obj.scn    = make_scn(obj.cfg);
            obj.s      = initial_state(obj.cfg);
            obj.k      = 0;
            obj.W_last = 1150; % Nominal initial net thermal power [kW]
        end

        function resetImpl(obj)
            obj.s      = initial_state(obj.cfg);
            obj.k      = 0;
            % Compute initial thermal power at t = 0 from initial state
            q0 = obj.s.q_prev(:) * 1e-3;
            [rN, CN] = fluid_props(obj.s.Tf(:, end), q0, obj.cfg.Ta);
            [r1, C1] = fluid_props(obj.s.Tin, q0, obj.cfg.Ta);
            obj.W_last = sum(rN .* CN .* q0 .* obj.s.Tf(:, end) - r1 .* C1 .* q0 .* obj.s.Tin) / 1e3;
        end

        function [ToutLoop, Tout, Tin, W, Pabs] = outputImpl(obj, ~)
            % OUTPUTIMPL Computes block outputs strictly from current internal state obj.s
            % and time counter obj.k (Zero direct feedthrough from input q_lps).
            D = get_dni(obj.scn, obj.k * obj.cfg.dT, obj.cfg);

            ToutLoop = obj.s.Tf(:, end); % [10 x 1] [degC]
            Tout     = obj.s.Tout;       % [1 x 1]  [degC]
            Tin      = obj.s.Tin;        % [1 x 1]  [degC]
            W        = obj.W_last;       % [1 x 1]  [kW]
            Pabs     = sum(D, 2);        % [10 x 1] Absorbed solar power per loop [W] (dl = 1 m)
        end

        function updateImpl(obj, q_lps)
            % UPDATEIMPL Advances the plant state by one 0.5 s step using input q_lps [l/s].
            D = get_dni(obj.scn, obj.k * obj.cfg.dT, obj.cfg);
            q_safe = min(max(q_lps(:), obj.cfg.qmin), obj.cfg.qmax);
            [obj.s, obj.W_last] = plant_step(obj.s, q_safe * 1e-3, D, obj.cfg);
            obj.k = obj.k + 1;
        end

        function flag = isInputDirectFeedthroughImpl(~, ~)
            % Explicitly inform Simulink that outputImpl does not depend on input q_lps
            flag = false;
        end

        function num = getNumInputsImpl(~)
            num = 1;
        end

        function num = getNumOutputsImpl(~)
            num = 5;
        end

        function st = getSampleTimeImpl(obj)
            st = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', 0.5);
        end

        function [sz1, sz2, sz3, sz4, sz5] = getOutputSizeImpl(~)
            sz1 = [10 1]; % ToutLoop
            sz2 = [1 1];  % Tout
            sz3 = [1 1];  % Tin
            sz4 = [1 1];  % W
            sz5 = [10 1]; % Pabs
        end

        function [dt1, dt2, dt3, dt4, dt5] = getOutputDataTypeImpl(~)
            dt1 = 'double';
            dt2 = 'double';
            dt3 = 'double';
            dt4 = 'double';
            dt5 = 'double';
        end

        function [c1, c2, c3, c4, c5] = isOutputComplexImpl(~)
            c1 = false;
            c2 = false;
            c3 = false;
            c4 = false;
            c5 = false;
        end

        function [f1, f2, f3, f4, f5] = isOutputFixedSizeImpl(~)
            f1 = true;
            f2 = true;
            f3 = true;
            f4 = true;
            f5 = true;
        end
    end

    methods (Static, Access = protected)
        function s = getSimulateUsingImpl()
            s = 'Interpreted execution';
        end
    end
end