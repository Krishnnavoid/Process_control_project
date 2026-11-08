function [q, info] = dmpc_fmincon(s, scn, cfg)
% DMPC_FMINCON  Distributed MPC with fmincon local solvers.
% Same signature as every other controller:  [q, info] = ctrl(s, scn, cfg)
% The local solver is injected as a handle, so Part 3 can pass the network instead.
    [q, info] = coordinator(s, scn, cfg, @local_mpc_fmincon);
end
