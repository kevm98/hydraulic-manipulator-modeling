% Example: build opts and run the tuner
opts.model       = 'Mulag_FME_500_with_cylinder';
opts.param_names = {'k_vc','k_vo','s_b0'};
opts.x0          = [1.0, 6.0, 1.0];
opts.lb          = [0.5, 4.0, 0.2];
opts.ub          = [2.0, 8.0, 5.0];
opts.mapFcn      = @my_ladrc_map;
opts.n_joints    = 5;

% Aliases for logsout names (printf-style; %d replaced by joint index)
opts.alias.ref  = {'joint%d_ref','q_ref%d','ref%d','qd%d'};
opts.alias.meas = {'joint%d_meas','q_meas%d','meas%d','qy%d'};
opts.alias.cmd  = {'u_cmd%d','joint%d_u','u%d'};

% Scenarios (you can set per-scenario StopTime here)
opts.scenarios = struct( ...
  'type', {'step','trap','sine'}, ...
  'amp',  {0.3, 0.5, 0.2}, ...
  'freq', {0,   0,   0.6}, ...
  'T_rise',{0.2, 0.6, 0.0}, ...
  'StopTime', {8, 8, 8} ...
);

% Goals
opts.goals(1) = struct('goal_IAE',0.15,'goal_SSE',0.010,'goal_OSC',0.05, ...
                       'goal_SAT',0.02,'goal_UNDER',0,'goal_NOEXC',0,'goal_Tset',3.0);
opts.goals(2) = struct('goal_IAE',0.18,'goal_SSE',0.012,'goal_OSC',0.06, ...
                       'goal_SAT',0.03,'goal_UNDER',0,'goal_NOEXC',0,'goal_Tset',4.0);
opts.goals(3) = struct('goal_IAE',0.20,'goal_SSE',0.015,'goal_OSC',0.08, ...
                       'goal_SAT',0.04,'goal_UNDER',0,'goal_NOEXC',0,'goal_Tset',4.0);

% Sim & solver convenience
opts.sim.maxStep = '5e-4'; opts.sim.RelTol = '1e-4'; opts.sim.AbsTol = '1e-5';
opts.useFastRestart = true;
opts.applyAndSave   = false;   % no side effects by default
opts.multistart.N   = 4;       % local restarts
opts.multistart.jitter = 0.2;

% Put your per-joint “bases” in the Model Workspace once (outside the tuner)
mw = get_param(opts.model,'ModelWorkspace');
if ~mw.evalin('exist(''vc_base'',''var'')'),  mw.assignin('vc_base',  [30;30;25;22;40]); end
if ~mw.evalin('exist(''b0_base0'',''var'')'), mw.assignin('b0_base0', [ 1; 1; 1; 1; 1]); end

% Run
results = ladrc_goal_tuner(opts);
disp(results.bestX)
