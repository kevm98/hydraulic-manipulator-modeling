function P = makeHydroParams()
% Build plant/controller params.
% Assumes Rtab (and optionally Qgrid) already exist in the BASE workspace
% (e.g., loaded by model InitFcn). Units: rad, rad/s, Pa, m, s

%% ---- Timing ----
P.Ts    = 0.005;    % controller sample [s]  <-- set to your actual Ts
P.tau_u = 0.010;    % spool 1st-order lag [s] (controller-side shaping)

%% ---- Supply (≈340 bar nominal) ----
P.Ps = 34e6;        % [Pa]
P.Pt = 0.1e6;       % [Pa]

%% ---- Fluid ----
P.beta_e = 1.7e9;   % [Pa]

%% ---- Valve (IL 4/3) ----
P.valve.x_open = 7.5e-3;   % [m]
P.valve.A_max  = 1.5e-4;   % [m^2]
P.valve.A_leak = 1e-10;    % [m^2]
P.valve.Cd     = 0.7;      % [-]
P.valve.u_max  = 7.5e-3;   % [m] hard clamp
P.valve.u0     = 0.0;      % [m] deadband half-width (set after ID)
P.valve.ubias  = 0.0;      % [m] null bias (set after ID)

%% ---- Cylinders (identical areas for all joints) ----
P.AA     = 0.00785 * ones(1,4);  % [m^2]
P.AB     = 0.00503 * ones(1,4);  % [m^2]
P.stroke = [1.2, 1.0, 0.6, 0.4]; % [m]

% Dead volumes [m^3]
P.V0A = [2.0e-4, 2.0e-4, 1.2e-4, 0.7e-4];
P.V0B = [1.5e-4, 1.5e-4, 0.85e-4, 0.4e-4];

% Initial chamber pressures [Pa]
P.PA0 = [1.0e5, 1.0e5, 2.0e5, 1.0e5];
P.PB0 = [1.0e5, 1.0e5, 8.0e5, 1.0e5];

%% ---- Trajectory limits (edit to your real limits) ----
P.qd_max   = [1,1,1,1];      % [rad/s]
P.qdd_max  = [5,5,5,5];      % [rad/s^2]
P.qddd_max = [50,50,50,50];  % [rad/s^3]

%% ---- Outer ISMC shaping from (ζ, ωn) ----
zeta = 0.8;                  % damping
wn   = 2*pi*3;               % ~3 Hz (hydraulic-friendly)
P.lambda = 2*zeta*wn * ones(1,4);
P.eta    = (wn^2)   * ones(1,4);

% Sliding gain and boundary layer
P.ks  = [80, 90, 90, 70];    % [Nm/rad] (tune per joint)
P.phi = [0.02,0.02,0.02,0.02]; % [rad/s]

%% ---- Friction feedforward (set after ID if available) ----
P.Fc = [0,0,0,0];            % [N] along cylinder axis
P.Fv = [0,0,0,0];            % [N·s/m]

%% ---- Inner-loop super-twisting gains (pressure error) ----
P.Phi = [3e5,3e5,3e5,3e5];           % [Pa]
P.k1  = [1.2e-7,1.2e-7,1.0e-7,1.2e-7]; % [m/Pa^0.5]
P.k2  = 10*P.k1;                      % [m/s] approx

%% ---- ESOs (position/velocity/disturbance & pressure dynamics) ----
P.eso.w0  = 2*pi*20 * ones(1,4);  % ~20 Hz
P.eso.Qn  = 0.02    * ones(1,4);
P.peso.w0 = 2*pi*40 * ones(1,4);  % ~40 Hz

%% ---- Pull Rtab / Qgrid from BASE workspace (InitFcn already loaded them) ----
try
    P.Rtab = evalin('base','Rtab');
catch
    error('makeHydroParams: Rtab not found in base workspace. Ensure InitFcn loaded it.');
end

if evalin('base','exist(''Qgrid'',''var'')')
    P.Qgrid = evalin('base','Qgrid');  % optional, preferred for interpolation
else
    P.Qgrid = [];
end

end
