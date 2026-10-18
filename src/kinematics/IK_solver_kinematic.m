%% ========================================================================
%  MULAG FME500 – CLOSED ARC + DIAMETER (A→B arc, B→A straight)
%  SMOOTH IK VERSION + FIXED Z-UP ORIENTATION
%
%  - Joint 1 spans: -1.57 -> +0.6578 rad along the ARC
%  - Joint 2: soft corridor [-8.389, -6.6] rad (radians!), intersected
%    with URDF limits; if infeasible, fall back to URDF limits.
%  - EE path: perfect semicircle A→B (XY plane at Z = homeZ) + diameter B→A,
%    start and end at A.
%  - EE orientation: fixed, Z-axis pointing up (world +Z).
%
%  Outputs (unchanged):
%       joint_positions.mat      (jointi_ref)
%       joint_velocities.mat     (jointi_dref)
%       joint_accelerations.mat  (jointi_ddref)
%       ee_trajectory.mat        (ee_desired, ee_actual)
% ========================================================================

clear; clc;

%% ---------------- FLAGS (speed vs diagnostics) --------------------------
DO_MONTE_CARLO = false;   % true = random workspace sampling
DO_PLOTS       = true;    % true = plot q/dq/ddq and EE path
DO_ANIM        = false;   % true = show() animation (slow)

%% ---------------- CONFIGURATION ----------------------------------------
Ts         = 0.01;        % internal sample time [s]
total_time = 20;          % total trajectory time [s]
nPts       = round(total_time / Ts);

maxPoseTol = 0.02;        % [m] only for reporting

% Joint 1 desired span for the ARC (A -> B)
q1_A = -1.57;             % rad
q1_B =  6.578e-01;        % rad

% Path segmentation (coarse points before resampling)
n_arc_coarse   = 400;
n_chord_coarse = 160;

% J2 soft corridor *in radians* (will be intersected with URDF limits)
J2_soft_raw = [-8.389, -6.6];    % rad

% Downsample factor for saving ONLY
save_ds_factor = 5;       % 1 = full, 5 = every 5th point

% Animation step (if enabled)
anim_every = 6;

%% ---------------- ROBOT SETUP ------------------------------------------
try
    robot = importrobot('Mulag_FME500.urdf');
catch
    warning('URDF not found. Creating simple 5-DOF demo robot.');
    robot = createSimpleRobot();
end
robot.DataFormat = 'column';

% End-effector
endEffector = 'Messerkopf';
if ~any(strcmp(robot.BodyNames, endEffector))
    endEffector = robot.BodyNames{end};
    warning('Using end effector: %s', endEffector);
end

% Home pose (for orientation ref / plotting)
homeQ  = homeConfiguration(robot);
T_home = getTransform(robot, homeQ, endEffector);
p_home = T_home(1:3,4);            % [x;y;z]

% ---- Desired EE orientation: Z-axis up in base frame -------------------
% We want the EE local z-axis aligned with world +Z.
% Simplest choice: EE frame = base frame (R_des = I).
R_des = eye(3);

% Joint meta info and limits
[jType,qmin,qmax,isRev,isPris] = jointMeta(robot);
JDOF = numel(qmin);

fprintf('Robot DOFs: %d | nPts: %d | Ts: %.3f s\n', JDOF, nPts, Ts);

% ----- Build J2 soft corridor with feasible fallback --------------------
raw    = sort(J2_soft_raw(:).');        % [min_raw, max_raw] in rad
rawMin = raw(1);
rawMax = raw(2);

% intersect with URDF limits of joint 2
J2_soft_min = max(rawMin, qmin(2));
J2_soft_max = min(rawMax, qmax(2));

if J2_soft_min >= J2_soft_max
    % no feasible corridor -> fall back to full URDF range
    J2_soft_min = qmin(2);
    J2_soft_max = qmax(2);
    fprintf('J2 raw corridor infeasible -> using URDF limits [%.3f, %.3f] rad\n', ...
        J2_soft_min, J2_soft_max);
else
    fprintf('J2 soft corridor (after feasibility check): [%.3f, %.3f] rad\n', ...
        J2_soft_min, J2_soft_max);
end

%% ---------------- Approx. global reach (fast) --------------------------
if DO_MONTE_CARLO
    nMC = 800;
    Pmc = zeros(3,nMC);
    for i = 1:nMC
        qc = randomConfiguration(robot);
        T  = getTransform(robot, qc, endEffector);
        Pmc(:,i) = T(1:3,4);
    end
    distances = vecnorm(Pmc);
    R_global  = max(distances);
else
    R_global = 1.5 * hypot(p_home(1), p_home(2));
end
fprintf('Approx. global reach ≈ %.3f m\n', R_global);

%% ---------------- IK OBJECT --------------------------------------------
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverAlgorithm = "LevenbergMarquardt";

sp = ik.SolverParameters;
sp.MaxIterations      = 60;
sp.GradientTolerance  = 1e-6;
sp.SolutionTolerance  = 1e-4;
sp.AllowRandomRestart = false;   % keep same branch, avoid jumps
sp.EnforceJointLimits = true;
ik.SolverParameters   = sp;

% Position + orientation IK (orientation softer than position)
% [w_ori_x w_ori_y w_ori_z w_pos_x w_pos_y w_pos_z]
weights = [0.5 0.5 0.5 1 1 1];

%% =======================================================================
%  BUILD CLOSED PATH: ARC (A->B) + DIAMETER (B->A)
% ========================================================================

homeZ   = p_home(3);
q1_home = homeQ(1);

% choose horizontal radius based on home EE position (slightly inflated)
r_home = hypot(p_home(1), p_home(2));
if r_home < 0.75
    r_home = max(0.75, 0.4*R_global);
end

phi_home = atan2(p_home(2), p_home(1));

% ----- 1) Arc A->B ------------------------------------------------------
q1_arc  = linspace(q1_A, q1_B, n_arc_coarse);
phi_arc = phi_home + (q1_arc - q1_home);

cos_phi = cos(phi_arc);
sin_phi = sin(phi_arc);

P_arc = [ r_home*cos_phi; ...
          r_home*sin_phi; ...
          repmat(homeZ,1,n_arc_coarse) ];

A = P_arc(:,1);
B = P_arc(:,end);

% ----- 2) Diameter B->A (straight line) ---------------------------------
P_chord = [ linspace(B(1), A(1), n_chord_coarse); ...
            linspace(B(2), A(2), n_chord_coarse); ...
            linspace(B(3), A(3), n_chord_coarse) ];

% Avoid duplicate B at join by dropping first chord point
P_full = [P_arc, P_chord(:,2:end)];
Nfull  = size(P_full,2);

% Joint1 desired profile along coarse path:
q1_chord = linspace(q1_B, q1_A, n_chord_coarse-1);   % chord without first point
q1_full  = [q1_arc, q1_chord];
q1_full  = min(max(q1_full, qmin(1)), qmax(1));      % clamp to URDF J1 limits

fprintf('Coarse path points: arc=%d, chord=%d, total=%d\n', ...
        n_arc_coarse, n_chord_coarse-1, Nfull);

%% =======================================================================
%  RESAMPLE CLOSED PATH TO EXACTLY nPts VIA ARC-LENGTH
% ========================================================================

dP = diff(P_full,1,2);
ds = sqrt(sum(dP.^2,1));
ds(ds < 1e-9) = 1e-9;          % avoid zero step -> unique s
s  = [0, cumsum(ds)];
if s(end) < 1e-6
    s(end) = 1e-6;
end

s_new = linspace(0, s(end), nPts);

pd_des = zeros(3, nPts);
for i = 1:3
    pd_des(i,:) = interp1(s, P_full(i,:), s_new, 'linear');
end

% Resample joint1 desired profile along same arc-length parameter
q1_des = interp1(s, q1_full, s_new, 'linear');
q1_des = min(max(q1_des, qmin(1)), qmax(1));

fprintf('q1_des range: [%.3f, %.3f] rad\n', min(q1_des), max(q1_des));

%% =======================================================================
%  SEQUENTIAL IK WITH FIXED ORIENTATION
% ========================================================================

q         = zeros(JDOF, nPts);
ee_actual = zeros(3, nPts);
pd_used   = pd_des;              % desired path (for saving)
poseErr   = zeros(1, nPts);

for k = 1:nPts
    p_des = pd_des(:,k);

    % Previous configuration (for k==1 we use home as seed)
    if k == 1
        q_prev = homeQ;
    else
        q_prev = q(:,k-1);
    end

    % Seed: previous configuration but bias J1, and gently pull J2
    q_seed    = q_prev;
    q_seed(1) = q1_des(k);
    q_seed(2) = min(max(q_seed(2), J2_soft_min), J2_soft_max);  % hint only
    q_seed    = clampToLimits(q_seed, qmin, qmax);

    % Desired pose: fixed Z-up orientation + desired position
    T_des          = T_home;
    T_des(1:3,1:3) = R_des;      % fixed orientation
    T_des(1:3,4)   = p_des;      % control point position

    [q_sol, info] = ik(endEffector, T_des, weights, q_seed); %#ok<NASGU>
    q_sol         = clampToLimits(q_sol, qmin, qmax);

    % Actual EE position
    T_act        = getTransform(robot, q_sol, endEffector);
    p_act        = T_act(1:3,4);
    poseErr(k)   = norm(p_act - p_des);

    q(:,k)         = q_sol;
    ee_actual(:,k) = p_act;
end

time = (0:nPts-1)' * Ts;
fprintf('Max pose error: %.3f m (median %.3f m)\n', max(poseErr), median(poseErr));

%% =======================================================================
%  FULL-RES DERIVATIVES (ONLY IF PLOTTING)
% ========================================================================

if DO_PLOTS
    dq_full  = zeros(JDOF, nPts);
    ddq_full = zeros(JDOF, nPts);
    for j = 1:JDOF
        dq_full(j,:)  = gradient(q(j,:), Ts);
        ddq_full(j,:) = gradient(dq_full(j,:), Ts);
    end

    peak_dq  = max(max(abs(dq_full),[],2));
    peak_ddq = max(max(abs(ddq_full),[],2));
    fprintf('Max |dq| over joints (full-res):  %.3f\n', peak_dq);
    fprintf('Max |ddq| over joints (full-res): %.3f\n', peak_ddq);
end

%% =======================================================================
%  VISUALISATION (OPTIONAL)
% ========================================================================

if DO_PLOTS
    figure('Name','EE path (desired vs actual)','Color','w');
    plot3(pd_des(1,:), pd_des(2,:), pd_des(3,:), 'k--','LineWidth',1.2); hold on;
    plot3(ee_actual(1,:), ee_actual(2,:), ee_actual(3,:), 'b-','LineWidth',1.6);
    scatter3(p_home(1), p_home(2), p_home(3), 60, 'r', 'filled'); % home ref
    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Closed path: semicircle (A→B) + diameter (B→A), starting/ending at A');
    legend('Desired (arc+diameter)','Actual reachable','Home ref','Location','best');

    figure('Name','Joint Positions','Color','w');
    plot(time, q'); grid on;
    xlabel('t [s]'); ylabel('q'); title('Joint positions');

    figure('Name','Joint Velocities (full-res)','Color','w');
    plot(time, dq_full'); grid on;
    xlabel('t [s]'); ylabel('dq'); title('Joint velocities');

    figure('Name','Joint Accelerations (full-res)','Color','w');
    plot(time, ddq_full'); grid on;
    xlabel('t [s]'); ylabel('ddq'); title('Joint accelerations');
end

if DO_ANIM
    figure('Name','Manipulator Animation','Color','w');
    show(robot, q(:,1), 'PreservePlot', false); axis equal;
    for k = 1:anim_every:nPts
        show(robot, q(:,k), 'PreservePlot', false);
        title(sprintf('t = %.2f s', time(k)));
        drawnow;
    end
end

%% =======================================================================
%  DOWNSAMPLE FOR SAVING ONLY (FAST REFERENCE)
% ========================================================================

if save_ds_factor < 1, save_ds_factor = 1; end
idx = 1:save_ds_factor:nPts;

time_save      = time(idx);
q_save         = q(:,idx);
ee_des_save    = pd_used(:,idx);
ee_actual_save = ee_actual(:,idx);

nSave = numel(idx);
dq_save  = zeros(JDOF, nSave);
ddq_save = zeros(JDOF, nSave);
for j = 1:JDOF
    dq_save(j,:)  = gradient(q_save(j,:), time_save);
    ddq_save(j,:) = gradient(dq_save(j,:), time_save);
end

%% =======================================================================
%  SAVE DATASETS (SAME STRUCTURE AS BEFORE)
% ========================================================================

jointPositions     = Simulink.SimulationData.Dataset;
jointVelocities    = Simulink.SimulationData.Dataset;
jointAccelerations = Simulink.SimulationData.Dataset;

for j = 1:JDOF
    if isRev(j)
        u_q  = 'rad';
        u_dq = 'rad/s';
        u_ddq= 'rad/s^2';
    else
        u_q  = 'm';
        u_dq = 'm/s';
        u_ddq= 'm/s^2';
    end

    % Position
    ts = timeseries(q_save(j,:).', time_save);
    ts.Name = sprintf('joint%d_ref', j);
    ts.DataInfo.Units = u_q;
    sig = Simulink.SimulationData.Signal;
    sig.Values = ts;
    sig.Name   = ts.Name;
    jointPositions = jointPositions.addElement(sig);

    % Velocity
    tsd = timeseries(dq_save(j,:).', time_save);
    tsd.Name = sprintf('joint%d_dref', j);
    tsd.DataInfo.Units = u_dq;
    sigd = Simulink.SimulationData.Signal;
    sigd.Values = tsd;
    sigd.Name   = tsd.Name;
    jointVelocities = jointVelocities.addElement(sigd);

    % Acceleration
    tsa = timeseries(ddq_save(j,:).', time_save);
    tsa.Name = sprintf('joint%d_ddref', j);
    tsa.DataInfo.Units = u_ddq;
    siga = Simulink.SimulationData.Signal;
    siga.Values = tsa;
    siga.Name   = tsa.Name;
    jointAccelerations = jointAccelerations.addElement(siga);
end

save('joint_positions.mat',     'jointPositions',     '-v7.3');
save('joint_velocities.mat',    'jointVelocities',    '-v7.3');
save('joint_accelerations.mat', 'jointAccelerations', '-v7.3');

% EE trajectory dataset (desired + actual), downsampled
eeTrajectory = Simulink.SimulationData.Dataset;
ts_des = timeseries(ee_des_save.',    time_save);
ts_act = timeseries(ee_actual_save.', time_save);
ts_des.Name = 'ee_desired'; ts_des.DataInfo.Units = 'm';
ts_act.Name = 'ee_actual';  ts_act.DataInfo.Units = 'm';

s1 = Simulink.SimulationData.Signal; s1.Values = ts_des; s1.Name = ts_des.Name;
s2 = Simulink.SimulationData.Signal; s2.Values = ts_act; s2.Name = ts_act.Name;
eeTrajectory = eeTrajectory.addElement(s1);
eeTrajectory = eeTrajectory.addElement(s2);

save('ee_trajectory.mat','eeTrajectory','-v7.3');

fprintf('\nSaved (downsample factor = %d):\n  joint_positions.mat\n  joint_velocities.mat\n  joint_accelerations.mat\n  ee_trajectory.mat\n', save_ds_factor);

%% ========================= HELPER FUNCTIONS =============================
function [jType,qmin,qmax,isRev,isPris] = jointMeta(robot)
    fmt0 = robot.DataFormat;
    robot.DataFormat = 'struct';
    cfgS = homeConfiguration(robot);
    robot.DataFormat = fmt0;

    n = numel(cfgS);
    jType = strings(n,1);
    qmin  = -inf(n,1);
    qmax  =  inf(n,1);

    bodies   = robot.Bodies;
    jointMap = containers.Map('KeyType','char','ValueType','any');
    for i = 1:numel(bodies)
        J = bodies{i}.Joint;
        if ~strcmp(J.Type,'fixed')
            jointMap(J.Name) = J;
        end
    end

    for i = 1:n
        jname = cfgS(i).JointName;
        if isKey(jointMap, jname)
            J = jointMap(jname);
            jType(i) = string(J.Type);
            lim = J.PositionLimits;
            if ~isempty(lim)
                qmin(i) = lim(1);
                qmax(i) = lim(2);
                if qmin(i) == qmax(i)
                    qmin(i) = -inf; qmax(i) = inf;
                end
            end
        else
            jType(i) = "revolute";
            qmin(i)  = -pi;
            qmax(i)  =  pi;
        end
    end

    isRev  = (jType == "revolute");
    isPris = (jType == "prismatic");
end

function qc = clampToLimits(q, qmin, qmax)
    qc = min(max(q, qmin), qmax);
end

function robot = createSimpleRobot()
    % Simple 5-DOF arm (fallback if URDF missing)
    L1=0.5; L2=0.5; L3=0.4; L4=0.3; L5=0.25;
    robot = rigidBodyTree('MaxNumBodies',6,'DataFormat','column');

    b1 = rigidBody('b1');
    j1 = rigidBodyJoint('j1','revolute');
    setFixedTransform(j1,trvec2tform([0 0 0]));
    b1.Joint = j1;
    addBody(robot,b1,'base');

    b2 = rigidBody('b2');
    j2 = rigidBodyJoint('j2','revolute');
    setFixedTransform(j2,trvec2tform([0 0 L1]));
    b2.Joint = j2;
    addBody(robot,b2,'b1');

    b3 = rigidBody('b3');
    j3 = rigidBodyJoint('j3','revolute');
    setFixedTransform(j3,trvec2tform([L2 0 0]));
    b3.Joint = j3;
    addBody(robot,b3,'b2');

    b4 = rigidBody('b4');
    j4 = rigidBodyJoint('j4','revolute');
    setFixedTransform(j4,trvec2tform([L3 0 0]));
    b4.Joint = j4;
    addBody(robot,b4,'b3');

    b5 = rigidBody('b5');
    j5 = rigidBodyJoint('j5','revolute');
    setFixedTransform(j5,trvec2tform([L4 0 0]));
    b5.Joint = j5;
    addBody(robot,b5,'b4');

    bee = rigidBody('Messerkopf');
    je  = rigidBodyJoint('je','fixed');
    setFixedTransform(je,trvec2tform([L5 0 0]));
    bee.Joint = je;
    addBody(robot,bee,'b5');
end
