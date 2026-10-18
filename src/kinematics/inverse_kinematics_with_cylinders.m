clc;

%% ---------------- USER SETTINGS ----------------
MODEL_HINT  = 'Mulag_FME_500_with_cylinder.slx';   % name/path; dialog appears if not found
DESIRED_DOF = 5;                               
PLOT        = true;                            % quick visualization (Visuals OFF)

% Mowing operation parameters
MOWING_WIDTH = 2.5;           % Total mowing width (meters)
FORWARD_DISTANCE = 4.0;       % Total forward travel distance (meters)
WORKING_HEIGHT = 0.15;        % Mowing height above ground (meters)
CLEARANCE_HEIGHT = 0.4;       % Clearance height for obstacles (meters)
SWEEP_CYCLES = 3;             % Number of left-right sweep cycles
Tf          = 30;             % Total operation time (seconds)
NPTS        = 200;            % trajectory waypoints
Ns          = 1000;           % samples to export
%% ------------------------------------------------

%% 1) Load model (robust)
try
    [mdlName, ~] = robustLoadModel(MODEL_HINT);
catch
    warning('Model not found from hint. Pick the .slx file...');
    [f,p] = uigetfile({'*.slx;*.mdl','Simulink models'}, 'Select Simscape model');
    assert(~isequal(f,0),'No model selected.');
    [mdlName, ~] = robustLoadModel(fullfile(p,f));
end
fprintf('Loaded model: %s\n', mdlName);

%% 2) Auto-pick end target (Solid/File Solid/Body preferred; else Joint/Rigid)
endPath = pick_end_target_auto(mdlName);
[~, endName] = fileparts(endPath);
fprintf('End target (auto): %s\n', endPath);

%% 3) Scrape World->End kinematics and build rigidBodyTree
kin = scrape_kinematics(mdlName, endPath);

% Trim/validate DOF
if numel(kin.chain) < DESIRED_DOF
    error('Found only %d joints on path to %s, need %d.', numel(kin.chain), endName, DESIRED_DOF);
elseif numel(kin.chain) > DESIRED_DOF
    kin.chain = kin.chain(1:DESIRED_DOF);
end

robot = build_rbt_from_kin(kin);
robot.DataFormat = "row";

% Apply URDF limits for the 5 joints
robot = applyUrdfLimits_Mulag(robot);

% Names, limits, home vector (and print table)
robot.DataFormat = 'struct';
cfg = homeConfiguration(robot);
jointNames = string({cfg.JointName});
qHome = [cfg.JointPosition];
robot.DataFormat = 'row';

L = cell(numel(cfg),1); T = strings(numel(cfg),1); A = zeros(numel(cfg),3);
Lo = nan(numel(cfg),1); Up = nan(numel(cfg),1); U = strings(numel(cfg),1);
for i=1:numel(cfg)
    % Access joint via body index
    body_i = robot.Bodies{i};
    ji = body_i.Joint;
    L{i} = ji.Name; 
    T(i) = string(ji.Type); 
    A(i,:) = ji.JointAxis(:).';
    lim = ji.PositionLimits; 
    Lo(i)=lim(1); 
    Up(i)=lim(2);
    if strcmp(ji.Type,'prismatic')
        U(i) = "m";
    elseif strcmp(ji.Type,'revolute')
        U(i) = "rad";
    else
        U(i) = "";
    end
end
JOINT_LIMITS = table(string(L),T,A,Lo,Up,U, 'VariableNames',{'Joint','Type','AxisXYZ','Lower','Upper','Units'});
disp('---- JOINT LIMITS (used by IK) ----'); 
disp(JOINT_LIMITS);
assignin('base','JOINT_LIMITS',JOINT_LIMITS);

% Extract joint limits for later use
jointLimits = zeros(numel(cfg), 2);
for i = 1:numel(cfg)
    body_i = robot.Bodies{i};
    ji = body_i.Joint;
    lim = ji.PositionLimits;
    jointLimits(i, :) = lim;
end

% Home transform
T_home = getTransform(robot, qHome, 'tool', robot.BaseName);
p_home = tform2trvec(T_home);

%% 4) Generate Practical Mowing Trajectory
fprintf('Generating realistic mowing trajectory...\n');

% Time vector for trajectory generation
t_traj = linspace(0, Tf, NPTS);

% Realistic mowing pattern parameters
forward_speed = FORWARD_DISTANCE / Tf;  % m/s
sweep_period = Tf / SWEEP_CYCLES;       % seconds per sweep cycle

% Initialize trajectory arrays
xPath = zeros(NPTS, 1);
yPath = zeros(NPTS, 1);
zPath = zeros(NPTS, 1);

for i = 1:NPTS
    t = t_traj(i);
    
    % Forward motion (vehicle moving slowly)
    xPath(i) = p_home(1) + forward_speed * t;
    
    % Sweeping motion (left-right)
    sweep_phase = 2 * pi * t / sweep_period;
    yPath(i) = (MOWING_WIDTH/2) * sin(sweep_phase);
    
    % Height management with realistic features
    base_height = WORKING_HEIGHT;
    
    % Add ground following (small random variations)
    ground_variation = 0.02 * sin(2*pi*0.8*t) + 0.01 * sin(2*pi*2.5*t);
    
    % Lift slightly at sweep extremes for better cutting
    sweep_extreme_lift = 0.02 * abs(sin(sweep_phase));
    
    % Obstacle avoidance (periodic lifts)
    obstacle_lift = 0;
    if mod(t, 8) > 6.5 && mod(t, 8) < 7.5
        obstacle_lift = (CLEARANCE_HEIGHT - WORKING_HEIGHT) * sin(pi*(mod(t,8)-6.5));
    end
    
    zPath(i) = base_height + ground_variation + sweep_extreme_lift + obstacle_lift;
end

% Smooth the trajectory for better IK convergence
xPath = smoothdata(xPath, 'gaussian', 15);
yPath = smoothdata(yPath, 'gaussian', 15);
zPath = smoothdata(zPath, 'gaussian', 15);

% Combine into waypoints
p_line = [xPath, yPath, zPath];

% Visualize the mowing trajectory
figure('Position', [100, 100, 1200, 800]);

subplot(2,2,1);
plot3(p_line(:,1), p_line(:,2), p_line(:,3), 'b-', 'LineWidth', 2);
hold on;
plot3(p_line(1,1), p_line(1,2), p_line(1,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(p_line(end,1), p_line(end,2), p_line(end,3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Mower Head Trajectory');
grid on; axis equal;
legend('Path', 'Start', 'End');

subplot(2,2,2);
plot(t_traj, p_line(:,3), 'b-', 'LineWidth', 2);
hold on;
yline(WORKING_HEIGHT, 'r--', 'Working Height');
yline(CLEARANCE_HEIGHT, 'g--', 'Clearance Height');
xlabel('Time (s)'); ylabel('Height (m)');
title('Mower Height Profile');
grid on;
legend('Actual Height', 'Location', 'best');

subplot(2,2,3);
plot(t_traj, p_line(:,2), 'm-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Lateral Position (m)');
title('Sweep Pattern');
grid on;

subplot(2,2,4);
plot(t_traj, p_line(:,1), 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Forward Position (m)');
title('Forward Motion');
grid on;

sgtitle('Realistic Mowing Operation Pattern');

%% 5) Position-only IK along the mowing trajectory
ik = inverseKinematics('RigidBodyTree', robot);
w  = [1 1 1 0 0 0];                 % position-only (safe for 5-DOF)
Rkeep = T_home(1:3,1:3);            % placeholder (ignored by weights)

Qsol  = zeros(NPTS, numel(cfg));
ok    = false(NPTS,1);
qSeed = qHome(:).';

fprintf('Solving inverse kinematics for mowing trajectory...\n');

for k=1:NPTS
    Tgoal = trvec2tform(p_line(k,:)) * rotm2tform(Rkeep);
    [qNext, ~] = ik('tool', Tgoal, w, qSeed);
    qNext = clampToLimits(robot, qNext);
    ok(k) = all(isfinite(qNext));
    if ok(k), Qsol(k,:) = qNext; qSeed = qNext; end
    
    % Progress display
    if mod(k, 20) == 0
        fprintf('Progress: %.1f%%\n', (k/NPTS)*100);
    end
end

keepRatio = mean(ok);
fprintf('IK kept %d/%d waypoints (%.0f%%)\n', nnz(ok), NPTS, 100*keepRatio);
assert(nnz(ok)>=2,'Too few solved points. Try adjusting trajectory.');

Qsol   = Qsol(ok,:); 
p_line = p_line(ok,:);
t_traj_ok = t_traj(ok);

%% 6) Time-parameterize & export to Simscape
tvec = linspace(0,Tf,Ns).';
s = [0; cumsum(vecnorm(diff(p_line),2,2))];
sQuery = linspace(0, s(end), Ns);
Qinterp = zeros(size(Qsol,2), Ns);
for j=1:size(Qsol,2)
    Qinterp(j,:) = interp1(s, Qsol(:,j), sQuery, 'pchip', 'extrap');
end

% Export joint references
for j=1:numel(cfg)
    assignin('base', sprintf('qRef_J%d', j), timeseries(Qinterp(j,:).', tvec));
    assignin('base', ['qRef_' sanitizeName(jointNames(j))], timeseries(Qinterp(j,:).', tvec));
end
assignin('base','tvec', tvec);
fprintf('Exported qRef_J1..J%d and tvec to base workspace.\n', numel(cfg));

% Also export the Cartesian trajectory for reference
assignin('base', 'mower_trajectory', p_line);
assignin('base', 'trajectory_time', t_traj_ok);

fprintf('Mowing trajectory parameters:\n');
fprintf('  Total distance: %.1f m\n', FORWARD_DISTANCE);
fprintf('  Mowing width: %.1f m\n', MOWING_WIDTH);
fprintf('  Working height: %.1f m\n', WORKING_HEIGHT);
fprintf('  Sweep cycles: %d\n', SWEEP_CYCLES);
fprintf('  Operation time: %.1f s\n', Tf);

%% 7) Compute velocity and acceleration profiles for PID control
fprintf('Generating velocity and acceleration profiles...\n');

% Compute derivatives for better PID performance
qd_interp = zeros(size(Qinterp));
qdd_interp = zeros(size(Qinterp));

for j = 1:size(Qinterp, 1)
    % Position
    pos = Qinterp(j, :);
    
    % Velocity (smoothed)
    vel = gradient(pos, tvec(2)-tvec(1));
    vel = smoothdata(vel, 'gaussian', 50); % Smooth for better control
    qd_interp(j, :) = vel;
    
    % Acceleration (smoothed)
    acc = gradient(vel, tvec(2)-tvec(1));
    acc = smoothdata(acc, 'gaussian', 50);
    qdd_interp(j, :) = acc;
end

%% 8) Save Complete Trajectory Data for PID Controller Design
fprintf('Saving trajectory data for PID controller...\n');

% Create a comprehensive data structure with all necessary information
trajectory_data = struct();

% Joint references (most important for PID)
trajectory_data.joint_positions = Qinterp';  % Ns x nJoints
trajectory_data.joint_velocities = qd_interp'; 
trajectory_data.joint_accelerations = qdd_interp';
trajectory_data.time_vector = tvec;

% Cartesian trajectory for reference
trajectory_data.cartesian_trajectory = p_line;
trajectory_data.cartesian_time = t_traj_ok;

% Robot information
trajectory_data.robot_config = cfg;
trajectory_data.joint_names = jointNames;
trajectory_data.joint_limits = jointLimits;

% Trajectory parameters
trajectory_data.parameters.MOWING_WIDTH = MOWING_WIDTH;
trajectory_data.parameters.FORWARD_DISTANCE = FORWARD_DISTANCE;
trajectory_data.parameters.WORKING_HEIGHT = WORKING_HEIGHT;
trajectory_data.parameters.CLEARANCE_HEIGHT = CLEARANCE_HEIGHT;
trajectory_data.parameters.SWEEP_CYCLES = SWEEP_CYCLES;
trajectory_data.parameters.TOTAL_TIME = Tf;

% IK information
trajectory_data.ik_success_rate = keepRatio;
trajectory_data.ik_waypoints_used = nnz(ok);

% Save to MATLAB file
save('mower_trajectory_reference.mat', '-struct', 'trajectory_data');

% Also create individual variables for easy access in Simulink
save('pid_reference_signals.mat', 'Qinterp', 'qd_interp', 'qdd_interp', 'tvec', 'jointNames', 'jointLimits');

fprintf('Trajectory data saved successfully!\n');
fprintf('   - mower_trajectory_reference.mat (complete data)\n');
fprintf('   - pid_reference_signals.mat (essential signals)\n');

%% 9) Export Timeseries for Direct Simulink Use
% Export complete timeseries for PID control
for j = 1:numel(cfg)
    % Position reference
    q_ref = timeseries(Qinterp(j, :)', tvec, 'Name', sprintf('q_ref_joint_%d', j));
    q_ref.DataInfo.Units = 'rad';
    q_ref.TimeInfo.Units = 'seconds';
    
    % Velocity reference
    qd_ref = timeseries(qd_interp(j, :)', tvec, 'Name', sprintf('qd_ref_joint_%d', j));
    qd_ref.DataInfo.Units = 'rad/s';
    qd_ref.TimeInfo.Units = 'seconds';
    
    % Acceleration reference
    qdd_ref = timeseries(qdd_interp(j, :)', tvec, 'Name', sprintf('qdd_ref_joint_%d', j));
    qdd_ref.DataInfo.Units = 'rad/s²';
    qdd_ref.TimeInfo.Units = 'seconds';
    
    % Export to base workspace
    assignin('base', sprintf('q_ref_%d', j), q_ref);
    assignin('base', sprintf('qd_ref_%d', j), qd_ref);
    assignin('base', sprintf('qdd_ref_%d', j), qdd_ref);
end

% Export trajectory info structure
traj_info = struct();
traj_info.joint_names = jointNames;
traj_info.joint_limits = jointLimits;
traj_info.sampling_time = tvec(2) - tvec(1);
traj_info.duration = Tf;
traj_info.num_joints = numel(cfg);
traj_info.trajectory_type = 'mowing_operation';

assignin('base', 'trajectory_info', traj_info);
assignin('base', 't_ref', tvec);

fprintf('All reference signals exported to workspace!\n');
fprintf('Available for PID controller:\n');
fprintf('   Joint positions: q_ref_1 to q_ref_%d\n', numel(cfg));
fprintf('   Joint velocities: qd_ref_1 to qd_ref_%d\n', numel(cfg));
fprintf('   Joint accelerations: qdd_ref_1 to qdd_ref_%d\n', numel(cfg));
fprintf('   Time vector: t_ref\n');
fprintf('   Trajectory info: trajectory_info\n');

%% 10) Quick visualization (optional)
if PLOT
    try
        fig = figure('Name','Mowing Trajectory IK Solution','NumberTitle','off','Visible','on');
        ax = axes('Parent',fig); 
        hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); view(ax,135,20);
        
        % Plot the Cartesian trajectory
        plot3(ax, p_line(:,1), p_line(:,2), p_line(:,3), 'b-', 'LineWidth', 2);
        
        % Show robot at key frames
        frames = unique(round(linspace(1, size(Qsol,1), min(8,size(Qsol,1)))));
        for k = frames(:).'
            show(robot, Qsol(k,:), 'Frames','off','PreservePlot',false, 'Parent',ax, 'Visuals','off');
            drawnow limitrate;
        end
        
        plot3(ax, p_line(1,1), p_line(1,2), p_line(1,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
        plot3(ax, p_line(end,1), p_line(end,2), p_line(end,3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        
        xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
        title(ax,'Mowing Operation - IK Solution');
        legend(ax, 'Trajectory', 'Start', 'End', 'Location', 'best');
        
    catch ME
        warning('Visualization skipped: %s', E.message);
    end
end

%% 11) Generate PID Controller Readme
fprintf('\n=== PID CONTROLLER SETUP INSTRUCTIONS ===\n');
fprintf('1. Use these signals in your Simulink PID controllers:\n');
fprintf('   - Reference: q_ref_1 to q_ref_%d\n', numel(cfg));
fprintf('   - Feedforward (optional): qd_ref_1 to qd_ref_%d\n', numel(cfg));
fprintf('   - Time vector: t_ref\n');
fprintf('\n2. Recommended PID structure:\n');
fprintf('   Error = q_ref - actual_joint_position\n');
fprintf('   PID Output = Kp*Error + Ki*∫Error + Kd*d(Error)/dt\n');
fprintf('   Optional: Add feedforward: + Kff*qd_ref\n');
fprintf('\n3. Load the saved .mat files in your controller initialization:\n');
fprintf('   load(''pid_reference_signals.mat'');\n');
fprintf('\n4. Joint limits for safety:\n');
for j = 1:numel(cfg)
    fprintf('   Joint %d: [%.3f, %.3f] rad\n', j, jointLimits(j,1), jointLimits(j,2));
end
fprintf('\nTrajectory generation complete! Ready for PID controller design.\n');

%% ======================= LOCAL FUNCTIONS =======================

function [mdlName, mdlPath] = robustLoadModel(modelInput)
    if ~(ischar(modelInput) || isstring(modelInput))
        error('modelInput must be char or string.');
    end
    modelInput = convertStringsToChars(modelInput);

    [p,f,e] = fileparts(modelInput);
    if isempty(e) || ~(strcmpi(e,'.slx') || strcmpi(e,'.mdl'))
        cand = which([modelInput '.slx']);
        if isempty(cand), cand = which([modelInput '.mdl']); end
        if isempty(cand), error('Model "%s" not found on path.', modelInput); end
        mdlFile = cand;
    else
        if exist(modelInput,'file') ~= 2
            cand = which(modelInput);
            if isempty(cand), error('Model file not found: %s', modelInput); end
            mdlFile = cand;
        else
            mdlFile = modelInput;
        end
    end
    mdlPath = mdlFile;
    load_system(mdlFile);

    bds = find_system('Type','block_diagram'); mdlName = '';
    for i = 1:numel(bds)
        if strcmpi(get_param(bds{i}, 'FileName'), mdlFile)
            mdlName = get_param(bds{i}, 'Name'); break;
        end
    end
    if isempty(mdlName), mdlName = f; end

    loaded = get_param(find_system('Type','block_diagram'),'Name');
    if iscell(loaded), loaded = string(loaded); else, loaded = string({loaded}); end
    if ~any(loaded == string(mdlName))
        error('Failed to load model from "%s".', mdlFile);
    end
end

function endPath = pick_end_target_auto(mdlName)
    worlds = find_system(mdlName,'LookUnderMasks','all','FollowLinks','on','MaskType','World Frame');
    assert(~isempty(worlds),'No World Frame found.');
    worldH = getSimulinkBlockHandle(worlds{1});

    solids1 = find_system(mdlName,'LookUnderMasks','all','FollowLinks','on', ...
                          'MatchFilter',@Simulink.match.allVariants,'MaskType','Solid');
    solids2 = find_system(mdlName,'LookUnderMasks','all','FollowLinks','on', ...
                          'MatchFilter',@Simulink.match.allVariants,'MaskType','File Solid');
    solids3 = find_system(mdlName,'LookUnderMasks','all','FollowLinks','on', ...
                          'MatchFilter',@Simulink.match.allVariants,'MaskType','Body');
    solids  = unique([solids1; solids2; solids3]);

    rev  = find_system(mdlName,'LookUnderMasks','all','FollowLinks','on','MaskType','Revolute Joint');
    pris = find_system(mdlName,'LookUnderMasks','all','FollowLinks','on','MaskType','Prismatic Joint');
    rigs = find_system(mdlName,'LookUnderMasks','all','FollowLinks','on','MaskType','Rigid Transform');

    nodes = unique([worlds; solids; rev; pris; rigs]);
    [h2p, E] = build_connectivity(nodes);
    dist = bfs_distances(worldH, E);
    degOf = @(h) sum(E(:,1)==h) + sum(E(:,2)==h);

    function bestH = rankPick(candsH, preferJoint)
        if isempty(candsH), bestH = []; return; end
        candsH = candsH(arrayfun(@(h) isKey(dist,h), candsH));
        if isempty(candsH), bestH = []; return; end
        D = arrayfun(@(h) dist(h), candsH);
        G = arrayfun(degOf, candsH);
        if preferJoint
            JH = [cellfun(@getSimulinkBlockHandle, rev(:)); cellfun(@getSimulinkBlockHandle, pris(:))];
            isJ = ismember(candsH, JH);
            S = [D(:), double(G(:)==1), double(isJ(:))];
            [~,idx] = sortrows(S, [-1,-2,-3]);
        else
            S = [D(:), double(G(:)==1)];
            [~,idx] = sortrows(S, [-1,-2]);
        end
        bestH = candsH(idx(1));
    end

    SH = cellfun(@getSimulinkBlockHandle, solids);
    pickH = rankPick(SH, false);
    if isempty(pickH)
        JH = [cellfun(@getSimulinkBlockHandle, rev(:)); cellfun(@getSimulinkBlockHandle, pris(:))];
        pickH = rankPick(JH, true);
    end
    if isempty(pickH)
        RH = cellfun(@getSimulinkBlockHandle, rigs);
        pickH = rankPick(RH, false);
    end
    assert(~isempty(pickH),'Could not find any reachable candidate from World.');

    endPath = h2p(pickH);
    [~,nm] = fileparts(endPath);
    fprintf('Auto-picked end target: %s (dist=%d, deg=%d)\n', nm, dist(pickH), degOf(pickH));
end

function [h2p, E] = build_connectivity(nodes)
    h2p = containers.Map('KeyType','double','ValueType','char');
    for i = 1:numel(nodes)
        try
            h = getSimulinkBlockHandle(nodes{i});
            if h > 0 && ~isKey(h2p,h), h2p(h) = getfullname(h); end
        catch
        end
    end
    E = [];
    for i = 1:numel(nodes)
        h = getSimulinkBlockHandle(nodes{i});
        if h <= 0, continue; end
        pc = get_param(h,'PortConnectivity');
        for k = 1:numel(pc)
            if isfield(pc(k),'Connected') && ~isempty(pc(k).Connected)
                for m = 1:numel(pc(k).Connected)
                    sb = pc(k).Connected(m).SrcBlock;
                    if ~isempty(sb)
                        for s = sb(:).'
                            if s > 0 && s ~= h
                                if ~isKey(h2p,h), h2p(h) = getfullname(h); end
                                if ~isKey(h2p,s), h2p(s) = getfullname(s); end
                                E(end+1,:) = sort([h s]);
                            end
                        end
                    end
                end
            else
                if isfield(pc(k),'SrcBlock') && ~isempty(pc(k).SrcBlock)
                    for s = pc(k).SrcBlock(:).'
                        if s>0 && s~=h
                            if ~isKey(h2p,h), h2p(h) = getfullname(h); end
                            if ~isKey(h2p,s), h2p(s) = getfullname(s); end
                            E(end+1,:) = sort([h s]);
                        end
                    end
                end
                if isfield(pc(k),'DstBlock') && ~isempty(pc(k).DstBlock)
                    for d = pc(k).DstBlock(:).'
                        if d>0 && d~=h
                            if ~isKey(h2p,h), h2p(h) = getfullname(h); end
                            if ~isKey(h2p,d), h2p(d) = getfullname(d); end
                            E(end+1,:) = sort([h d]);
                        end
                    end
                end
            end
        end
    end
    if ~isempty(E), E = unique(E,'rows'); end
end

function dist = bfs_distances(startH,E)
    nbr=@(h) unique([E(E(:,1)==h,2); E(E(:,2)==h,1)].');
    dist=containers.Map('KeyType','double','ValueType','double');
    seen=containers.Map('KeyType','double','ValueType','logical'); seen(startH)=true;
    q=startH; head=1; dist(startH)=0;
    while head<=numel(q)
        cur=q(head); head=head+1;
        for t=nbr(cur)
            if ~isKey(seen,t)
                seen(t)=true; dist(t)=dist(cur)+1; q(end+1)=t;
            end
        end
    end
end

function kin = scrape_kinematics(mdlName, endPath)
    worlds = find_system(mdlName,'LookUnderMasks','all','FollowLinks','on','MaskType','World Frame');
    assert(~isempty(worlds),'World Frame not found.');
    worldH = getSimulinkBlockHandle(worlds{1});

    allNodes = [worlds; ...
                find_system(mdlName,'LookUnderMasks','all','FollowLinks','on','MaskType','Revolute Joint'); ...
                find_system(mdlName,'LookUnderMasks','all','FollowLinks','on','MaskType','Prismatic Joint'); ...
                find_system(mdlName,'LookUnderMasks','all','FollowLinks','on','MaskType','Rigid Transform'); ...
                find_system(mdlName,'LookUnderMasks','all','FollowLinks','on','MatchFilter',@Simulink.match.allVariants,'MaskType','Solid'); ...
                find_system(mdlName,'LookUnderMasks','all','FollowLinks','on','MatchFilter',@Simulink.match.allVariants,'MaskType','File Solid'); ...
                find_system(mdlName,'LookUnderMasks','all','FollowLinks','on','MatchFilter',@Simulink.match.allVariants,'MaskType','Body')];

    [h2p,E] = build_connectivity(unique(allNodes));
    nbr = @(h) unique([E(E(:,1)==h,2); E(E(:,2)==h,1)].');

    endH  = getSimulinkBlockHandle(endPath);
    pathH = bfs(worldH, endH, nbr);
    assert(~isempty(pathH),'No path World->End.');

    chain = struct('joint',[],'X_parent_child',[]);
    T = eye(4); idx = 0;

    for k=1:numel(pathH)
        cur = pathH(k);
        pth = h2p(cur);
        mt  = get_param(cur,'MaskType');

        if any(strcmp(mt, {'Revolute Joint','Prismatic Joint'}))
            idx = idx+1;
            chain(idx).joint = readJoint(pth);
            chain(idx).X_parent_child = eye(4);
        elseif strcmp(mt,'Rigid Transform')
            X = readRigid(pth);
            T = T*X; 
            if idx>0, chain(idx).X_parent_child = chain(idx).X_parent_child * X; end
        else
            % Solid/File Solid/Body -> no params to scrape (visual only)
        end
    end

    kin.chain      = chain;
    kin.T_base_end = T;
    kin.toolT      = eye(4);
end

function path = bfs(startH,goalH,nbr)
    q=startH; head=1; prev=containers.Map('KeyType','double','ValueType','double'); 
    seen=containers.Map('KeyType','double','ValueType','logical'); seen(startH)=true;
    while head<=numel(q)
        cur=q(head); head=head+1; if cur==goalH, break; end
        for t=nbr(cur)
            if ~isKey(seen,t), seen(t)=true; prev(t)=cur; q(end+1)=t; end
        end
    end
    if ~isKey(prev,goalH), path=[]; return; end
    path=goalH; while path(1)~=startH, path=[prev(path(1)) path]; end
end

function J = readJoint(path)
    names=get_param(path,'MaskNames'); vals=get_param(path,'MaskValues');
    S=containers.Map; for i=1:numel(names), S(names{i})=vals{i}; end
    mt=get_param(path,'MaskType');
    axis = tryVec(S,["Axis","RevoluteAxis","PrismaticAxis","Axis of rotation","Axis of translation"]);
    hasL = tryBool(S,["SpecifyPositionLimits","Specify Limits","SpecifyPositionTargetAndLimits","SpecifyLimits"]);
    lo   = tryScalar(S,["LowerLimit","Lower Position Limit","PositionLowerLimit","Lower limit"]);
    up   = tryScalar(S,["UpperLimit","Upper Position Limit","PositionUpperLimit","Upper limit"]);
    units= tryText(S,["PositionUnits","Units","Position Target Units"]);
    if isempty(units)
        if strcmpi(mt,'Prismatic Joint')
            units = 'm';
        elseif strcmpi(mt,'Revolute Joint')
            units = 'rad';
        else
            units = '';
        end
    end
    [~,short]=fileparts(path);
    J=struct('path',path,'shortName',short,'type',mt,'axis',axis, ...
             'hasLimits',logical(hasL),'lower',lo,'upper',up,'units',units);
end

function X = readRigid(path)
    names=get_param(path,'MaskNames'); vals=get_param(path,'MaskValues');
    S=containers.Map; for i=1:numel(names), S(names{i})=vals{i}; end
    t = tryVec(S,["Translation","TranslationVector","Translation vector"]); if isempty(t), t=[0 0 0]; end
    R = eye(3);
    ang = tryScalar(S,["RotationAngle","Angle"]);
    axe = tryVec(S,["RotationAxis","Axis"]);
    if ~isempty(ang) && ~isempty(axe)
        R = axang2rotm([axe(:).' ang]);
    else
        seq = upper(tryText(S,["RotationSequence","Rotation Sequence"])); if isempty(seq), seq='ZYX'; end
        eul = tryVec(S,["RotationAngles","Angles","EulerAngles"]); if isempty(eul), eul=[0 0 0]; end
        R   = eul2rotm(eul(:).', seq);
    end
    X = eye(4); X(1:3,1:3)=R; X(1:3,4)=t(:);
end

function robot = build_rbt_from_kin(kin)
    robot = rigidBodyTree("DataFormat","row","MaxNumBodies", numel(kin.chain)+1);
    parent = robot.BaseName;
    for i=1:numel(kin.chain)
        b = rigidBody(sprintf('Body%d',i));
        if strcmpi(kin.chain(i).joint.type,'Prismatic Joint')
            j = rigidBodyJoint(sprintf('J%d',i),'prismatic');
        else
            j = rigidBodyJoint(sprintf('J%d',i),'revolute');
        end
        setFixedTransform(j, kin.chain(i).X_parent_child);
        if ~isempty(kin.chain(i).joint.axis), j.JointAxis = kin.chain(i).joint.axis(:).'; end
        if kin.chain(i).joint.hasLimits, j.PositionLimits=[kin.chain(i).joint.lower, kin.chain(i).joint.upper]; end
        b.Joint = j; addBody(robot,b,parent); parent=b.Name;
    end
    tool = rigidBody('tool'); jf = rigidBodyJoint('tool_fix','fixed');
    setFixedTransform(jf, kin.toolT); tool.Joint=jf; addBody(robot,tool,parent);
end

function robot = applyUrdfLimits_Mulag(robot)
    urdfLim = [ -1.57000   0.52333;
                -0.78500   0.78500;
                -1.22111   0.17444;
                -0.78500   0.78500;
                -1.04667   1.04667 ];
    urdfNames = ["Drehzapfen_joint","Ausleger_I_joint","Ausleger_II_joint","Messerkopf_Schwenk_joint","Messerkopf_joint"];

    assert(robot.NumBodies >= 5, 'applyUrdfLimits_Mulag: robot has fewer than 5 bodies.');
    for i = 1:5
        body_i = robot.Bodies{i};
        jObj = body_i.Joint;
        jName = jObj.Name;
        lim = urdfLim(i,:);
        ax = jObj.JointAxis(:);
        n = norm(ax);
        if n < eps
            ax = [0;0;1];
        else
            ax = ax / n;
        end
        if dot(ax, [0;0;1]) < -0.5
            lim = sort(-lim);
        end
        jObj.PositionLimits = lim;
        fprintf('Applied URDF limits to %-24s (RBT: %-12s): [% .5f, % .5f] rad\n', ...
            urdfNames(i), jName, lim(1), lim(2));
    end
end

function q = clampToLimits(robot,q)
    robot.DataFormat = 'struct';
    cfg = homeConfiguration(robot);
    robot.DataFormat = 'row';
    for i=1:numel(cfg)
        body_i = robot.Bodies{i};
        jObj = body_i.Joint;
        lim = jObj.PositionLimits;
        if all(isfinite(lim))
            q(i) = min(max(q(i), lim(1)), lim(2));
        end
    end
end

function v = tryVec(S,keys)
    v=[];
    for k=1:numel(keys)
        if isKey(S,keys(k))
            try
                vv=str2num(S(keys(k)));
                if isnumeric(vv) && numel(vv)==3, v=vv(:).'; return; end
            catch
                v=[];
            end
        end
    end
end

function x = tryScalar(S,keys)
    x=[];
    for k=1:numel(keys)
        if isKey(S,keys(k))
            xx=str2double(S(keys(k)));
            if isfinite(xx), x=xx; return; end
        end
    end
end

function t = tryText(S,keys)
    t='';
    for k=1:numel(keys)
        if isKey(S,keys(k)), t=S(keys(k)); return; end
    end
end

function b = tryBool(S,keys)
    b=false;
    for k=1:numel(keys)
        if isKey(S,keys(k))
            v=lower(strtrim(S(keys(k))));
            b=any(strcmp(v,{'on','true','1','yes'})); return;
        end
    end
end

function out = sanitizeName(nameIn)
    out = regexprep(char(nameIn), '[^a-zA-Z0-9_]', '_');
    if ~isvarname(out), out = matlab.lang.makeValidName(out); end
end