%% MULAG FME500 — Read kinematics from Simscape, rebuild RBT, and run a tiny IK demo
% This script does:
%  1) Resolves and loads your Simscape model (any format of name/path given).
%  2) Scrapes joint axes, limits, and rigid transforms (no sensors needed).
%  3) Builds a matching Robotics System Toolbox rigidBodyTree.
%  4) Runs a position-only IK example for 5-DOF.
%  5) Generates smooth joint trajectories for Simscape.

clear; clc;

%% --- USER SETTINGS ---
modelInput  = 'Mulag_FME500_kinematic.slx';  % name, filename, or full path
body5Name   = 'Body5';                       % last link block in your Simscape chain
toolOffsetT = eye(4);                        % tool frame offset on Body5 (identity if unknown)

%% --- 1) Resolve & load the model ---
[modelName, modelPath] = resolveModel(modelInput); %#ok<NASGU>

%% --- 2) Scrape kinematics from Simscape ---
kin = scrape_mulag_kinematics(modelName, body5Name);

fprintf('\n== Joint summary (base -> %s) ==\n', body5Name);
fprintf('#  name                 type         axis (base frame of joint)     limits\n');
for i = 1:numel(kin.chain)
    J = kin.chain(i).joint;
    ax = J.axis; if isempty(ax), ax = [NaN;NaN;NaN]; end
    limTxt = "open";
    if J.hasLimits, limTxt = sprintf('[%g, %g] %s', J.lower, J.upper, J.units); end
    fprintf('%-2d %-20s %-12s [%7.4f %7.4f %7.4f]  %s\n', ...
        i, J.shortName, J.type, ax(1), ax(2), ax(3), limTxt);
end

fprintf('\nComposed base->%s (fixed-only) at zero joint motion:\n', body5Name);
disp(kin.T_base_bodyN);

%% --- 3) Build a matching rigidBodyTree ---
robot = rigidBodyTree("DataFormat","row","MaxNumBodies", numel(kin.chain)+1);
parentName = robot.BaseName;
for i = 1:numel(kin.chain)
    bName = sprintf('Body%d', i);
    jName = sprintf('J%d', i);
    body  = rigidBody(bName);
    if strcmpi(kin.chain(i).joint.type, 'Prismatic Joint')
        joint = rigidBodyJoint(jName, 'prismatic');
    else
        joint = rigidBodyJoint(jName, 'revolute');
    end
    setFixedTransform(joint, kin.chain(i).X_parent_child);
    if ~isempty(kin.chain(i).joint.axis)
        joint.JointAxis = kin.chain(i).joint.axis(:).';
    end
    if kin.chain(i).joint.hasLimits
        joint.PositionLimits = [kin.chain(i).joint.lower, kin.chain(i).joint.upper];
    end
    body.Joint = joint;
    addBody(robot, body, parentName);
    parentName = bName;
end
tool = rigidBody('tool');
toolJoint = rigidBodyJoint('tool_fix','fixed');
setFixedTransform(toolJoint, toolOffsetT);
tool.Joint = toolJoint;
addBody(robot, tool, parentName);
showdetails(robot);

%% --- 4) IK example (position-only) ---
qHome = homeConfiguration(robot);
T_home = getTransform(robot, [qHome.JointPosition], 'tool');
p_home = tform2trvec(T_home);
R_free = eye(3);
p_goal = p_home + [0.20 0.00 0.00];
T_goal = trvec2tform(p_goal) * rotm2tform(R_free);
ik = inverseKinematics('RigidBodyTree', robot);
w_posOnly = [1 1 1 0 0 0];
[qSol, solInfo] = ik('tool', T_goal, w_posOnly, [qHome.JointPosition]);
fprintf('\nIK exit: %s (pos err=%.4g, orient err=%.4g)\n', ...
    solInfo.Status, solInfo.PoseErrorNorm(1), solInfo.PoseErrorNorm(2));
figure; show(robot, qHome, 'Frames','off','PreservePlot',false); hold on;
show(robot, qSol, 'Frames','off','PreservePlot',false);
legend('home','ik-solution');

%% --- 5) Smooth trajectory for Simscape ---
Tf = 3.0; Ns = 200; tvec = linspace(0, Tf, Ns);
Qway = [[qHome.JointPosition].' [qSol(:)]].';
[qRef, ~, ~] = trapveltraj(Qway, Ns);
jointNames = arrayfun(@(i) sprintf('qRef_J%d', i), 1:numel(kin.chain), 'uni', 0);
for j = 1:numel(kin.chain)
    assignin('base', jointNames{j}, timeseries(qRef(j,:).', tvec.'));
end
assignin('base','tvec', tvec);
fprintf('\nExported joint trajectories to base workspace:\n');
fprintf('  %s ... %s  (timeseries, seconds in ''tvec'')\n', jointNames{1}, jointNames{end});

%% ==================== Helper: resolve model name/path ====================
function [mdlName, mdlPath] = resolveModel(modelInput)
    if ~ischar(modelInput), modelInput = char(modelInput); end
    if endsWith(modelInput, {'.slx','.mdl'}, 'IgnoreCase',true)
        mdlPath = which(modelInput);
        if isempty(mdlPath), error('Model file not found: %s', modelInput); end
        [p, b] = fileparts(mdlPath); %#ok<ASGLU>
        mdlName = b;
        load_system(mdlPath);
    else
        mdlName = modelInput;
        mdlPath = '';
        if ~bdIsLoaded(mdlName)
            cand = which([mdlName '.slx']);
            if isempty(cand), cand = which([mdlName '.mdl']); end
            if isempty(cand)
                error('Model "%s" not loaded and no file found.', mdlName);
            end
            load_system(cand);
        end
    end
    if ~bdIsLoaded(mdlName)
        error('Failed to load model "%s".', mdlName);
    end
end

%% ==================== Helper: scrape kinematics ====================
function kin = scrape_mulag_kinematics(model, bodyN)
% Walks World -> bodyN, reading:
%   - each Joint's axis/limits
%   - each Rigid Transform's fixed transform
% Builds:
%   kin.chain(i).joint  (path, shortName, type, axis, limits)
%   kin.chain(i).X_parent_child  (4x4)
%   kin.T_base_bodyN (4x4)
    if ~bdIsLoaded(model), load_system(model); end

    % Collect candidate blocks we care about
    rev = find_system(model,'LookUnderMasks','all','FollowLinks','on', ...
        'MatchFilter',@Simulink.match.allVariants,'MaskType','Revolute Joint');
    pri = find_system(model,'LookUnderMasks','all','FollowLinks','on', ...
        'MatchFilter',@Simulink.match.allVariants,'MaskType','Prismatic Joint');
    rigs = find_system(model,'LookUnderMasks','all','FollowLinks','on', ...
        'MatchFilter',@Simulink.match.allVariants,'MaskType','Rigid Transform');
    world = find_system(model,'LookUnderMasks','all','FollowLinks','on', ...
        'MatchFilter',@Simulink.match.allVariants,'MaskType','World Frame');
    body  = find_system(model,'LookUnderMasks','all','FollowLinks','on','Name',bodyN);

    assert(~isempty(world), 'World Frame not found.');
    assert(~isempty(body),  'Body "%s" not found.', bodyN);

    blocks = [rev; pri; rigs; world; body];
    h2p = containers.Map('KeyType','double','ValueType','char');
    for i = 1:numel(blocks)
        h = getSimulinkBlockHandle(blocks{i});
        if h>0, h2p(h) = blocks{i}; end
    end

    % Build undirected graph via PortConnectivity
    edges = [];
    for i = 1:numel(blocks)
        h = getSimulinkBlockHandle(blocks{i});
        pc = get_param(h,'PortConnectivity');
        for k = 1:numel(pc)
            for m = 1:numel(pc(k).Connected)
                h2 = pc(k).Connected(m).SrcBlock;
                if h2>0 && h2~=h
                    edges(end+1,:) = sort([h h2]); %#ok<AGROW>
                end
            end
        end
    end
    edges = unique(edges,'rows');

    baseH  = getSimulinkBlockHandle(world{1});
    bodyH  = getSimulinkBlockHandle(body{1});

    pathH = bfs(baseH, bodyH, edges);
    assert(~isempty(pathH), 'No connection path from World to %s found.', bodyN);

    % Walk the path, capturing joints and composing fixed transforms
    chain = struct('joint',[],'X_parent_child',[]);
    T = eye(4); idx = 0;
    for k = 1:numel(pathH)
        cur = pathH(k);
        curPath = h2p(cur);
        mtype = get_param(cur,'MaskType');

        if any(strcmp(mtype, {'Revolute Joint','Prismatic Joint'}))
            % Start a new stage
            idx = idx + 1;
            chain(idx).joint = readJoint(curPath);
            chain(idx).X_parent_child = eye(4);
        elseif strcmp(mtype,'Rigid Transform')
            X = readRigid(curPath);
            T = T * X;  % for the overall base->bodyN at zero motion
            if idx>0
                chain(idx).X_parent_child = chain(idx).X_parent_child * X;
            end
        end
    end

    kin.chain = chain;
    kin.T_base_bodyN = T;
    kin.baseBlock = h2p(baseH);
    kin.bodyBlock = h2p(bodyH);
end

function path = bfs(startH, goalH, edges)
% Bare-bones BFS on block handles
    q = startH; head=1;
    prev = containers.Map('KeyType','double','ValueType','double');
    seen = containers.Map('KeyType','double','ValueType','logical'); seen(startH)=true;
    nbrs = @(h) unique([edges(edges(:,1)==h,2); edges(edges(:,2)==h,1)].');
    while head<=numel(q)
        cur = q(head); head=head+1;
        if cur==goalH, break; end
        for t = nbrs(cur)
            if ~isKey(seen,t), seen(t)=true; prev(t)=cur; q(end+1)=t; end %#ok<AGROW>
        end
    end
    if ~(exist('prev','var') && isKey(prev,goalH)), path=[]; return; end
    path = goalH;
    while path(1)~=startH, path = [prev(path(1)) path]; end %#ok<AGROW>
end

function J = readJoint(path)
% Pull axis/limits directly from the joint mask
    names = get_param(path,'MaskNames'); vals = get_param(path,'MaskValues');
    S = containers.Map; for i=1:numel(names), S(names{i}) = vals{i}; end
    mtype = get_param(path,'MaskType');
    axis  = tryVec(S, ["Axis","RevoluteAxis","PrismaticAxis","Axis of rotation","Axis of translation"]);
    hasL  = tryBool(S, ["SpecifyPositionLimits","Specify Limits","SpecifyPositionTargetAndLimits","SpecifyLimits"]);
    lo    = tryScalar(S, ["LowerLimit","Lower Position Limit","PositionLowerLimit","Lower limit"]);
    up    = tryScalar(S, ["UpperLimit","Upper Position Limit","PositionUpperLimit","Upper limit"]);
    units = tryText  (S, ["PositionUnits","Units","Position Target Units"]);
    if isempty(units), units = strcmpi(mtype,'Prismatic Joint')*"m" + strcmpi(mtype,'Revolute Joint')*"rad"; units = char(units); end
    [~, short] = fileparts(path);
    J = struct('path',path,'shortName',short,'type',mtype,'axis',axis, ...
        'hasLimits',logical(hasL),'lower',lo,'upper',up,'units',units);
end

function X = readRigid(path)
% Read a Rigid Transform block (handles common parameterizations).
    names = get_param(path,'MaskNames'); vals = get_param(path,'MaskValues');
    S = containers.Map; for i=1:numel(names), S(names{i}) = vals{i}; end

    % Translation
    t = tryVec(S, ["Translation","TranslationVector","Translation vector"]); if isempty(t), t=[0 0 0]; end

    % Rotation (try axis-angle first, then Euler)
    R = eye(3);
    ang = tryScalar(S, ["RotationAngle","Angle"]); axe = tryVec(S, ["RotationAxis","Axis"]);
    if ~isempty(ang) && ~isempty(axe)
        R = axang2rotm([axe(:).' ang]);
    else
        seq = upper(tryText(S, ["RotationSequence","Rotation Sequence"]));
        if isempty(seq), seq='ZYX'; end
        eul = tryVec(S, ["RotationAngles","Angles","EulerAngles"]); if isempty(eul), eul=[0 0 0]; end
        R = eul2rotm(eul(:).', seq);
    end

    X = eye(4); X(1:3,1:3)=R; X(1:3,4)=t(:);
end

% --- tiny parsing utilities ---
function v = tryVec(S, keys)
    v = [];
    for k=1:numel(keys)
        if isKey(S, keys(k))
            try
                vv = str2num(S(keys(k))); %#ok<ST2NM>
                if isnumeric(vv) && numel(vv)==3, v = vv(:).'; return; end
            end, v=[];
        end
    end
end
function x = tryScalar(S, keys)
    x = [];
    for k=1:numel(keys)
        if isKey(S, keys(k))
            xx = str2double(S(keys(k)));
            if isfinite(xx), x = xx; return; end
        end
    end
end
function t = tryText(S, keys)
    t = '';
    for k=1:numel(keys)
        if isKey(S, keys(k)), t = S(keys(k)); return; end
    end
end
function b = tryBool(S, keys)
    b = false;
    for k=1:numel(keys)
        if isKey(S, keys(k))
            v = lower(strtrim(S(keys(k))));
            b = any(strcmp(v, {'on','true','1','yes'})); return;
        end
    end
end

