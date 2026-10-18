function reset_hydraulic_session(MODEL)
% Full Simulink/Simscape reset + safe file-gen folders + clean rebuild.
% Usage:
%   reset_hydraulic_session            % uses default model name (below)
%   reset_hydraulic_session('YourModel')

% --------- 1) Model name ----------
if nargin < 1 || isempty(MODEL)
    MODEL = 'Mulag_FME_500_with_cylinder';
end

fprintf('\n=== RESET HYDRAULIC SESSION: %s ===\n', MODEL);

% --------- 2) Stop & close everything ----------
try, set_param(0,'SimulationCommand','stop'); catch, end
bdclose('all');  % close models without saving

% --------- 3) Clear ALL executable state (kills persistent in functions) ----------
clear classes;      % clears class definitions
clear functions;    % clears persistent variables in MATLAB functions
clear mex;          % unloads MEX files
clc;

% --------- 4) Clean Simulink/Simscape caches ----------
% Put cache/codegen folders in a SAFE location (not system temp).
userDocs = fullfile(getenv('USERPROFILE'),'Documents','MATLAB');
cacheDir = fullfile(userDocs,'slcache');
codeDir  = fullfile(userDocs,'slcodegen');
if ~exist(cacheDir,'dir'), mkdir(cacheDir); end
if ~exist(codeDir,'dir'),  mkdir(codeDir);  end

% Reset file-gen control then set to safe folders
Simulink.fileGenControl('reset');
Simulink.fileGenControl('set', ...
    'CacheFolder',   cacheDir, ...
    'CodeGenFolder', codeDir);

% Clean derived data cache
try
    Simulink.cache.cleanup;
% catch ME
%     warning('Simulink.cache.cleanup warning: %s', E.message);
end

% --------- 5) Delete local generated folders in the project/model tree ----------
% % Try to locate the model file; otherwise, use the current folder
% mdlPath = which([MODEL 'Mulag_FME_500_with_cylinder.slx']);
% if isempty(mdlPath), mdlPath = which([MODEL '.mdl']); end
% if isempty(mdlPath)
%     rootDir = pwd;
% else
%     rootDir = fileparts(mdlPath);
% end
% 
% toZap = { ...
%     fullfile(rootDir,'slprj'), ...
%     fullfile(rootDir,'sscprj'), ...
%     fullfile(rootDir,'codegen'), ...
%     fullfile(rootDir,'work')};
% 
% for k = 1:numel(toZap)
%     if exist(toZap{k},'dir')
%         fprintf(' - Removing %s ...\n', toZap{k});
%         try
%             rmdir(toZap{k},'s');
%         catch ME
%             warning('Could not remove %s: %s', toZap{k}, E.message);
%         end
%     end
% end

% Simscape specific cleanup (if available)
% if exist('ssc_clean','file') == 2
%     try
%         fprintf(' - Running ssc_clean on %s ...\n', rootDir);
%         ssc_clean(rootDir);
%     catch ME
%         warning('ssc_clean warning: %s', E.message);
%     end
% end

% --------- 6) Reopen model and set safe compile settings ----------
load_system(MODEL);

% Make compilation as benign as possible for the first build
try
    % Conservative solver & options (adjust if you already have tuned setup)
    set_param(MODEL, ...
        'StopTime',          '1', ...            % quick compile pass
        'SolverType',        'Variable-step', ...
        'Solver',            'ode23t', ...       % good for stiff hydraulics
        'MaxOrder',          '2', ...
        'ZeroCrossControl',  'DisableAll', ...
        'ConsistencyChecking','none', ...
        'EnablePacing',      'off', ...
        'SignalResolutionControl','UseLocalSettings');
catch ME
    warning('Could not set solver/options: %s', E.message);
end

% --------- 7) Force a clean compile of the Simscape network ----------
% We just compile; we don’t need to run a full sim here.
try
    fprintf(' - Compiling model (no run)...\n');
    feval(MODEL, [], [], [], 'compile');
    feval(MODEL, [], [], [], 'term');
% catch ME
%     warning('Compile step reported: %s', ME.message);
end

% --------- 8) Open the top-level model window ----------
open_system(MODEL);

fprintf('=== RESET COMPLETE. Model reopened: %s ===\n\n', MODEL);
end
