function root = setup_project()
%SETUP_PROJECT Set portable paths for the imported R2023b model.
root = fileparts(mfilename('fullpath'));
folders = {'models','models/variants','src','src/kinematics', ...
    'src/analysis','src/parameters','src/robot','robot/urdf', ...
    'robot/meshes/stl','data/references'};
for i = 1:numel(folders)
    addpath(fullfile(root, folders{i}));
end
cd(root); % Models use project-relative geometry and Signal Editor files.
build = fullfile(root, 'build');
if ~isfolder(build), mkdir(build); end
assert(exist('load_system','file') ~= 0, 'Simulink is required.');
assert(exist('smimport','file') ~= 0, 'Simscape Multibody is required.');
Simulink.fileGenControl('set', 'CacheFolder', fullfile(build,'cache'), ...
    'CodeGenFolder', fullfile(build,'codegen'), 'createDir', true);
fprintf('Project ready: %s\nOpen the .slx model with open_project.\n',root);
end
