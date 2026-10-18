function out = run_simulation()
%RUN_SIMULATION Run the imported main model and return its SimulationOutput.
% Requires licensed MATLAB/Simulink/Simscape/Multibody/Fluids installation.
root = setup_project();
load_system(fullfile(root,'models','Mulag_FME_500_with_cylinder.slx'));
out = sim('Mulag_FME_500_with_cylinder');
end
