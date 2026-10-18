function open_project()
%OPEN_PROJECT Open the editable integrated hydraulic manipulator model.
root = setup_project();
open_system(fullfile(root,'models','Mulag_FME_500_with_cylinder.slx'));
end
