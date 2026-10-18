function plot_mulag_path(p_ref, p_meas)
% EE_ref, EE_meas: N×3 arrays [x y z]

% If they came out as 3×N instead, transpose:
if size(p_ref,2) == 3 && size(p_ref,1) ~= size(p_meas,1)
    p_meas = p_meas.';
end
if size(p_ref,1) == 3 && size(p_ref,2) ~= 3
    p_ref  = p_ref.';
    p_meas = p_meas.';
end

xr = p_ref(:,1);  yr = p_ref(:,2);  zr = p_ref(:,3);
xm = p_meas(:,1); ym = p_meas(:,2); zm = p_meas(:,3);

figure; hold on; grid on; box on; axis equal

% Reference path (e.g. red)
plot3(xr, yr, zr, 'r', 'LineWidth', 1.5);

% Measured path (e.g. blue)
plot3(xm, ym, zm, 'b', 'LineWidth', 1.5);

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('End-effector trajectory');
legend('Reference path','Measured path','Location','best');
view(135, 25);   % similar “3D map” perspective
end
