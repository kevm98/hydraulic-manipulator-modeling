% Mower_Analysis_J1_J3_Detail.m
clc; clear; close all;

%% 1. DATA ENTRY (From Table I in the Paper)
beta = [0.5, 1.0, 1.5, 2.0]; % Bulk Moduli [GPa]

% Valve-to-Joint 50% Delays [ms]
delay_J1 = [847.4, 725.3, 674.0, 649.7]; 
delay_J3 = [186.3, 168.8, 149.9, 152.4]; 

% Grouping for loop
all_delays = {delay_J1, delay_J3};
titles = {'Joint 1: Swivel Rotation (Worst Case)', 'Joint 3: Boom Lift (Best Case)'};

%% 2. PLOTTING SETUP
figure('Color', 'w', 'Position', [100 200 1000 500]); % 2-Panel Layout

% Distinct Colors for clarity
colors = [0 0.4470 0.7410;   % 0.5 GPa (Blue)
          0.8500 0.3250 0.0980; % 1.0 GPa (Red)
          0.9290 0.6940 0.1250; % 1.5 GPa (Yellow)
          0.4940 0.1840 0.5560]; % 2.0 GPa (Purple)

t = 0:0.001:1.5; % Time vector (0 to 1.5s)

%% 3. GENERATE SUBPLOTS
for j = 1:2
    subplot(1, 2, j);
    hold on;
    
    current_data = all_delays{j};
    
    for i = 1:4
        % Get target delay in seconds
        Td = current_data(i) / 1000;
        
        % --- PHYSICS ENGINE (Visual Reconstruction) ---
        % We use a 2nd-order underdamped system to mimic heavy hydraulic arms.
        % This creates a realistic "S-curve" rather than a straight line.
        zeta = 0.85; % Damping ratio (Hydraulics are strictly damped)
        
        % Solver: Find 'wn' (Natural Freq) so the curve hits 0.5 exactly at Td
        target_func = @(w) (1 - (1/sqrt(1-zeta^2)) * exp(-zeta*w*Td) * sin(w*Td*sqrt(1-zeta^2) + acos(zeta))) - 0.5;
        try
            wn = fzero(target_func, [0.5, 50]);
        catch
            wn = 10; % Fallback
        end
        
        % Generate the Curve
        y = 1 - (1/sqrt(1-zeta^2)) * exp(-zeta*wn*t) .* sin(wn*t*sqrt(1-zeta^2) + acos(zeta));
        
        % Plot the Trajectory Line
        p = plot(t, y, 'LineWidth', 2.5, 'Color', colors(i,:));
        
        % Plot the "50% Marker" (The Proof)
        plot(Td, 0.5, 'o', 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
        
        % Add vertical drop-line to X-axis to emphasize the time difference
        plot([Td Td], [0 0.5], ':', 'Color', colors(i,:), 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
    
    % --- FORMATTING ---
    yline(0.5, '--k', '50% Threshold', 'LabelHorizontalAlignment', 'left', 'FontSize', 10, 'HandleVisibility', 'off');
    title(titles{j}, 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Time [s]');
    ylabel('Normalized Position');
    grid on;
    ylim([-0.05 1.1]);
    
    % Zoom X-axis differently for J1 vs J3 because J3 is much faster
    if j == 1
        xlim([0 1.2]); % Joint 1 is slow (up to 0.8s delay)
        legend({'\beta=0.5 (Soft)', '\beta=1.0 (Nominal)', '\beta=1.5 (Stiff)', '\beta=2.0 (Very Stiff)'}, ...
               'Location', 'southeast', 'FontSize', 9);
    else
        xlim([0 0.6]); % Joint 3 is fast (delay around 0.15s)
    end
end

sgtitle('Sensitivity of Actuation Delay to Bulk Modulus', 'FontSize', 14, 'FontWeight', 'bold');