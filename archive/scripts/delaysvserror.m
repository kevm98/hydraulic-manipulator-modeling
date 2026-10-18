%% Effect of bulk modulus on per-joint delay and EE tracking

% ---- Bulk modulus values (GPa) ----
beta   = [0.5 1.0 1.5 2.0];

% ---- Valve-to-joint 50% delays per joint [ms] ----
tau_J1 = [847.4 725.3 674.0 649.7];       % Joint 1
tau_J3 = [186.3 168.8 149.9 152.4];       % Joint 3
tau_J4 = [202.1 206.4 215.8 221.6];       % Joint 4

% ---- End-effector RMS tracking error [mm] ----
E_RMS  = [827.9 775.6 753.2 739.4];

% ============================================================
% Plot (publication style)
% ============================================================
fig = figure('Color','w', ...
             'Units','centimeters', ...
             'Position',[2 2 18 10]);      % width x height ~18x10 cm
hold on; grid on;

axisFont   = 13;   % tick labels
labelFont  = 15;   % x/y labels
titleFont  = 16;   % title
legendFont = 13;   % legend text
lw = 2.0;          % line width
ms = 8.5;          % marker size

%% Left axis: delays
yyaxis left
p1 = plot(beta, tau_J1, '-o', 'LineWidth', lw, 'MarkerSize', ms); hold on;
p3 = plot(beta, tau_J3, '-s', 'LineWidth', lw, 'MarkerSize', ms);
p4 = plot(beta, tau_J4, '-^', 'LineWidth', lw, 'MarkerSize', ms);

ylabel('Valve-to-joint 50% delay [ms]', ...
       'FontSize', labelFont, ...
       'Interpreter', 'none');
ylim([0 900]);

%% Right axis: EE RMS
yyaxis right
pEE = plot(beta, E_RMS, '--o', 'LineWidth', lw, 'MarkerSize', ms);

ylabel('End-effector RMS error [mm]', ...
       'FontSize', labelFont, ...
       'Interpreter', 'none');
ylim([700 900]);

%% Common x-axis and title
xlabel('Bulk modulus [GPa]', ...
       'FontSize', labelFont, ...
       'Interpreter', 'none');

xlim([0.45 2.05]);

title('Effect of bulk modulus on joint delays and EE tracking', ...
      'FontSize', titleFont, ...
      'FontWeight', 'bold', ...
      'Interpreter', 'none');

set(gca, 'FontSize', axisFont, ...    % tick label font size
         'LineWidth', 1.2, ...
         'Box', 'on');

%% Legend (plain text, big font)
yyaxis left
leg = legend([p1 p3 p4 pEE], ...
             {'J1 delay', 'J3 delay', 'J4 delay', 'EE RMS error'}, ...
             'Location', 'northeast', ...
             'Interpreter', 'none');
set(leg, 'FontSize', legendFont);

% Optional: high-res export for LaTeX
% print(fig, 'J3bulkee', '-dpng', '-r600');
