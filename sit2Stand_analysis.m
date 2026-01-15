%% Setup
close all; clear; clc;

% Initialize KUKA iiwa robot using Exp[licit]
robot = iiwa14('high');
robot.init();

% Plot settings
plt = struct;
plt.label = 18;
plt.title = 20;
plt.axes = 16;
plt.lwidth = 4;
plt.c_stor = '#298C8C'; % storing energy; teal
plt.c_rel = '#F1A226'; % releasing energy; gold
plt.c_sim = '#3594CC'; % simulation; medium blue
% plt.c_stor = '#D93600'; % orange
% plt.c_rel = '#1a80bb'; % blue

% Specify locations of data for experimental and simulated sit-to-stand and 
% stand-to-sit force profiles
% folder.si2st_exp = 'robot_posCtrl/prints/2026-01-10_Sit2Stand_02/'; % paper 1st draft
folder.st2si_exp = 'robot_posCtrl/prints/2026-01-10_Stand2Sit_03/'; % paper 1st draft
folder.si2st_exp = '2026-01-14_1-5s-duration/';
folder.si2st_sim = 'simulation_data/si2st_force_63.mat';
folder.st2si_sim = 'simulation_data/st2si_force.mat';

% Load experimental joint and force data and convert to Cartesian space
[p_traj_si2st, F_si2st, t_si2st] = q_convert(folder.si2st_exp, robot); % Si2St
[p_traj_st2si, F_st2si, t_st2si] = q_convert(folder.st2si_exp, robot); % St2Si

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plotting

% Plot predicted force profiles
sim_si2st = load(folder.si2st_sim);
sim_st2si = load(folder.st2si_sim);

% Force vs. stance percentage
figure();
plot(sim_st2si.pct, sim_st2si.F, ':', 'LineWidth', plt.lwidth, 'Color', plt.c_stor)
hold on;
plot(sim_si2st.pct, sim_si2st.F, ':', 'LineWidth', plt.lwidth, 'Color', plt.c_rel)
set(gca, 'FontSize', plt.axes);
xlabel('Stance Percentage (%)', 'FontSize', plt.label)
ylabel('Cable Tension (N)', 'FontSize', plt.label)
% title('Simulated Sit-to-Stand and Stand-to-Sit Force Profiles', ...
%     'FontSize', plt.title)
legend('Stand-to-Sit (Storage)', 'Sit-to-Stand (Release)', ...
    'Location', 'northeast', 'FontSize', plt.axes)

% Force vs. cable displacement
figure();
plot(100*sim_st2si.x, sim_st2si.F, ':', 'LineWidth', plt.lwidth, 'Color', plt.c_stor)
hold on;
plot(100*sim_si2st.x, sim_si2st.F, ':', 'LineWidth', plt.lwidth, 'Color', plt.c_rel)
set(gca, 'FontSize', plt.axes);
xlabel('Transmission Cable Displacement (cm)', 'FontSize', plt.label)
ylabel('Cable Tension (N)', 'FontSize', plt.label)
% title('Simulated Sit-to-Stand and Stand-to-Sit Force Profiles', ...
%     'FontSize', plt.title)
legend('Stand-to-Sit (Storage)', 'Sit-to-Stand (Release)', ...
    'Location', 'northwest', 'FontSize', plt.axes)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Plot experimental force profiles
plot_fcn(p_traj_si2st, F_si2st, t_si2st, 'Sit-to-Stand', folder, plt)
plot_fcn(p_traj_st2si, F_st2si, t_st2si, 'Stand-to-Sit', folder, plt)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Analysis

% Pre-process x-direction data from raw trajectory
x_si2st = process_x(p_traj_si2st);
x_st2si = process_x(p_traj_st2si);

% Analyze key points and store results in tables
results_si2st = analysis(sim_si2st, F_si2st, x_si2st);
results_st2si = analysis(sim_st2si, F_st2si, x_st2si);

writetable(results_si2st, 'results/results_si2st.csv', 'WriteRowNames', true);
writetable(results_st2si, 'results/results_st2si.csv', 'WriteRowNames', true);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Functions
function [p_traj, F_mag, t_exp] = q_convert(folder, robot)
% Q_CONVERT Convert robot joint angles to end effector Cartesian
% coordinates.

    % Load experimental data
    q_exp = load(strcat(folder, 'File_q.txt'));
    F_exp = load(strcat(folder, 'File_FExt.txt'));
    t_exp = load(strcat(folder, 'File_dt.txt'));

    % Reshape data
    q_exp = reshape(q_exp, [7, numel(q_exp)/7]);
    % dq_exp = reshape(dq_exp, [7, numel(dq_exp)/7]);
    F_exp = reshape(F_exp, [6, numel(F_exp)/6]);
    F_N = F_exp * 4.45; % convert from lbf to N

    % Extract force by direction.
    F_X = F_N(1, :);
    F_Y = F_N(2, :);
    F_Z = F_N(3, :);

    % Calculate total magnitude of force at each point along trajectory.
    F_mag = vecnorm(F_N(1:3,:));

    % Visualization time
    % t_end = 6;      % seconds
    % dt = 0.005;     % period of data capture
    % t_end = t_exp(end);
    % dt = t_exp(2) - t_exp(1);
    % Nt = t_end/dt + 1;  % number of time points
    Nt = length(t_exp);
    
    % Preallocate
    p_traj = zeros(3, Nt);
    
    % Convert joint angles to end effector positions
    for i=1:Nt
        q = q_exp(:, i);
        H = robot.getForwardKinematics(q);
        p = H(1:3, 4);
        p_traj(:, i) = p;
    end

end

function plot_fcn(p_traj, F_mag, t_exp, profile_name, folder, plt)
% PLOT_FCN Generate plots for predicted and measured force profiles. Input
% determines whether plot is for sit-to-stand or stand-to-sit profile.

    % Extract trajectories in each direction and convert to centimeters.
    x_traj = 100 * p_traj(1, :);
    y_traj = 100 * p_traj(2, :);
    z_traj = 100 * p_traj(3, :);

    % Plot end-effector trajectory.
    figure();
    plot3(x_traj, y_traj, z_traj);
    axis equal
    xlabel('X (cm)')
    ylabel('Y (cm)')
    zlabel('Z (cm)')
    title('End Effector Trajectory')
    
    % Plot raw experimental force vs. time data.
    figure();
    plot(t_exp, F_mag, 'LineWidth', plt.lwidth)
    % plot(t_exp(1:Nt)/t_exp(Nt), flip(F_mag(1,1:Nt), 2), 'LineWidth', plt.lwidth)
    set(gca, 'FontSize', plt.axes);
    title(['Measured ', profile_name, ' Force Profile'], 'FontSize', plt.title);
    xlabel('Time (s)', 'FontSize', plt.label);
    ylabel('Cable Tension (N)', 'FontSize', plt.label)
    ylim([0, 100])
    
    %% Plot experimental force-displacement profile
    % Offset end-effector data so that pulling on the cable results in 
    % positive cable displacement, starting from 0.
    x_traj = x_traj - max(x_traj);

    % Normalize trajectory in x-direction to plot against normalized stroke
    % length instead of absolute displacement.
    x_traj = normalize(x_traj, "range");
    
    % Plot only unwrapping portion of data.
    % Nt_i = 801;
    % Nt_f = 1200;
    Nt_i = 1;
    Nt_f = length(F_mag);
    F_trial = F_mag(Nt_i:Nt_f);
    x_trial = x_traj(Nt_i:Nt_f);
    half_ind = round((Nt_f - Nt_i)/2);
    
    figure();
    % plot(x_trial, F_trial, 'LineWidth', plt.lwidth); % whole trajectory in 1 color
    plot(x_trial(1:half_ind), F_trial(1:half_ind), ...
        'LineWidth', plt.lwidth, 'Color', plt.c_stor); % storage color
    hold on;
    plot(x_trial(half_ind+1:end), F_trial(half_ind+1:end), ...
        'LineWidth', plt.lwidth, 'Color', plt.c_rel); % release color
    set(gca, 'FontSize', plt.axes);
    xlabel('End Effector X-position (cm)')
    ylabel('Cable Tension (N)')
    title(['Measured ', profile_name, ' Force Profile (X-Direction)'])

    %% Import and plot simulated force-displacement profiles
    % Import
    if strcmp(profile_name, 'Sit-to-Stand')
        sim = load(folder.si2st_sim);
    else
        sim = load(folder.st2si_sim);
    end
    
    % Convert simulated displacement to centimeters
    sim.x = 100 * sim.x;

    % Normalize simulated displacement.
    sim.x = normalize(sim.x, "range");

    % Plot simulated profile alone
    figure();
    plot(sim.x, sim.F, 'LineWidth', plt.lwidth);
    set(gca, 'FontSize', plt.axes);
    xlabel('Cable Displacement (cm)')
    ylabel('Cable Tension (N)')
    % title(['Simulated ', profile_name, ' Cam Force Profile'])
    
    %% Plot experimental and simulated force-displacement profiles together
    figure();
    plot(x_trial(1:half_ind), F_trial(1:half_ind), 'LineWidth', plt.lwidth, ...
        'Color', plt.c_stor); % plot in storage color
    hold on;
    plot(x_trial(half_ind+1:end), F_trial(half_ind+1:end), 'LineWidth', plt.lwidth, ...
        'Color', plt.c_rel); % plot in storage color
    % plot(x_trial, F_trial, 'LineWidth', plt.lwidth, 'Color', plt.c_rel)
    % hold on;
    plot(sim.x, sim.F, ':', 'LineWidth', plt.lwidth, 'Color', plt.c_sim);
    set(gca, 'FontSize', plt.axes);
    xlabel('Normalized Cable Stroke Distance', 'FontSize', plt.label)
    % xlabel('Cable Displacement (cm)', 'FontSize', plt.label)
    ylabel('Cable Tension (N)', 'FontSize', plt.label)
    % title(['Measured and Predicted ', profile_name, ' Force Profiles'], ...
    %     'FontSize', plt.title)
    legend('Measured, Storage', 'Measured, Release', 'Predicted', ...
        'Location', 'northwest', 'FontSize', plt.axes)
    ylim([0 100])

end

function x_traj = process_x(p_traj)
% PROCESS_X Offset and negate end-effector data so that pulling on the 
% cable results in positive cable displacement, starting from 0. Convert to
% centimeters.

    x_traj = 100 * p_traj(1, :);
    x_traj = max(x_traj) - x_traj;

end

function results_table = analysis(sim, F_exp, x_exp)
% ANALYSIS Analyze force and displacement values at key points: peak 
% assistance and when seated
    
    % Divide experimental data into storage and release phases
    half_ind = round(length(F_exp)/2);
    F_stor = F_exp(1:half_ind);
    x_stor = x_exp(1:half_ind);
    F_rel = F_exp(half_ind+1:end);
    x_rel = x_exp(half_ind+1:end);

    % Storage phase
    [F_peak_stor, x_peak_stor, F_seat_stor, x_seat_stor] = analysis_calcs(F_stor, x_stor);

    % Release phase
    [F_peak_rel, x_peak_rel, F_seat_rel, x_seat_rel] = analysis_calcs(F_rel, x_rel);

    % Simulated
    sim.x = 100 * sim.x;
    [F_peak_sim, x_peak_sim, F_seat_sim, x_seat_sim] = analysis_calcs(sim.F, sim.x);

    % Calculate experimental percent difference from prediction
    % Storage phase
    diff_F_peak_stor = (F_peak_sim - F_peak_stor) / F_peak_sim * 100;
    diff_x_peak_stor = (x_peak_sim - x_peak_stor) / x_peak_sim * 100;

    diff_F_seat_stor = (F_seat_sim - F_seat_stor) / F_seat_sim * 100;
    diff_x_seat_stor = (x_seat_sim - x_seat_stor) / x_seat_sim * 100;

    % Release phase
    diff_F_peak_rel = (F_peak_sim - F_peak_rel) / F_peak_sim * 100;
    diff_x_peak_rel = (x_peak_sim - x_peak_rel) / x_peak_sim * 100;

    diff_F_seat_rel = (F_seat_sim - F_seat_rel) / F_seat_sim * 100;
    diff_x_seat_rel = (x_seat_sim - x_seat_rel) / x_seat_sim * 100;

    % Store all calculated values in a table
    results_table = table([F_peak_sim; F_peak_stor; diff_F_peak_stor; F_peak_rel; diff_F_peak_rel], ...
                          [x_peak_sim; x_peak_stor; diff_x_peak_stor; x_peak_rel; diff_x_peak_rel], ...
                          [F_seat_sim; F_seat_stor; diff_F_seat_stor; F_seat_rel; diff_F_seat_rel], ...
                          [x_seat_sim; x_seat_stor; diff_x_seat_stor; x_seat_rel; diff_x_seat_rel], ...
                          'VariableNames', {'F_peak', 'X_Peak', 'F_Seat', 'X_Seat'}, ...
                          'RowNames', {'Predicted', 'Measured (Storage)', '% Difference (Storage)', ...
                          'Measured (Release)', '% Difference (Release)'});

    function [F_peak, x_peak, F_seat, x_seat] = analysis_calcs(F, x)
    
        [F_peak, F_peak_ind] = max(F);
        x_peak = x(F_peak_ind);
        
        [F_seat, F_seat_ind] = max([F(1), F(end)]);
        if F_seat_ind == 1
            x_seat = x(1);
        else
            x_seat = x(end);
        end

    end

end