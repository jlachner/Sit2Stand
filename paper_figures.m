%% Setup
close all; clear; clc;

% Plot settings
plt = struct;
plt.label = 18;
plt.legend = 16;
plt.title = 20;
plt.axes = 16;
plt.lwidth = 4;

% plt.label = 10;
% plt.legend = 8;
% plt.title = 12;
% plt.axes = 8;
% plt.lwidth = 2;

% plt.label = 10*2;
% plt.legend = 8*2;
% plt.title = 12*2;
% plt.axes = 8*2;
% plt.lwidth = 2;

plt.c_ankle = [42 191 117]/255; % ankle angle; light green
plt.c_knee = [116 42 191]/255; % knee angle; purple
plt.c_pelvis = [191 42 116]/255; % pelvis angle; warm brown
plt.c_compound = [90 119 82]/255; % compound bow; camo green
% plt.c_straight = [54 57 61]/255; % straight bow; camo slate
plt.c_straight = [102 90 62]/255; % straight bow; camo brown

plt.c_stor = [41 140 140]/255; % storing energy; teal
plt.c_rel = [241 162 38]/255; % releasing energy; gold
plt.c_sim = [53 148 204]/255; % simulation; medium blue
plt.c_stor_cam = [94 94 94]/255; % storage cam color; dark gray

plt.mkr = '.';
plt.sz = 150;

% plt.w = 3.25 * 72 * 2;
% plt.h = 2.3 * 72 * 2;

%% Figure: Joint Angles (Adapted from Chugo et al. 2006)
% Import extracted data.
[ankle_pct, ankle_theta, knee_pct, knee_theta, pelvis_pct, pelvis_theta] = readvars('figure_data/Chugo_2006_Fig-5-b.csv');

% Sort data according to ascending movement percentage.
ankle_sorted = sortrows([ankle_pct, ankle_theta], 1);
knee_sorted = sortrows([knee_pct, knee_theta], 1);
pelvis_sorted = sortrows([pelvis_pct, pelvis_theta], 1);

% Plot.
figure();
plot(ankle_sorted(:,1), ankle_sorted(:,2), 'LineWidth', plt.lwidth, 'Color', plt.c_ankle)
hold on;
plot(knee_sorted(:,1), knee_sorted(:,2), 'LineWidth', plt.lwidth, 'Color', plt.c_knee)
plot(pelvis_sorted(:,1), pelvis_sorted(:,2), 'LineWidth', plt.lwidth, 'Color', plt.c_pelvis)
set(gca, 'FontSize', plt.axes)
xlabel('Movement Pattern (%)', 'FontSize', plt.label)
ylabel('Angle (degrees)', 'FontSize', plt.label)
% title('Joint Angles During Sit-to-Stand', 'FontSize', plt.title)
legend('Ankle', 'Knee', 'Pelvis', 'FontSize', plt.label, 'Location', 'northwest')

%% Figure: Compound Bow (Adapted from Aronson 1977)
% Import extracted data.
[compound_x, compound_F, straight_x, straight_F] = readvars('figure_data/Aronson_1977_compound-bow.csv');

% Sort data according to ascending draw length.
compound_sorted = sortrows([compound_x, compound_F], 1);
straight_sorted = sortrows([straight_x, straight_F], 1);

% Plot.
figure();
plot(compound_sorted(:,1), compound_sorted(:,2), 'LineWidth', plt.lwidth, 'Color', plt.c_compound)
hold on;
plot(straight_sorted(:,1), straight_sorted(:,2), '-.', 'LineWidth', plt.lwidth, 'Color', plt.c_straight)
set(gca, 'FontSize', plt.axes)
xlabel('Draw Length (in.)', 'FontSize', plt.label)
ylabel('Draw Load (lbf)', 'FontSize', plt.label)
% title('Bow Force-Displacement Profiles', 'FontSize', plt.title)
legend('Compound Bow', 'Straight Bow', 'FontSize', plt.legend, 'Location', 'northwest')

%% Figure: Desired Force Trajectories

% Sit-to-stand
x_si2st = [0, 20, 32.5, 40, 60, 80, 100];
F_si2st = [0, 25, 100,  20, 10, 5,  0];
E_si2st = cumtrapz(x_si2st, F_si2st);

% Stand-to-sit
x_st2si = [0, 20, 32.5, 40, 60, 80, 100];
F_st2si = [0, 20, 50,   37.5, 22.5, 10, 0];
E_st2si = cumtrapz(x_st2si, F_st2si);

figure();
% figure('Position', [500, 500, plt.w plt.h]);
plot(x_st2si, F_st2si, 'LineWidth', plt.lwidth, 'Color', plt.c_stor)
hold on;
plot(x_si2st, F_si2st, 'LineWidth', plt.lwidth, 'Color', plt.c_rel)
set(gca, 'FontSize', plt.axes)
xlabel('Stance Percentage (%)', 'FontSize', plt.label)
ylabel('Cable Tension (N)', 'FontSize', plt.label)
% legend('Stand-to-Sit (Energy Storage)', 'Sit-to-Stand (Energy Release)', ...
%     'FontSize', plt.label, 'Location', 'northeast')
legend('St-Si (Energy Storage)', 'Si-St (Energy Release)', ...
    'FontSize', plt.legend, 'Location', 'northeast')

%% Figure: Generated Cam Shapes
% plt.label = 27;
% plt.axes = 24;
% plt.legend = 24;


% Import sit-to-stand cam text files
cam_pts_si2st = load('simulation_data/inner_63.txt');
cam_pts_outer = load('simulation_data/outer_63.txt');
cam_pts_st2si = load('simulation_data/st2si_63.txt');

% Convert points to centimeters
cam_pts_si2st = cam_pts_si2st/10;
cam_pts_outer = cam_pts_outer/10;
cam_pts_st2si = cam_pts_st2si/10;

plt.alpha = 1;

figure();
scatter(cam_pts_outer(:,1), cam_pts_outer(:,2), plt.sz, plt.c_stor_cam, plt.mkr)
hold on;
scatter(cam_pts_st2si(:,1), cam_pts_st2si(:,2), plt.sz, plt.c_stor, plt.mkr)
    % 'MarkerFaceAlpha', plt.alpha, 'MarkerEdgeAlpha', plt.alpha)
scatter(cam_pts_si2st(:,1), cam_pts_si2st(:,2), plt.sz, plt.c_rel, plt.mkr, ...
    'MarkerFaceAlpha', plt.alpha, 'MarkerEdgeAlpha', plt.alpha)
scatter(0, 0, 200, 'k', 'o', 'LineWidth', 1.5)
scatter(0, 0, 300, 'k', '+', 'LineWidth', 1.5)
axis equal
set(gca, 'FontSize', plt.axes);
xlabel('x (cm)', 'FontSize', plt.label)
ylabel('y (cm)', 'FontSize', plt.label)
% legend('Storage Cam', 'Transmission Cam', 'FontSize', plt.legend)
legend('Storage Cam', 'St-Si Cam', ...
    'Si-St Cam', 'FontSize', plt.legend, ...
    'Location', 'northeast')
% title('Sit-to-Stand Cam Shapes','FontSize',plt.title)

%%
% Import stand-to-sit cam text files
cam_pts_st2si = load('simulation_data/st2si_63.txt');
cam_pts_st2si = cam_pts_st2si/10; % convert to cm

figure();
scatter(cam_pts_outer(:,1), cam_pts_outer(:,2), plt.sz, plt.c_stor_cam, plt.mkr)
hold on;
scatter(cam_pts_st2si(:,1), cam_pts_st2si(:,2), plt.sz, plt.c_stor, plt.mkr)
scatter(0, 0, 200, 'k', 'o', 'LineWidth', 1.5)
scatter(0, 0, 300, 'k', '+', 'LineWidth', 1.5)
axis equal
set(gca, 'FontSize',plt.axes);
xlabel('x (cm)', 'FontSize',plt.label)
ylabel('y (cm)', 'FontSize',plt.label)
legend('Storage Cam', 'Transmission Cam', 'FontSize', plt.legend)
title('Stand-to-Sit Cam Shapes','FontSize',plt.title)