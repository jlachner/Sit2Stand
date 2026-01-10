%% Setup
robot = iiwa14('high');
robot.init();

% Plotting settings
sz_label = 18;
sz_title = 20;
sz_axes = 16;

% Load experimental data
folder = 'robot_posCtrl/prints/2026-01-10_Stand2Sit_02/'; % % St2Si
% folder = 'robot_posCtrl/prints/2025-12-17_Stand2SitCable_02/'; % Si2St
q_exp = load(strcat(folder, 'File_q.txt'));
dq_exp = load(strcat(folder, 'File_dq.txt'));
F_exp = load(strcat(folder, 'File_FExt.txt'));
t_exp = load(strcat(folder, 'File_dt.txt'));

% Reshape data
q_exp = reshape(q_exp, [7, numel(q_exp)/7]);
dq_exp = reshape(dq_exp, [7, numel(dq_exp)/7]);
F_exp = reshape(F_exp, [6, numel(F_exp)/6]);
F_N = F_exp * 4.45; % convert from lbf to N
F_X = F_N(1, :);
F_Y = F_N(2, :);
F_Z = F_N(3, :);

% Visualization time
t_end = 2; % seconds
dt = 0.005; % period of data capture
Nt = t_end/dt;

% Plot force data
figure();
% plot(t_exp(1:Nt)/t_exp(Nt)*100, flip(F_N(1,1:Nt), 2), 'LineWidth', 2)
plot(t_exp, F_X, 'LineWidth', 2)
set(gca, 'FontSize', sz_axes);
% title('Measured Sit-to-Stand Force Profile', 'FontSize', sz_title);
title('Measured Stand-to-Sit Force Profile', 'FontSize', sz_title);
xlabel('Time (s)', 'FontSize', sz_label);
% xlabel('Stance Percentage (%)', 'FontSize', sz_label);
ylabel('Cable Tension (N)', 'FontSize', sz_label)
ylim([0, 100])


%% Convert joint angles to end effector Cartesian coordinates
% Preallocate
p_traj = zeros(3, Nt);
pointPos = [ 0, 0, 0.072 ]';

% Get end effector positions of text file
for i=1:Nt
    q = q_exp(:, i);
    H = robot.getForwardKinematics(q);
    p = H(1:3, 4);
    p_traj(:, i) = p;
end

X_traj = p_traj(1, :);
Y_traj = p_traj(2, :);
Z_traj = p_traj(3, :);

figure();
plot3(100*X_traj, 100*Y_traj, 100*Z_traj);
axis equal
xlabel('X (cm)')
ylabel('Y (cm)')
zlabel('Z (cm)')
title('End Effector Trajectory')

%% Plot experimental force vs. displacement
F_trial = F_N(1,1:Nt);
% X_traj = X_traj - min(X_traj); % offset to start at 0 displacement

figure();
plot(100*X_traj, F_trial, 'LineWidth', 2);
xlabel('End Effector X-position (cm)')
ylabel('Cable Tension (N)')
% title('Measured Sit-to-Stand Force Profile (Unwrapping)')
title('Measured Stand-to-Sit Force Profile')

%% Import and plot simulated force-displacement profile

load("simulation_data/si2st_force_63.mat")

figure();
plot(x_sim, F_sim, 'LineWidth', 2);
xlabel('Cable Displacement (cm)')
ylabel('Cable Tension (N)')
title('Simulated Sit-to-Stand Force Profile')

%% Plot experimental and simulated force profiles together
% offset and negate end-effector data so that pulling on the cable results
% in positive cable displacement, starting from 0
X_traj = max(X_traj) - X_traj;
X_trial = X_traj(1:Nt);

figure();
plot(X_trial, F_trial, 'LineWidth', 2)
hold on;
plot(x_sim, F_sim, 'LineWidth', 2);

%%
% % Example to get task space velocity of text file
% dq_exp = [-0.39, 0.49, 0.71, -0.70, 0.68, 0.05, 0.81]';                % Replace this with the joint velocities of your text files
% J_test = robot.getHybridJacobian(q, 'bodyID', 7, 'pointPos', pointPos );
% dx = J_test * dq_exp;