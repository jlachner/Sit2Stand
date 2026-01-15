%% Clean up
clear; clc; close all;

%% Setup
robot = iiwa14('high');
robot.init();

% Initialize Animation
anim = Animation('Dimension', 3, ...
    'xLim', [-0.2, 1.2], 'yLim', [-0.7, 0.7], 'zLim', [-0.5, 1.2], ...
    'isSaveVideo', false, 'VideoSpeed', 1.0);
anim.init();
anim.attachRobot(robot);

% Line parameters, stand-to-sit
A = 0.05;           % distance (meters)
T_sit = 2;          % period for one revolution
omega_sit = 2 * pi / T_sit;
dt = 0.005;
N_sit = round(T_sit / dt);

% "Hold" time before release
T_hold = 2;
N_hold = round (T_hold / dt);

% Line parameters, sit-to-stand
T_stand = 0.25;          % period for one revolution
omega_stand = 2 * pi / T_stand;
N_stand = round(T_stand / dt);

% Total recorded time
T_tot = T_sit + T_hold + T_stand;

% Timing
dt = 0.005;                         
% Ntot = round(T_tot / dt);
N_tot = N_sit + N_hold + N_stand;
t_record = linspace(0, T_tot, N_tot);

% Initial joint configuration
% q = deg2rad([-86.91, 106.55, 26.71, -58.99, 65.13, 70.34, 22.97]'); % stand-to-sit groove aligned
% q = deg2rad([-86.98, 105.51, 26.71, -60.28, 65.55, 69.99, 22.75]'); % sit-to-stand groove aligned
q = deg2rad([-101.06, 104.70, -0.01, -53.22, -113.81, 81.67, -18.19]'); % table against wall, sit-to-stand groove aligned

H0 = robot.getForwardKinematics(q); 
R0 = H0(1:3, 1:3);                      % Starting orientation    
p0 = H0(1:3, 4);                         % Starting EE position
p_des = p0;
p_des_ee_ini = R0 * p_des;
p_des_ee = p_des_ee_ini;

% Example to get joint positions of text file
q_test = q;                                             % Replace this with the joint configurations of your text files
pointPos = [ 0, 0, 0.072 ]';
H_test = robot.getHybridJacobian(q, 'bodyID', 7, 'pointPos', pointPos );
p_test = H_test(1:3,4);
R_test = H_test(1:3,1:3);

% Example to get task space velocity of text file
dq_test = [-0.39, 0.49, 0.71, -0.70, 0.68, 0.05, 0.81]';                % Replace this with the joint velocities of your text files
J_test = robot.getHybridJacobian(q, 'bodyID', 7, 'pointPos', pointPos );
dx = J_test * dq_test;

% Get Jacobian and extract translational part
J = robot.getHybridJacobian(q);         % 6x7
Jv = J(1:3, :);                          % translational velocities

% Preallocate
z_des_ee = 0;
q_record = zeros(robot.nq, N_tot);
p_traj = zeros(3, N_tot);
p_des_traj = zeros(3, N_tot);

% Loop over time steps
% 1: Stand-to-sit (pull on transmission cable)
for i = 1:N_sit
    t = t_record(i);
    
    % Move along +z in ee coords
    p_des_ee(3) = p_des_ee_ini(3) + A / 2 * (1 - cos( 0.5 * omega_sit * t ));
    p_des = R0' * p_des_ee;
    p_des_traj(:, i) = p_des;

    % Current EE pose
    H = robot.getForwardKinematics(q);
    p = H(1:3, 4);
    p_traj(:, i) = p;

    % Compute twist (task-space velocity)
    dp = (p_des - p) / dt;
    w = zeros(3,1);  % fixed orientation
    twist = [dp; w];

    % Inverse Kinematics: least-square dynamically consistent
    J = robot.getHybridJacobian(q);
    M = robot.getMassMatrix(q);
    M_inv = M \ eye(size(M));
    k = 0.01;
    Lambda_inv = J * M_inv * J' + k^2 * eye(6);
    Lambda = Lambda_inv \ eye(6);
    J_inv = M_inv * J' * Lambda;

    dq = J_inv * twist;
    q = q + dq * dt;

    q_record(:, i) = q;
    t = t + dt;

    robot.updateKinematics(q);
    anim.update(t);
end

% 2: Hold position for N_hold time steps
for i = N_sit + 1 : N_sit + N_hold
    t = t_record(i);
    q_record(:, i) = q_record(:, N_sit);
    t = t + dt;

    % Updated desired and actual current EE pose
    p_des_traj(:, i) = p_des;
    H = robot.getForwardKinematics(q);
    p = H(1:3, 4);
    p_traj(:, i) = p;

    robot.updateKinematics(q);
    anim.update(t);
end


% 3: Sit-to-stand (release transmission cable)
for i = N_sit + N_hold + 1 : N_tot
    t = t_record(i);
    
    % Move along -z in ee coords
    % p_des_ee(3) = p_des_ee_ini(3) + A * sin( 0.25 * omega_sit * t_record(N_sit) ) ... % add amplitude of motion after sitting
    %     + A * sin(0.25 * omega_stand * (t - t_record(N_sit + N_hold) + T_stand / 4)) ; % zero out the time for stand frequency
    p_des_ee(3) = p_des_ee_ini(3) + A / 2 * (1 - cos(0.5 * omega_stand * (t - t_record(N_sit + N_hold) + T_stand))) ; %
    p_des = R0' * p_des_ee;
    p_des_traj(:, i) = p_des;

    % Current EE pose
    H = robot.getForwardKinematics(q);
    p = H(1:3, 4);
    p_traj(:, i) = p;

    % Compute twist (task-space velocity)
    dp = (p_des - p) / dt;
    w = zeros(3,1);  % fixed orientation
    twist = [dp; w];

    % Inverse Kinematics: least-square dynamically consistent
    J = robot.getHybridJacobian(q);
    M = robot.getMassMatrix(q);
    M_inv = M \ eye(size(M));
    k = 0.01;
    Lambda_inv = J * M_inv * J' + k^2 * eye(6);
    Lambda = Lambda_inv \ eye(6);
    J_inv = M_inv * J' * Lambda;

    dq = J_inv * twist;
    q = q + dq * dt;

    q_record(:, i) = q;
    t = t + dt;

    robot.updateKinematics(q);
    anim.update(t);
end

anim.close();

%% Smooth and Save
q_record_smoothed = sgolayfilt(q_record, 3, 11, [], 2);
csvwrite('q_linear_z.csv', q_record_smoothed);

% Plot joint trajectories
figure;
plot(t_record, q_record_smoothed);
xlabel('Time [s]');
ylabel('Joint Angles [rad]');
title('Smoothed Joint Trajectories');

%% Plot Workspace Trajectory
figure;
plot3(p_traj(1,:), p_traj(2,:), p_traj(3,:), 'b-', 'LineWidth', 2); hold on;
plot3(p_des_traj(1,:), p_des_traj(2,:), p_des_traj(3,:), 'r--', 'LineWidth', 1.5);
grid on; axis equal;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend('Actual EE Trajectory', 'Desired EE Trajectory');
title('End-Effector Workspace Trajectory');