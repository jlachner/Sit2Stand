% compareWrenches.m
% Compare the external wrench measured by the FT-sensor (File_F_ft) with the
% one estimated from the robot joint-torque sensors (File_F_tau).
%
% Both wrenches are 6x1 [Fx Fy Fz Mx My Mz]^T in the robot base frame and are
% streamed by Eigen as one component per line -> 6 lines per timestep.


clear; close all; clc;


% ---- Path to the folder holding the .txt logs (EDIT if needed) -----------
dataDir = fileparts(mfilename('fullpath'));   % defaults to this script's folder


% ---- Load and reshape to [N x 6] ----------------------------------------
F_ft    = reshapeWrench(fullfile(dataDir, 'File_F_ft.txt'));    % FT-sensor wrench
F_tau   = reshapeWrench(fullfile(dataDir, 'File_F_tau.txt'));   % joint-torque estimate
dt      = load(fullfile(dataDir, 'File_dt.txt'));                  % time stamps [N x 1]


% ---- Convert FT-sensor wrench from imperial to metric units ---------------
% ATI FT-sensor logs forces in lbf and torques in lbf*in; F_tau is already
% in N/Nm (derived from the robot's SI joint-torque estimate).
LBF2N   = 4.4482216153;      % 1 lbf    -> N
LBFIN2NM = 0.1129848333;     % 1 lbf*in -> Nm
F_ft(:,1:3) = F_ft(:,1:3) * LBF2N;
F_ft(:,4:6) = F_ft(:,4:6) * LBFIN2NM;


% Align lengths defensively (in case logging was cut mid-cycle)
N     = min([size(F_ft,1), size(F_tau,1), numel(dt)]);
F_ft  = F_ft(1:N,:);
F_tau = F_tau(1:N,:);
t     = dt(1:N);


% ---- Low-pass filter the (noisy) FT-sensor wrench --------------------------
% Zero-phase Gaussian-weighted moving average (no Signal Processing Toolbox
% required, unlike butter/filtfilt). smoothWindow is the window length in
% seconds: larger -> smoother but more averaging across fast transients.
% Tune to trade off noise rejection vs. responsiveness.
smoothWindow = 0.05;   % [s]
F_ft_filt = F_ft;
for i = 1:6
    F_ft_filt(:,i) = smoothdata(F_ft(:,i), 'gaussian', smoothWindow, 'SamplePoints', t);
end


% ---- Plot ----------------------------------------------------------------
labels = {'F_x [N]','F_y [N]','F_z [N]','M_x [Nm]','M_y [Nm]','M_z [Nm]'};


figure('Name','FT-sensor vs. joint-torque wrench','Color','w');
for i = 1:6
    subplot(3,2,i); hold on; grid on;
    plot(t, F_ft(:,i),      'Color', [0.6 0.75 1], 'LineWidth', 0.8);
    plot(t, F_ft_filt(:,i), 'Color', [0   0.45 0.9], 'LineWidth', 1.5);
    plot(t, F_tau(:,i),     '--', 'Color', [0.85 0.33 0.1], 'LineWidth', 1.3);
    ylabel(labels{i});
    if i >= 5, xlabel('time [s]'); end
    if i == 1
        legend('FT-sensor (raw)','FT-sensor (filtered)','joint-torque est. (F\_tau)','Location','best');
    end
end
sgtitle('External wrench: FT-sensor vs. joint-torque estimate');


% ---- Helper --------------------------------------------------------------
function W = reshapeWrench(fname)
    v = load(fname);            % flat column: 6 values per timestep
    v = v(:);
    assert(mod(numel(v),6) == 0, ...
        '%s: number of values (%d) is not a multiple of 6.', fname, numel(v));
    W = reshape(v, 6, []).';    % -> [N x 6]
end
