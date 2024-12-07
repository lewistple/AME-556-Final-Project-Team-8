%% AME-556

% Project Task 2 Setup

%% Clear

clear, clc, close all;

%% System Setup (Include Simscape)

global params;

% Mass and geometry
params.m = 0.25;
params.l = 0.22;
params.mb = 8;
params.a = 0.25;
params.b = 0.15;
params.g = [0; 9.81; 0];

% Moment of inerita
Il_zz = params.m * params.l^2 / 12;
Ib_xx = params.mb * params.a^2 / 12;
Ib_yy = params.mb * params.b^2 / 12;
Ib_zz = params.mb * (params.a^2 + params.b^2) / 12;

% Spatial contact force parameters
Kp_g = 1e5;
Kd_g = 1e3;
 
% Initial states
params.q0 = [0; 0.45; 0; -pi/3; pi/2; -pi/6; pi/2];

%% Plot

% plot(out.F.Time, squeeze(out.F.Data(1, :, :)), LineWidth=2); hold on; grid on;
% plot(out.F.Time, squeeze(out.F.Data(2, :, :)), LineWidth=2); 
% plot(out.F.Time, squeeze(out.F.Data(4, :, :)), LineWidth=2); 
% plot(out.F.Time, squeeze(out.F.Data(5, :, :)), LineWidth=2); 
% legend('F1x', 'F1y', 'F2x', 'F2y');