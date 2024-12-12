%% AME-556

% Project Task 2 Setup (Condensed MPC)

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
 
% Coeffcient of friction (static & dynamic)
Cof_s = 0.7;
Cof_d = 0.5;

% Initial states
params.q0 = [0; 0.43; 0; -pi/3; pi/2; -pi/6; pi/2];

% MPC parameters
dt = 0.04; % Time step
N = 10; % Number of horizons

% Stairs (grid surface)
l = 2;
dy = 1e-8;
x_grid = [0 1];
y_grid = [0 l l+dy l+0.2 l+0.2+dy l+0.4 l+0.4+dy l+0.6 l+0.6+dy l+0.8 l+0.8+dy l+0.8+l];
z_height = [0 0 0.1 0.1 0.2 0.2 0.3 0.3 0.4 0.4 0.5 0.5;
            0 0 0.1 0.1 0.2 0.2 0.3 0.3 0.4 0.4 0.5 0.5];