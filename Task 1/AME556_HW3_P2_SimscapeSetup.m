%% AME-556

% HW3 Problem 2 (Simscape setup)

%% Clear

clear, clc, close all;

%% System Properties

global params2;
params2.m = 0.25;
params2.l = 0.22;
params2.mb = 8;
params2.a = 0.25;
params2.b = 0.15;
params2.g = 9.81;
params2.Kp_g = 1e5;
params2.Kd_g = 1e3;
params2.Ib_zz = params2.mb * (params2.a^2 + params2.b^2) / 12;
params2.Il_zz = params2.m * params2.l^2 / 12;
params2.Kp = 50;
params2.Kd = 2;

% Initial condition
% q = [x; y; theta; q1; q2; q3; q4]
q0 = [0; 0.65; 0; -pi/3; pi/2; 0; pi/2];