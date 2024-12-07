clc;
clear;
close all;

%% Parameters

leg_m = 0.25; % Each leg mass (kg)
leg_l = 0.22; % Leg length (m)

body_M = 8; % Body mass (kg)
body_a = 0.25;
body_b = 0.15;
I_body = 1/12 * body_M * (body_a^2 + body_b^2);

static_friction = 0.7;
dynamic_friction = 0.5;

% I.C.
x0 = 0;
y0 = 0.45;
theta0 = 0;
q0 = [-pi/3; pi/2; -pi/6; pi/2]; 
X0 = [x0; y0; theta0];

[foot1, foot, CoM] = compute_biped_positions_IC(x0, y0, theta0, q0, leg_l, body_a);

% 调用函数生成雅可比矩阵
ComputeJacobianSymbolic();

%% controller

dt = 0.04;
N = 10;

Q = diag([600 600 400 150 150 20 1]); 
R = diag([0.0001 0.0001 0.0001 0.0001]);    

ref = [0; 0.55; 0; 0; 0; 0; 9.81];

%% load simscape
% load_system("problem2b_sim.slx");
% out = sim("problem2b_sim.slx");
% save_system("problem2b_sim.slx");

function [foot1, foot2, CoM] = compute_biped_positions_IC(x, y, theta, q, leg_l, body_a)

    % in body frame
    x1 = leg_l * sin(q(1)) + leg_l * sin(q(1) + q(2));
    y1 = -leg_l * cos(q(1)) - leg_l * cos(q(1) + q(2)) - body_a/2;
    x2 = leg_l * sin(q(3)) + leg_l * sin(q(3) + q(4));
    y2 = -leg_l * cos(q(3)) - leg_l * cos(q(3) + q(4)) - body_a/2;
    foot1_body = [x1; y1];
    foot2_body = [x2; y2];
    
    % in world frame
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    foot1 = R * foot1_body + [x; y];
    foot2 = R * foot2_body + [x; y];
    CoM = [x, y];
end

function J = ComputeJacobianSymbolic()
    % 定义符号变量
    syms q1 q2 q3 q4 real  % 四个关节角
    syms leg_l real        % 腿段长度
    syms body_a real       % 身体垂直尺寸

    % 足端1的位置 (身体坐标系)
    x1 = leg_l * sin(q1) + leg_l * sin(q1 + q2);
    y1 = -leg_l * cos(q1) - leg_l * cos(q1 + q2) - body_a/2;

    % 足端2的位置 (身体坐标系)
    x2 = leg_l * sin(q3) + leg_l * sin(q3 + q4);
    y2 = -leg_l * cos(q3) - leg_l * cos(q3 + q4) - body_a/2;

    % 计算雅可比矩阵
    J1 = jacobian([x1; y1], [q1; q2]); % 第一条腿
    J2 = jacobian([x2; y2], [q3; q4]); % 第二条腿

    % 合并雅可比矩阵
    J = blkdiag(J1, J2);
end