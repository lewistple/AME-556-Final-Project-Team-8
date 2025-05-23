clc;
clear;
close all;

%% Parameters

global leg_m leg_l body_M body_a body_b q0 X0 static_friction dynamic_friction

leg_m = 0.25; % Each leg mass (kg)
leg_l = 0.22; % Leg length (m)

body_M = 8; % Body mass (kg)
body_a = 0.25;
body_b = 0.15;

static_friction = 0.7;
dynamic_friction = 0.5;

% I.C.
x0 = 0.1;
y0 = 0.44;
theta0 = 0;
q0 = [-pi/3; pi/2; -pi/6; pi/2]; 
X0 = [x0; y0; theta0];

[foot1, foot, CoM] = compute_biped_positions_IC(x0, y0, theta0, q0, leg_l, body_a)

ComputeJacobianSymbolic()

% %% load model
% problem2a_sim;
% 
% %% load simscape
% load_system("problem2a_sim.slx");
% out = sim("problem2a_sim.slx");
% save_system("problem2a_sim.slx");

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

    syms q1 q2 q3 q4 real  
    syms leg_l real   
    syms body_a real    

    x1 = leg_l * sin(q1) + leg_l * sin(q1 + q2);
    y1 = -leg_l * cos(q1) - leg_l * cos(q1 + q2) - body_a/2;

    x2 = leg_l * sin(q3) + leg_l * sin(q3 + q4);
    y2 = -leg_l * cos(q3) - leg_l * cos(q3 + q4) - body_a/2;

    J1 = jacobian([x1; y1], [q1; q2]);
    J2 = jacobian([x2; y2], [q3; q4]); 

    J = blkdiag(J1, J2);
end