%% AME-556

% HW4 Problem 2 (a. QP controller)

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
params.Ib = diag([Ib_xx Ib_yy Ib_zz]);

% Spatial contact force parameters
Kp_g = 1e5;
Kd_g = 1e3;

% Ground contact positions
params.P1 = [0.0197; 0.0022; 0];
params.P2 = [0.1838; 0.0041; 0]; 

% State variables
params.q0 = [0.1; 0.45; 0; -pi/3; pi/2; -pi/6; pi/2]; % Initial states (before landing)
params.p_d = [0; 0.5; 0]; % Desired positions (after standing): [x_d; y_d; z_d];
params.theta_d = 0; % Desired angle

%% Program Execution

% Initial condition & timespan
% X = [x; y; z; R(9x1); dx; dy; dz; wbx; wby; wbz];
R0 = reshape(eul2rotm([-0.0056 0 0]), [9 1]);
X0 = [0.0999; 0.4192; 0; R0; 0; 0; 0; 0; 0; 0];
tspan = [0 2];

% Solve system dynamics through ODE
[T, X] = ode45(@(t, Q) sys_dynamics(Q), tspan, X0);

% Solve forward kinematics symbolically
syms q1 q2 q3 q4 x y theta
q_sym = [x; y; theta; q1; q2; q3; q4];
[P_E, J_sym, Jt_sym, K1_sym, K2_sym] = forward_k(q_sym);
eqn1 = P_E(1: 3) == params.P1;
eqn2 = P_E(4: 6) == params.P2;
S = solve([eqn1 eqn2], [q1 q2 q3 q4]);

Theta = zeros(length(T), 1);
for i = 1: length(T)

    % Obtain theta
    Rot = reshape(X(i, 4: 12), [3 3]);
    Eul = rotm2eul(Rot);
    Theta(i) = Eul(1);

    % Obtain control inputs
    F(:, i) = qp_controller(X(i, :)');

    % Find system states
    q(i, 1: 2) = X(i, 1: 2);
    q(i, 3) = Theta(i);
    qs = subs([S.q1 S.q2 S.q3 S.q4], [x y theta], [X(i, 1) X(i, 2) Theta(i)]);
    for j = 1: size(qs, 1)

        if qs(j, 1) <= 0 && qs(j, 3) <= 0 

            q(i, 4: 7) = qs(j, :);

        end

    end

    % Solve for joint/knee positions
    Jt(i, :) = double(subs(Jt_sym, [x y theta q1 q2 q3 q4], q(i, :)))';
    K1(i, :) = double(subs(K1_sym, [x y theta q1 q2 q3 q4], q(i, :)))';
    K2(i, :) = double(subs(K2_sym, [x y theta q1 q2 q3 q4], q(i, :)))';

    % Substitute states into Jacobian
    J = double(subs(J_sym, [x y theta q1 q2 q3 q4], q(i, :)));

    % Compute joint input torques
    tau(i, :) = -(J' * F(:, i))';

end

% % Animate
% anim(T, [q tau Jt K1 K2], 1/96, 'video_HW4_P2_QP.avi');

%% Plots

% Plot 1: body positions (x, y, theta) 
figure; hold on;
p1 = tiledlayout(3, 1); title(p1, 'Robot Body Positions, QP Control');
nexttile; plot(T, X(:, 1), LineWidth=2); grid on; xlabel('$t[s]$', Interpreter='latex'); ylabel('$x[m]$', Interpreter='latex');
nexttile; plot(T, X(:, 2), LineWidth=2); grid on; xlabel('$t[s]$', Interpreter='latex'); ylabel('$y[m]$', Interpreter='latex');
nexttile; plot(T, Theta, LineWidth=2); grid on; xlabel('$t[s]$', Interpreter='latex'); ylabel('$\theta[rad]$', Interpreter='latex');

% Plot 2: joint input torques (tau1 ~ tau4)
figure; hold on;
p2 = tiledlayout(2, 2); title(p2, 'Robot Joint Input Torques, QP Control');
nexttile; plot(T, tau(:, 1), LineWidth=2); grid on; xlabel('$t[s]$', Interpreter='latex'); ylabel('$\tau_1[Nm]$', Interpreter='latex');
nexttile; plot(T, tau(:, 3), LineWidth=2); grid on; xlabel('$t[s]$', Interpreter='latex'); ylabel('$\tau_3[Nm]$', Interpreter='latex');
nexttile; plot(T, tau(:, 2), LineWidth=2); grid on; xlabel('$t[s]$', Interpreter='latex'); ylabel('$\tau_2[Nm]$', Interpreter='latex');
nexttile; plot(T, tau(:, 4), LineWidth=2); grid on; xlabel('$t[s]$', Interpreter='latex'); ylabel('$\tau_4[Nm]$', Interpreter='latex');

%% Function: System Dynamics

function dq = sys_dynamics(q)

global params;

dq = zeros(18, 1);

R = reshape(q(4: 12), [3, 3]);

wb = q(16: 18);
swb = skew_tensor(wb);

% F = [F1x; F1y; F1z; F2x; F2y; F2z]
F = qp_controller(q);

r1 = params.P1 - q(1: 3);
r2 = params.P2 - q(1: 3);

M = cross(r1, F(1: 3)) + cross(r2, F(4: 6));
Mb = R.' * M;

dq(1: 3) = q(13: 15);
dq(4: 12) = reshape(R * swb, [9, 1]);
dq(13: 15) = (F(1: 3) + F(4: 6)) / params.mb - params.g;
dq(16: 18) = params.Ib \ (Mb - cross(wb, params.Ib * wb));

end

%% Function: QP Controller

function F_qp = qp_controller(q)

global params;

% Input propagation (A)
r1 = params.P1 - q(1: 3);
r2 = params.P2 - q(1: 3);
A = [         eye(3)          eye(3);
     skew_tensor(r1) skew_tensor(r2)];

% PD controller
Kp_p = diag([110 500 0]); Kd_p = diag([250 250 0]);
Kp_r = diag([0 0 100]); Kd_r = diag([0 0 50]);

% Desired dynamics (b_d), controlled by PD
ddp_d = -Kp_p * (q(1: 3) - params.p_d) - Kd_p * q(13: 15); % Desired accel: [ddx; ddy; ddz]

R = reshape(q(4: 12), [3 3]);
wb = q(16: 18);
R_d = eul2rotm([params.theta_d 0 0]);
R_e = R * R_d';
eul_e = rotm2eul(R_e);
ypr_e = [eul_e(3); eul_e(2); eul_e(1)];
dwb_d = -Kp_r * ypr_e - Kd_r * wb; % Desired angular accel: [roll; pitch; yaw]

M_b = params.Ib * dwb_d + cross(wb, params.Ib * wb);
M = R * M_b;

b_d = [params.mb * (ddp_d + params.g); M];

% Optimization parameters (H, f)
H = A'*A;
f = -A'*b_d;

% Inequality constraints (A_ineq, b_ineq)
A_ineq = [ 0    1  0  0    0  0;
           0   -1  0  0    0  0;
           0    0  0  0    1  0;
           0    0  0  0   -1  0;
           1 -0.7  0  0    0  0;
          -1 -0.7  0  0    0  0;
           0    0  0  1 -0.7  0;
           0    0  0 -1 -0.7  0];
b_ineq = [250; -10; 250; -10; 0; 0; 0; 0];

% Equality constraints (A_eq, b_eq)
A_eq = [0 0 1 0 0 0;
        0 0 0 0 0 1];
b_eq = [0; 0];

% Quadratic programming
F_qp = quadprog(H, f, A_ineq, b_ineq, A_eq, b_eq);

end

%% Function: Create a 3D Skew Tensor out of a Vector

function sv = skew_tensor(v)

sv = [    0 -v(3)  v(2);
       v(3)     0 -v(1);
      -v(2)  v(1)     0];

end

%% Function: Forward Kinematics

function [P_E, J, Jt_w, K1_w, K2_w] = forward_k(q)

global params;
x = q(1); y = q(2); theta = q(3);
q1 = q(4); q2 = q(5); q3 = q(6); q4 = q(7);

% P1
P1_2 = [-params.l; 0; 0]; % Frame {2}
P1_1 = rotz_sym(q2) * P1_2 + [0; -params.l; 0]; % Frame {1} to {2}
P1_b = rotz_sym(q1) * P1_1 + [0; -params.a/2; 0]; % Frame {b} to {1}
P1_w = rotz_sym(theta) * P1_b + [x; y; 0]; % Frame {w} to {b}

% P2 
P2_4 = [0; -params.l; 0]; % Frame {4}
P2_3 = rotz_sym(q4) * P2_4 + [0; -params.l; 0]; % Frame {3} to {4}
P2_b = rotz_sym(q3) * P2_3 + [0; -params.a/2; 0]; % Frame {b} to {3}
P2_w = rotz_sym(theta) * P2_b + [x; y; 0]; % Frame {w} to {b}

% Joint
Jt_b = [0; -params.a/2; 0]; % Frame {b}
Jt_w = rotz_sym(theta) * Jt_b + [x; y; 0]; % Frame {w} to {b}

% Knee1
K1_1 = [0; -params.l; 0]; % Frame {1}
K1_b = rotz_sym(q1) * K1_1 + [0; -params.a/2; 0]; % Frame {b} to {1}
K1_w = rotz_sym(theta) * K1_b + [x; y; 0]; % Frame {w} to {b}

% Knee2
K2_3 = [0; -params.l; 0]; % Frame {3}
K2_b = rotz_sym(q3) * K2_3 + [0; -params.a/2; 0]; % Frame {b} to {3}
K2_w = rotz_sym(theta) * K2_b + [x; y; 0]; % Frame {w} to {b}

% End effector coordinates
P_E = [P1_w; P2_w];

% Jacobian
J = simplify(jacobian(P_E, q(4: 7)));

end

%% Function: Symbolic Z-rotation

function M = rotz_sym(theta)

M = [cos(theta) -sin(theta) 0;
     sin(theta)  cos(theta) 0;
              0           0 1];

end

%% Animation Functions

function anim(t, x, ts, video_name)
[te,xe] = even_sample(t,x,1/ts);

% save as a video
spwriter = VideoWriter(video_name);
set(spwriter, 'FrameRate', 1/ts,'Quality',100);
open(spwriter);

fig1 = figure;

figure_x_limits = [-0.5 0.5];
figure_y_limits = [-0.2 0.8];

axes1 = axes;
set(axes1,'XLim',figure_x_limits,'YLim',figure_y_limits);
set(axes1,'Color','w');

xlabel('x [m]');
ylabel('y [m]');
title('System Animation');

for k = 1:length(te)
    drawone(axes1, xe(k,:)');    
    text(-0.4, 0.7, sprintf('Time = %.2f s', te(k)));
    text(-0.4, 0.6, sprintf('x = %.4f m', xe(k, 1)));
    text(-0.4, 0.55, sprintf('y = %.4f m', xe(k, 2)));
    text(-0.4, 0.5, sprintf('θ = %.4f rad', xe(k, 3)));
    text(-0.4, 0.4, sprintf('τ1 = %.2f Nm', xe(k, 8)));
    text(-0.4, 0.35, sprintf('τ2 = %.2f Nm', xe(k, 9)));
    text(-0.4, 0.3, sprintf('τ3 = %.2f Nm', xe(k, 10)));
    text(-0.4, 0.25, sprintf('τ4 = %.2f Nm', xe(k, 11)));
    set(axes1,'XLim',figure_x_limits,'YLim',figure_y_limits);
    drawnow;
    pause(ts);
    frame = getframe(gcf);
    writeVideo(spwriter, frame);
end

end

function [Et, Ex] = even_sample(t, x, Fs)
%       CONVERTS A RANDOMLY SAMPLED SIGNAL SET INTO AN EVENLY SAMPLED
%       SIGNAL SET (by interpolation)
% Obtain the process related parameters
N = size(x, 2);    % number of signals to be interpolated
M = size(t, 1);    % Number of samples provided
t0 = t(1,1);       % Initial time
tf = t(M,1);       % Final time
EM = (tf-t0)*Fs;   % Number of samples in the evenly sampled case with
                   % the specified sampling frequency
Et = linspace(t0, tf, EM)';

% Using linear interpolation (used to be cubic spline interpolation)
% and re-sample each signal to obtain the evenly sampled forms
for s = 1:N
  Ex(:,s) = interp1(t(:,1), x(:,s), Et(:,1)); 
end
end

function drawone(parent, x)
% draw the robot at the current frame
global params;
tem = get(parent,'Children');
delete(tem);
% x(1: 7) = [x; y; theta; q1; q2; q3; q4];
% x(8: 11) = tau;
% x(12: 14) = Jt (xyz);
% x(15: 17) = K1 (xyz);
% x(18: 20) = K2 (xyz);
p1 = rotz(x(3)) * [-params.b/2; -params.a/2; 0] + [x(1); x(2); 0];
p2 = rotz(x(3)) * [params.b/2; -params.a/2; 0] + [x(1); x(2); 0];
p3 = rotz(x(3)) * [params.b/2; params.a/2; 0] + [x(1); x(2); 0];
p4 = rotz(x(3)) * [-params.b/2; params.a/2; 0] + [x(1); x(2); 0];
patch([p1(1) p2(1) p3(1) p4(1)], [p1(2) p2(2) p3(2) p4(2)], 'k');
leg1 = line([x(12) x(15)],[x(13) x(16)]); leg1.Color = 'b'; leg1.LineWidth = 2;
leg2 = line([x(15) params.P1(1)],[x(16) params.P1(2)]); leg2.Color = 'r'; leg2.LineWidth = 2;
leg3 = line([x(12) x(18)],[x(13) x(19)]); leg3.Color = 'g'; leg3.LineWidth = 2;
leg4 = line([x(18) params.P2(1)],[x(19) params.P2(2)]); leg4.Color = 'y'; leg4.LineWidth = 2;
yline(0);
grid on;
end