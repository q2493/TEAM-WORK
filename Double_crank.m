% Slider crank kinematic analysis
clc, clear, close all
%% Coordinates

a = 0.150; b = 0.300; c = 0.120;

% ground body 1
q1 = [0; 0; 0];
% body 2
q2 = [0
    a/2
    pi/2];
% body 3
phi_1 = asin(0.5*c/ b); % link's angle
q3 = [-0.5*b * sin(phi_1)
       a + 0.5*b*cos(phi_1)
      pi/2 + phi_1];
% body 4
q4 = [0.5*b * sin(phi_1)
       a + 0.5*b*cos(phi_1)
      pi/2 - phi_1];
% body 5
q5 = [-0.5*c
    a + b*cos(phi_1)
    0];
% body 6
q6 = [0.5*c
    a + b*cos(phi_1)
    0];

q_0 = [q1; q2; q3; q4; q5; q6]; % initial coordinates

%% We need two constraint types (geometric ones)
% - revolute
% - simple constraints

%% Revolute joints
% 1 connects ground and crank
revolute(1).i = 1;
revolute(1).j = 2;
revolute(1).s_i = [0; 0];
revolute(1).s_j = [-0.5*a; 0];

% 2 connects crank and link
revolute(2).i = 2;
revolute(2).j = 3;
revolute(2).s_i = [0.5*a; 0];
revolute(2).s_j = [-0.5*b; 0];

% 2 connects crank and link
revolute(3).i = 2;
revolute(3).j = 4;
revolute(3).s_i = [0.5*a; 0];
revolute(3).s_j = [-0.5*b; 0];

% 3 connects link and slider
revolute(4).i = 3;
revolute(4).j = 5;
revolute(4).s_i = [0.5*b; 0];
revolute(4).s_j = [0; 0];

% 3 connects link and slider
revolute(5).i = 4;
revolute(5).j = 6;
revolute(5).s_i = [0.5*b; 0];
revolute(5).s_j = [0; 0];

% % Check revolute joint constraints
% r = revolute(5);
% C_r_i = revolute_joint(r.i, r.j, r.s_i, r.s_j, q_0)

%% Simple constraints

% Three simple joints to fix the ground origin
simple(1).i = 1;
simple(1).k = 1;
simple(1).c_k = 0;

simple(2).i = 1;
simple(2).k = 2;
simple(2).c_k = 0;

simple(3).i = 1;
simple(3).k = 3;
simple(3).c_k = 0;

% slider - use simple joints instead of translational
simple(4).i = 5;
simple(4).k = 1;
simple(4).c_k = -0.5*c;

simple(5).i = 5;
simple(5).k = 3;
simple(5).c_k = 0;

% slider - use simple joints instead of translational
simple(6).i = 6;
simple(6).k = 1;
simple(6).c_k = 0.5*c;

simple(7).i = 6;
simple(7).k = 3;
simple(7).c_k = 0;

% % check simple constraints
 for s = simple
   C_s_i = simple_joint(s.i, s.k, s.c_k, q_0)
 end

%% Add some driving constraints
driving.i = 2;
driving.k = 3;
driving.d_k = @(t) pi/2 + 6 * t;
driving.d_k_t = @(t) +6;
driving.d_k_tt = @(t) 0;

% % Verify
% d = driving(1);
% C_d_i = driving_joint(d.i, d.k, d.d_k, 0, q_0)

% %% Verify constraint function
% clc
% C = constraint(revolute, simple, driving, 0, q_0)

%% Jacobian of our constraints
Cq = constraint_dq(revolute, simple, driving, 0, q_0);
Cq_dyn = constraint_dq_dyn(revolute, simple, 0, q_0);
C_dyn = constraint_dyn(revolute, simple, 0, q_0)

%% Solve constraint equation using NR
%C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
%Cq_fun = @(t, q) constraint_dq(revolute, simple, driving, t, q);
%[T, Q] = position_NR(C_fun, Cq_fun, 1, q_0, 0.1);

%% Some verification plots
%plot(Q(:, 4), Q(:, 5), ...
 %   Q(:, 7), Q(:, 8), ...
 %   Q(:, 10), Q(:, 11), ...
 %   0, 0, '*', 'LineWidth', 2);
% axis equal

%% Verify Ct
Ct = constraint_dt(revolute, simple, driving, 0, q_0);
Ctt = constraint_dtt(revolute, simple, driving, 0, q_0, zeros(length(q_0)));
C = constraint(revolute, simple, driving, 0, q_0);
Ct_dyn = constraint_dt_dyn(revolute, simple, 0, q_0);

%% Solve constraint equation using NR for position and velocity
C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
Cq_fun = @(t, q) constraint_dq(revolute, simple, driving, t, q);
Ct_fun = @(t, q) constraint_dt(revolute, simple, driving, t, q);
Ctt_fun = @(t, q, qp) constraint_dtt(revolute, simple, driving, t, q, qp);
[T, Q, QP, QPP] = pos_vel_acc_NR(C_fun, Cq_fun, Ct_fun, Ctt_fun, 1, q_0, 0.001);

%% Some verification plots
figure(1)
subplot(3,1,1)
plot(Q(:, 4), Q(:, 5), ...
    Q(:, 7), Q(:, 8), ...
    Q(:, 10), Q(:, 11), ...
    Q(:, 13), Q(:, 14), ...
    Q(:, 16), Q(:, 17), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
xlabel('x [m]');
ylabel('y [m]');
title('Position');

subplot(3,1,2)
plot(QP(:, 4), QP(:, 5), ...
    QP(:, 7), QP(:, 8), ...
    QP(:, 10), QP(:, 11), ...
    QP(:, 13), QP(:, 14), ...
    QP(:, 16), QP(:, 17), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
legend('Crank', 'Rod 1', 'Rod 2', 'Slider 1', 'Slider 2', 'Origin', 'Location','northeast');
xlabel('x_d [m/s]');
ylabel('y_d [m/s]');
title('Velocity');

subplot(3,1,3)
plot(QPP(:, 4), QPP(:, 5), ...
    QPP(:, 7), QPP(:, 8), ...
    QPP(:, 10), QPP(:, 11), ...
    QPP(:, 13), QPP(:, 14), ...
    QPP(:, 16), QPP(:, 17), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
xlabel('x_d_d [m/s^2]');
ylabel('y_d_d [m/s^2]');
title('Acceleration');


%% Verification by primitive derivation

V = zeros(length(T)-1,length(q_0));
for ii = 1:length(T)-1
   V(ii,:) = (Q(ii+1,:)-Q(ii,:))/T(2);
end

figure(2)
subplot(2,1,1)
plot(V(:, 4), V(:, 5), ...
    V(:, 7), V(:, 8), ...
    V(:, 10), V(:, 11), ...
    V(:, 13), V(:, 14), ...
    V(:, 16), V(:, 17), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
title('Velocity');

A = zeros(length(T)-1,length(q_0));
for k = 1:length(T)-1
   A(k,:) = (QP(k+1,:)-QP(k,:))/T(2);
end

subplot(2,1,2)
plot(A(:, 4), A(:, 5), ...
    A(:, 7), A(:, 8), ...
    A(:, 10), A(:, 11), ...
    A(:, 13), A(:, 14), ...
    A(:, 16), A(:, 17), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
title('Acceleration');
