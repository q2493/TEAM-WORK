clc, clear, close all
example_slider_crank
close all
%% Define one body

body(1).m = 1; % mass equals to one kg
body(2).m = 1;
body(3).m = 2;
body(4).m = 3;

body(1).l = 1; 
body(2).l = 0.2;
body(3).l = 0.5;
body(4).l = 0.1;

body(1).Ic = body(1).m * body(1).l^2 / 12; % mass moment of inertia along center of mass in kgm2
body(2).Ic = body(2).m * body(2).l^2 / 12;
body(3).Ic = body(3).m * body(3).l^2 / 12;
body(4).Ic = body(4).m * body(4).l^2 / 12;

body(1).q = q1;
body(2).q = q2;
body(3).q = q3;
body(4).q = q4;

grav = [0; 0]; % gravitational acceleration

%% Get mass matrix

M = mass_matrix(body);
q0 = system_coordinates(body);

%% Add single force to the system
sforce.f = [10; 0];
sforce.i = 4;
sforce.u_i = [0; 0];

F = @(q) force_vector(grav, sforce, body, q);

%% Constrains

C_dyn = @(t,q) constraint_dyn(revolute, simple, t, q);
Cq_dyn = @(t,q) constraint_dq_dyn(revolute, simple, t, q);
Ct_dyn = @(t,q) constraint_dt_dyn(revolute, simple, t, q);
Ctt_dyn = @(t,q, qp) constraint_dtt_dyn(revolute, simple, t, q, qp);

alpha = 20;
beta  = 20;
g = @(t,q, qp) Ctt_dyn(t,q, qp) - 2*alpha*Ct_dyn(t,q) - beta^2*C_dyn(t,q);

F_dyn = @(t,q,qp) [F(q) ; g(t,q,qp)];

lll = size(Cq_dyn(0,q0),1);
matrix = @(t,q) [M, Cq_dyn(t,q)';
          Cq_dyn(t,q), zeros(lll,lll) ];

%u0_dyn = [zeros(size(q0)); zeros(lll,1); q0; zeros(lll,1)];
%ff_dyn = @(t,x) [ matrix(t,x(24:35))\ F_dyn(t,x(24:35),x(1:12)); x(24:46)];

dt = 0.001; 
t_end = 1;                
%opts = odeset('AbsTol', 1e-9, 'RelTol', 1e-6);
%[t, u_v_dyn] = ode45(ff_dyn, 0:dt:t_end, u0_dyn, opts);

acc_f = @(t, q, dq) matrix(t,q)\F_dyn(t,q,dq);
[t, u, v] = EulerCromer_mod_my(matrix,F_dyn, t_end, [q_0;zeros(11,1)], [zeros(size(q_0));zeros(11,1)], dt);

%%


figure(4)
subplot(2,1,1)
plot(u(:, 4), u(:, 5), ...
    u(:, 7), u(:, 8), ...
    u(:, 10), u(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
xlabel('x [m]');
ylabel('y [m]');
title('Position');

subplot(2,1,2)
plot(v(:, 4), v(:, 5), ...
    v(:, 7), v(:, 8), ...
    v(:, 10), v(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
legend('Crank', 'Rod', 'Slider', 'Origin', 'Location','northeast');
xlabel('x_d [m/s]');
ylabel('y_d [m/s]');
title('Velocity');


