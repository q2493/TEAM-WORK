clc, clear, close all
example_slider_crank
%close all
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

grav = [0; -10]; % gravitational acceleration

%% Get mass matrix

M = mass_matrix(body);
q0 = system_coordinates(body);

%% Add single force to the system
sforce.f = [1; 0];
sforce.i = 4;
sforce.u_i = [0; 0];

F = force_vector(grav, sforce, body, q0);

%% Time to integrate it
% Note that M is constant, but F, in general, no
% We can use one of the following:
%   ode45 from Matlab
%   Euler-Cromer as introduced some time ago
%   Lets try Euler-Cromer
acc_f = @(~, ~, ~) M\F;

dt = 1e-1; t_end = 1;
[t, u1, v1] = EulerCromer(acc_f, t_end, q0, zeros(size(q0)), dt);
ff = @(t,x) [M\F; x(1:size(M,1))];

u0 = [zeros(size(q0)), q0];
opts = odeset('AbsTol', 1e-9, 'RelTol', 1e-6);

[t, u_v] = ode45(ff, 0:dt:t_end, u0, opts);
a1 = [M\F]' .* ones(length(q0), length(0:dt:t_end))';

v2 = u_v(:, 1:length(q0));
u2 = u_v(:, length(q0)+1:2*length(q0));

ddu = u1 - u2;
ddv = v1 - v2;
   

%% Now some verification plots
%plot(t, u1(:, 2), t, u1(1, 2)-9.81 / 2 .* t .^ 2)

figure(3)
subplot(3,1,1)
plot(u2(:, 4), u2(:, 5), ...
    u2(:, 7), u2(:, 8), ...
    u2(:, 10), u2(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
xlabel('x [m]');
ylabel('y [m]');
title('Position');

subplot(3,1,2)
plot(v2(:, 4), v2(:, 5), ...
    v2(:, 7), v2(:, 8), ...
    v2(:, 10), v2(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
legend('Crank', 'Rod', 'Slider', 'Origin', 'Location','northeast');
xlabel('x_d [m/s]');
ylabel('y_d [m/s]');
title('Velocity');

subplot(3,1,3)
plot(a1(:, 4), a1(:, 5), '*', ...
    a1(:, 7), a1(:, 8), '*', ...
    a1(:, 10), a1(:, 11),'*', ...
    0, 0, '*', 'LineWidth', 2);
%axis equal
xlabel('x_d_d [m/s^2]');
ylabel('y_d_d [m/s^2]');
title('Acceleration');

%% Constrains


C_dyn = constraint_dyn(revolute, simple, 0, q_0);
Cq_dyn = constraint_dq_dyn(revolute, simple, 0, q_0);
Ct_dyn = constraint_dt_dyn(revolute, simple, 0, q_0);

lll = size(Cq_dyn,1)
matrix = [M, Cq_dyn';
          Cq_dyn, zeros(lll,lll) ];

alpha = 20;
beta  = 20;
c_idx = 0;
n_constr = 2 * length(revolute) +  length(simple);
g = zeros(n_constr, 1);
for r = revolute
   g(c_idx + (1:2)) = revolute_joint_dtt(r.i, r.j, r.s_i, r.s_j, q0, zeros(length(q0)));
   c_idx = c_idx + 2;
end
g = g - 2*alpha*Ct_dyn - beta^2*C_dyn;


%%

acc_dyn = matrix\[F; g]
acc_f = @(~, ~, ~) acc_dyn(1:length(q0));

[t, u1_dyn, v1_dyn] = EulerCromer(acc_f, t_end, q0, zeros(size(q0)), dt);


u0_dyn = [zeros(size(q0)); q0];
ff_dyn = @(t,x) [acc_dyn(1:length(q0)); x(1:length(q0))];

opts = odeset('AbsTol', 1e-9, 'RelTol', 1e-6);
[t, u_v_dyn] = ode45(ff_dyn, 0:dt:t_end, u0_dyn, opts);

v2_dyn = u_v_dyn(:, 1:length(q0));
u2_dyn = u_v_dyn(:, length(q0)+1:2*length(q0));

ddu_dyn = u1_dyn - u2_dyn;
ddv_dyn = v1_dyn - v2_dyn;

a1_dyn = acc_dyn(1:length(q0))' .* ones(length(q0), length(0:dt:t_end))';

figure(4)
subplot(3,1,1)
plot(u2_dyn(:, 4), u2_dyn(:, 5), ...
    u2_dyn(:, 7), u2_dyn(:, 8), ...
    u2_dyn(:, 10), u2_dyn(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
xlabel('x [m]');
ylabel('y [m]');
title('Position');

subplot(3,1,2)
plot(v2_dyn(:, 4), v2_dyn(:, 5), ...
    v2_dyn(:, 7), v2_dyn(:, 8), ...
    v2_dyn(:, 10), v2_dyn(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
legend('Crank', 'Rod', 'Slider', 'Origin', 'Location','northeast');
xlabel('x_d [m/s]');
ylabel('y_d [m/s]');
title('Velocity');

subplot(3,1,3)
plot(a1_dyn(:, 4), a1_dyn(:, 5), '*', ...
    a1_dyn(:, 7), a1_dyn(:, 8), '*', ...
    a1_dyn(:, 10), a1_dyn(:, 11),'*', ...
    0, 0, '*', 'LineWidth', 2);
axis equal
xlabel('x_d_d [m/s^2]');
ylabel('y_d_d [m/s^2]');
title('Acceleration');
