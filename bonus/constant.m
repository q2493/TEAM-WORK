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
sforce.f = [1; 0];
sforce.i = 4;
sforce.u_i = [0; 0];

F(:,1) = force_vector(grav, sforce, body, q0);

%% Constrains

C_dyn(:,1) =  constraint_dyn(revolute, simple, 0, q0);
Cq_dyn(:,:,1) =  constraint_dq_dyn(revolute, simple, 0, q0);
Ct_dyn(:,1) =  constraint_dt_dyn(revolute, simple, 0, q0);
Ctt_dyn(:,1) =  constraint_dtt_dyn(revolute, simple, 0, q0, zeros(length(q0)));

alpha = 10;
beta  = 10;
g(:,1) = Ctt_dyn(:,1) - 2*alpha*Ct_dyn(:,1) - beta^2*C_dyn(:,1);

F_dyn(:,1) =  [F(:,1) ; g(:,1)];

lll = size(Cq_dyn(:,1),1);
matrix(:,:,1) =  [M, Cq_dyn(:,:,1)';
          Cq_dyn(:,:,1), zeros(lll,lll) ];

u0_dyn = [zeros(size(q0)); zeros(lll,1); q0; zeros(lll,1)];
acc_dyn = matrix(:,:,1)\ F_dyn(:,1);
ff_dyn = @(t,x) [ acc_dyn; x(1:23)];

 dt = 0.01; t_end = 1;
                
opts = odeset('AbsTol', 1e-9, 'RelTol', 1e-6);
[t, u_v_dyn] = ode45(ff_dyn, 0:dt:t_end, u0_dyn, opts);


%%

v2_dyn = u_v_dyn(:,1:12);
u2_dyn = u_v_dyn(:, 24:35);

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


