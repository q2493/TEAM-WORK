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

F(:,1) = force_vector(grav, sforce, body, q0);

%% Constrains

q_start(:,1) = q0;
qp_start(:,:,1) = zeros(length(q0));
t_start(1,1) = 0; t_end = 1; dt = 0.1; N_t = floor(round(t_end/dt)); 
for k = 1:N_t
    t_start(k+1,1) = t_start(k,1) + dt;

C_dyn =  constraint_dyn(revolute, simple, t_start(k,1), q_start(:,k));
Cq_dyn =  constraint_dq_dyn(revolute, simple, t_start(k,1), q_start(:,k));
Ct_dyn =  constraint_dt_dyn(revolute, simple, t_start(k,1), q_start(:,k));
Ctt_dyn =  constraint_dtt_dyn(revolute, simple, t_start(k,1), q_start(:,k), qp_start(:,:,k));

alpha = 10;
beta  = 10;
g = Ctt_dyn - 2*alpha*Ct_dyn - beta^2*C_dyn;

F_dyn =  [F ; g];

lll = size(Cq_dyn,1);
matrix =  [M, Cq_dyn';
          Cq_dyn, zeros(lll,lll) ];

u0_dyn = [zeros(size(q0)); zeros(lll,1); q0; zeros(lll,1)];
acc_dyn = matrix\ F_dyn;
ff_dyn = @(t,x) [ acc_dyn; x(1:23)];

         
opts = odeset('AbsTol', 1e-9, 'RelTol', 1e-6);
[t, u_v_dyn] = ode45(ff_dyn, t_start(k,1):dt:t_start(k+1,1), u0_dyn, opts);

%%%%%%%%%%%%%% Ploting

v2_dyn = u_v_dyn(:,1:12);
u2_dyn = u_v_dyn(:,24:35);

qqqq = v2_dyn(end-length(q0)+1:end,:,:)

q_start(:,k+1) = u2_dyn(end,:);
qp_start(:,:,k+1) = v2_dyn(end-length(q0)+1:end,:,:);

figure(4)
subplot(3,1,1)
plot(u2_dyn(:, 4), u2_dyn(:, 5),'*', ...
    u2_dyn(:, 7), u2_dyn(:, 8),'*',  ...
    u2_dyn(:, 10), u2_dyn(:, 11), '*', ...
    0, 0, '*', 'LineWidth', 2);
axis equal
xlabel('x [m]');
ylabel('y [m]');
title('Position');
hold on

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
hold on
end
