clear all;
close all;
clc;

%% Sim Parameters
num_chord = 8;
ref_chord = 0.2743;
V_inf = 10;
ds = 1/num_chord;
dt = ds*ref_chord/V_inf;
num_states = 2528;

%% Sim control
simulation_time = 5;
timesteps = floor(simulation_time/dt)

%% State-space matrices
A_sys = load("A_fsys.mat").arr;
B_sys = load("B_fsys.mat").arr;
C_sys = load("C_fsys.mat").arr;
D_sys = load("D_fsys.mat").arr;

fsys = ss(A_sys, B_sys, C_sys, D_sys, dt);

%% Disturbance matrices
discrete_gust = load("discretegusts/H5gust.mat").arr;
U = [0; 0; 0];

%% Full system time marching simulation
% Initialize states
x_old = zeros(num_states,1);
fplunge = zeros(timesteps,1);
fpitch = zeros(timesteps,1);
fbend = zeros(timesteps,1);
T = zeros(timesteps,1);

for t = 1:timesteps
    x_new = fsys.A*x_old + fsys.B*U + discrete_gust(:,t);
    y = fsys.C * x_new;
    fplunge(t,1) = y(1,1);
    fpitch(t,1) = y(2,1);
    fbend(t,1) = y(3,1);
    T(t,1) = t*dt;
    x_old = x_new;
end

%% Balance Reduction
balred(fsys);
% [rsys, info] = balred(fsys,20);

[sys,g,T,Ti] = balreal(fsys);
elim = (g<1e-3);
rsys = modred(sys,elim, "Truncate");
truncated_state = size(rsys.A,1)

rom_gust = T(1:truncated_state, 1:end) * discrete_gust;

A_rsys = rsys.A;
B_rsys = rsys.B;
C_rsys = rsys.C;
D_rsys = rsys.D;

T_rom = T(1:truncated_state, 1:end);
Ti_rom = Ti(1:end, 1:truncated_state);

%% ROM Time marching simulation
% Initialize states
x_old = zeros(truncated_state,1);
rplunge = zeros(timesteps,1);
rpitch = zeros(timesteps,1);
rbend = zeros(timesteps,1);
T = zeros(timesteps,1);

for t = 1:timesteps
    x_new = rsys.A*x_old + rsys.B*U + rom_gust(:,t);
    y = rsys.C * x_new;
    rplunge(t,1) = y(1,1);
    rpitch(t,1) = y(2,1);
    rbend(t,1) = y(3,1);
    T(t,1) = t*dt;
    x_old = x_new;
end

%% Plot and compare
figure(2);
hold on;
plot(T,fplunge, "k", "DisplayName", "2528 states", "LineWidth", 1);
plot(T,rplunge, "r--", "DisplayName","18 states", "LineWidth", 1);
grid on;
legend();
xlabel("time (s)")
ylabel("Plunge (m)")
hold off;

figure(3);
hold on;
plot(T,fpitch, "k", "DisplayName", "2528 states", "LineWidth", 1);
plot(T,rpitch, "r--", "DisplayName","18 states", "LineWidth", 1);
grid on;
legend();
xlabel("time (s)")
ylabel("Pitch (rad)")
hold off;

figure(4);
hold on;
plot(T,fbend*200, "k", "DisplayName", "2528 states", "LineWidth", 1);
plot(T,rbend*200, "r--", "DisplayName","18 states", "LineWidth", 1);
grid on;
legend();
xlabel("time (s)")
ylabel("Relative tip displacement (%)")
hold off;

%% ROM Bode plot
%
%bode(fsys);