clear all;
close all;
clc;

%% Sim Parameters
num_chord = 8;
num_panels = 8*(4+4+18);
ref_chord = 0.2743;
V_inf = 10;
ds = 1/num_chord;
dt = ds*ref_chord/V_inf;

%% Sim control
simulation_time = 5;
timesteps = floor(simulation_time/dt)

%% State-space matrices
UVLM_A = load("UVLM_A.mat").arr;
UVLM_B = load("UVLM_B.mat").arr;
UVLM_input = load("UVLM_input.mat").arr;

num_states = size(UVLM_A,1);

A_sys = inv(UVLM_A) * UVLM_B;
B_sys = inv(UVLM_A) * UVLM_input(1:num_states,:);
C_sys = zeros(num_states,num_states);

for i = 1:num_panels
    C_sys(i,i) = 1;
end

D_sys = zeros(num_states, size(B_sys,2));


fsys = ss(A_sys, B_sys, C_sys, D_sys, dt);

%% Disturbance matrices
discrete_gust = load("H5gust.mat").arr;
discrete_gust = discrete_gust(1:num_states,:);
U = [0; 0; 0];

%% Full system time marching simulation
% Initialize states
x_old = zeros(num_states,1);
g1 = zeros(timesteps,1);
g2 = zeros(timesteps,1);
g3 = zeros(timesteps,1);
T = zeros(timesteps,1);

for t = 1:timesteps
    x_new = fsys.A*x_old + fsys.B*U + discrete_gust(:,t);
    y = fsys.C * x_new;
    g1(t,1) = y(4,1);
    g2(t,1) = y(10,1);
    g3(t,1) = y(30,1);
    T(t,1) = t*dt;
    x_old = x_new;
end

%% Balance Reduction
balred(fsys);
% [rsys, info] = balred(fsys,20);

%% Reduce

[sys,g,T,Ti] = balreal(fsys);
elim = (g<1e-5);
rsys = modred(sys,elim, "Truncate");
truncated_state = size(rsys.A,1)

rom_gust = T(1:truncated_state, 1:end) * discrete_gust;

A_rsys = rsys.A;
B_rsys = rsys.B;
C_rsys = rsys.C;
D_rsys = rsys.D;

T_rom = T(1:truncated_state, 1:end);
Ti_rom = Ti(1:end, 1:truncated_state);

%%
TiT = T_rom * Ti_rom;

%% ROM Time marching simulation
% Initialize states
x_old = zeros(truncated_state,1);
rg1 = zeros(timesteps,1);
rg2 = zeros(timesteps,1);
rg3 = zeros(timesteps,1);
T = zeros(timesteps,1);

for t = 1:timesteps
    x_new = rsys.A*x_old + rsys.B*U + rom_gust(:,t);
    y = rsys.C * x_new;
    rg1(t,1) = y(4,1);
    rg2(t,1) = y(10,1);
    rg3(t,1) = y(30,1);
    T(t,1) = t*dt;
    x_old = x_new;
end

%% Plot and compare
figure(2);
hold on;
plot(T,g1, "k", "DisplayName", "2528 states", "LineWidth", 1);
plot(T,rg1, "r--", "DisplayName","18 states", "LineWidth", 1);
grid on;
legend();
xlabel("time (s)")
ylabel("Plunge (m)")
hold off;

figure(3);
hold on;
plot(T,g2, "k", "DisplayName", "2528 states", "LineWidth", 1);
plot(T,rg2, "r--", "DisplayName","18 states", "LineWidth", 1);
grid on;
legend();
xlabel("time (s)")
ylabel("Pitch (rad)")
hold off;

figure(4);
hold on;
plot(T,g3, "k", "DisplayName", "2528 states", "LineWidth", 1);
plot(T,rg3, "r--", "DisplayName","18 states", "LineWidth", 1);
grid on;
legend();
xlabel("time (s)")
ylabel("Relative tip displacement (%)")
hold off;

%% ROM Bode plot
%
%bode(fsys);