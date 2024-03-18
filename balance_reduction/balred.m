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
simulation_time = 10;
timesteps = floor(simulation_time/dt)

%% State-space matrices
A_sys = load("../model/full/A_fsys.mat").arr;
B_sys = load("../model/full/B_fsys.mat").arr;
C_sys = load("../model/full/C_fsys.mat").arr;
D_sys = load("../model/full/D_fsys.mat").arr;

fsys = ss(A_sys, B_sys, C_sys, D_sys, dt);

%% Disturbance matrices
H5_gust = load("../model/discretegusts/H5gust.mat").arr;
H10_gust = load("../model/discretegusts/H10gust.mat").arr;
H15_gust = load("../model/discretegusts/H15gust.mat").arr;
H20_gust = load("../model/discretegusts/H20gust.mat").arr;
U = [0; 0; 0];

%% Full system time marching simulation
% Initialize states
x_old = zeros(num_states,1);
fplunge = zeros(timesteps,1);
fpitch = zeros(timesteps,1);
fbend = zeros(timesteps,1);
time = zeros(timesteps,1);

for t = 1:timesteps
    x_new = fsys.A*x_old + fsys.B*U + H5_gust(:,t);
    y = fsys.C * x_new;
    fplunge(t,1) = y(1,1);
    fpitch(t,1) = y(2,1);
    fbend(t,1) = y(3,1);
    time(t,1) = t*dt;
    x_old = x_new;
end

%% Balance Reduction
balred(fsys);
% [rsys, info] = balred(fsys,20);

[sys,g,T,Ti] = balreal(fsys);
elim = (g<1e-3);
rsys = modred(sys,elim, "Truncate");
truncated_state = size(rsys.A,1)

A_rsys = rsys.A;
B_rsys = rsys.B;
C_rsys = rsys.C;
D_rsys = rsys.D;

T_rom = T(1:truncated_state, 1:end);
Ti_rom = Ti(1:end, 1:truncated_state);

%% Gusts
rom_gust5 = T_rom * H5_gust;
rom_gust10 = T_rom * H10_gust;
rom_gust15 = T_rom * H15_gust;
rom_gust20 = T_rom * H20_gust;


%% ROM Time marching simulation
% Initialize states
x_old = zeros(truncated_state,1);
rplunge = zeros(timesteps,4);
rpitch = zeros(timesteps,4);
rbend = zeros(timesteps,4);
time = zeros(timesteps,4);

for t = 1:timesteps
    x_new = rsys.A*x_old + rsys.B*U + rom_gust5(:,t);
    y = rsys.C * x_new;
    rplunge(t,1) = y(1,1);
    rpitch(t,1) = y(2,1);
    rbend(t,1) = y(3,1);
    time(t,1) = t*dt;
    x_old = x_new;
end

x_old = zeros(truncated_state,1);
for t = 1:timesteps
    x_new = rsys.A*x_old + rsys.B*U + rom_gust10(:,t);
    y = rsys.C * x_new;
    rplunge(t,2) = y(1,1);
    rpitch(t,2) = y(2,1);
    rbend(t,2) = y(3,1);
    time(t,2) = t*dt;
    x_old = x_new;
end

x_old = zeros(truncated_state,1);
for t = 1:timesteps
    x_new = rsys.A*x_old + rsys.B*U + rom_gust15(:,t);
    y = rsys.C * x_new;
    rplunge(t,3) = y(1,1);
    rpitch(t,3) = y(2,1);
    rbend(t,3) = y(3,1);
    time(t,3) = t*dt;
    x_old = x_new;
end

x_old = zeros(truncated_state,1);
for t = 1:timesteps
    x_new = rsys.A*x_old + rsys.B*U + rom_gust20(:,t);
    y = rsys.C * x_new;
    rplunge(t,4) = y(1,1);
    rpitch(t,4) = y(2,1);
    rbend(t,4) = y(3,1);
    time(t,4) = t*dt;
    x_old = x_new;
end
%% Plot and compare
figure(2);
hold on;
plot(time(:,1),fplunge(:,1), "k", "DisplayName", "2528 states", "LineWidth", 3);
plot(time(:,1),rplunge(:,1), "r--", "DisplayName","18 states", "LineWidth", 3);
grid on;
%legend();
xlabel("time (s)")
ylabel("\Delta h (m)")
hold off;

figure(3);
hold on;
plot(time(:,1),fpitch(:,1)/pi*180, "k", "DisplayName", "2528 states", "LineWidth", 3);
plot(time(:,1),rpitch(:,1)/pi*180, "r--", "DisplayName","18 states", "LineWidth", 3);
grid on;
%legend();
xlabel("time (s)")
ylabel("\Delta \alpha (deg)")
hold off;

figure(4);
hold on;
plot(time(:,1),fbend(:,1)*200, "k", "DisplayName", "2528 states", "LineWidth", 3);
plot(time(:,1),rbend(:,1)*200, "r--", "DisplayName","18 states", "LineWidth", 3);
grid on;
legend();
xlabel("time (s)")
ylabel("\Delta z_{tip} (% b/2)")
hold off;

%% Plot different responses
figure(5);
hold on;
plot(time(:,1),rplunge(:,1), "DisplayName", "H = 5m", "LineWidth", 3);
plot(time(:,2),rplunge(:,2), "DisplayName", "H = 10m", "LineWidth", 3);
plot(time(:,3),rplunge(:,3), "DisplayName", "H = 15m", "LineWidth", 3);
plot(time(:,4),rplunge(:,4), "DisplayName", "H = 20m", "LineWidth", 3);
grid on;
%legend();
xlabel("time (s)")
ylabel("\Delta h (m)")
hold off;

figure(6);
hold on;
plot(time(:,1),rpitch(:,1)/pi*180, "DisplayName", "H = 5m", "LineWidth", 3);
plot(time(:,2),rpitch(:,2)/pi*180, "DisplayName", "H = 10m", "LineWidth", 3);
plot(time(:,3),rpitch(:,3)/pi*180, "DisplayName", "H = 15m", "LineWidth", 3);
plot(time(:,4),rpitch(:,4)/pi*180, "DisplayName", "H = 20m", "LineWidth", 3);
grid on;
%legend();
xlabel("time (s)")
ylabel("\Delta \alpha (deg)")
hold off;

figure(7);
hold on;
plot(time(:,1),rbend(:,1)*200, "DisplayName", "H = 5m", "LineWidth", 3);
plot(time(:,2),rbend(:,2)*200, "DisplayName", "H = 10m", "LineWidth", 3);
plot(time(:,3),rbend(:,3)*200, "DisplayName", "H = 15m", "LineWidth", 3);
plot(time(:,4),rbend(:,4)*200, "DisplayName", "H = 20m", "LineWidth", 3);
grid on;
legend();
xlabel("time (s)")
ylabel("\Delta z_{tip} (% b/2)")
hold off;

%% Bode plot
% w= {1, 100};
% 
% figure(8)
% bode(rsys, w);
% 
% fig2 = figure(9) % bode plot for cs1
% opts = bodeoptions
% opts.Title.String = '';
% opts.Title.FontSize = 10;
% opts.FreqUnits = 'Hz';
% bodeplot(rsys(1,1), w, opts);
% hold on;
% bodeplot(rsys(2,1), w, opts);
% bodeplot(rsys(3,1), w, opts);
% legend(["Plunge" "Pitch" "Tip Bending"])
% grid on;
% fontsize(fig2, scale=1.6)
% 
% 
% fig3 = figure(10) % bode plot for cs2
% opts = bodeoptions
% opts.Title.String = '';
% opts.Title.FontSize = 10;
% opts.FreqUnits = 'Hz';
% bodeplot(rsys(1,2), w, opts);
% hold on;
% bode(rsys(2,2), w, opts);
% bode(rsys(3,2), w, opts);
% %legend(["Plunge" "Pitch" "Tip Bending"])
% grid on;
% fontsize(fig3, scale=1.6)
% 
% fig4 = figure(11) % bode plot for cs3
% opts = bodeoptions
% opts.Title.String = '';
% opts.Title.FontSize = 10;
% opts.FreqUnits = 'Hz';
% bodeplot(rsys(1,3), w, opts);
% hold on;
% bode(rsys(2,3), w, opts);
% bode(rsys(3,3), w, opts);
% %legend(["Plunge" "Pitch" "Tip Bending"])
% grid on;
% fontsize(fig4, scale=1.6)