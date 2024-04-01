<<<<<<< HEAD
clear all;
close all;
clc;

%% Sim Parameters
num_chord = 8;
ref_chord = 0.2743;
V_inf = 10;
ds = 1/num_chord;
dt = ds*ref_chord/V_inf;

%% Sim control
simulation_time = 10;
timesteps = floor(simulation_time/dt);

%% State-space matrices
A_sys = load("A_fsys.mat").arr;
B_sys = load("B_fsys.mat").arr;
C_sys = load("C_fsys.mat").arr;
D_sys = load("D_fsys.mat").arr;

sys_size = size(A_sys,1);


%% Generate gust
%H5_gust = load("../model/discretegusts/H5gust.mat").arr;
%H10_gust = load("../model/discretegusts/H10gust.mat").arr;
%H15_gust = load("../model/discretegusts/H15gust.mat").arr;
H20_gust = load("H20gust.mat").arr;

%% Select gust
gust_select = H20_gust;

%% Create ROM
T_rom = load("T_rom.mat").T_rom;
Ti_rom = load("Ti_rom.mat").Ti_rom;
A_rsys = T_rom*A_sys*Ti_rom;

B_rsys = T_rom*B_sys;

C_rsys = C_sys*Ti_rom;

rgust_select = T_rom*gust_select;

rsys_size = size(A_rsys,1);

%% Load LQR Control
K_optimal = load("K_optimal.mat").K_optimal;

%% ROM Close Loop Time marching simulation
% Initialize states
x_rold = zeros(rsys_size,1);
plunge_rclose = zeros(timesteps,1);
pitch_rclose = zeros(timesteps,1);
bend_rclose = zeros(timesteps,1);
input1 = zeros(timesteps,1);
input2 = zeros(timesteps,1);
input3 = zeros(timesteps,1);
T = zeros(timesteps,1);

for t = 1:timesteps
    % Compute "measurements"
    x_rnew = A_rsys*x_rold + rgust_select(:,t) - B_rsys*K_optimal*x_rold;
    y = C_rsys * x_rnew;
    plunge_rclose(t,1) = y(1,1);
    pitch_rclose(t,1) = y(2,1);
    bend_rclose(t,1) = y(3,1);
    inputs = -K_optimal*x_rold;
    input1(t,1) = inputs(1);
    input2(t,1) = inputs(2);
    input3(t,1) = inputs(3);
    T(t,1) = t*dt;
    x_rold = x_rnew;
end

%% FULL Close Loop Time marching simulation
% Initialize states
x_old = zeros(sys_size,1);
plunge_close = zeros(timesteps,1);
pitch_close = zeros(timesteps,1);
bend_close = zeros(timesteps,1);
input1 = zeros(timesteps,1);
input2 = zeros(timesteps,1);
input3 = zeros(timesteps,1);
T = zeros(timesteps,1);

for t = 1:timesteps
    % Compute "measurements"
    x_new = A_sys*x_old + gust_select(:,t) - B_sys*K_optimal*T_rom*x_old;
    y = C_sys * x_new;
    plunge_close(t,1) = y(1,1);
    pitch_close(t,1) = y(2,1);
    bend_close(t,1) = y(3,1);
    inputs = -K_optimal*T_rom*x_old;
    input1(t,1) = inputs(1);
    input2(t,1) = inputs(2);
    input3(t,1) = inputs(3);
    T(t,1) = t*dt;
    x_old = x_new;
end

%% Plot results

figure(1);
plot(T,plunge_close, "k", LineWidth=3);
hold on;
plot(T,plunge_rclose, "--r", LineWidth=3);
xlabel("time (s)")
ylabel("\Delta h (m)")
hold off;
grid on;
set(gcf,'position',[300,300,500,450])
fontsize(35, "points")
xlim([0 simulation_time])

figure(2);
plot(T,pitch_close/pi*180, "k", LineWidth=3);
hold on;
plot(T,pitch_rclose/pi*180, "--r", LineWidth=3);
xlabel("time (s)")
ylabel("\Delta \alpha (deg)")
hold off;
grid on;
set(gcf,'position',[300,300,500,450])
fontsize(35, "points")
xlim([0 simulation_time])

figure(3);
plot(T,bend_close/0.5*100, "k", LineWidth=3);
hold on;
plot(T,bend_rclose/0.5*100, "--r", LineWidth=3);
xlabel("time (s)")
ylabel("\Delta z_{tip} (% b/2)")
hold off;
grid on;
set(gcf,'position',[300,300,500,450])
fontsize(35, "points")
xlim([0 simulation_time])

% figure(8);
% plot(T, input1/pi*180, LineWidth=3);
% hold on;
% plot(T, input2/pi*180, LineWidth=3);
% plot(T, input3/pi*180, LineWidth=3);
% xlabel("time (s)")
% ylabel("\delta (deg)")
% legend(["CS1" "CS2" "CS3"])
% grid on;
% set(gcf,'position',[300,300,500,450])
% fontsize(35, "points")
% xlim([0 simulation_time])

%% Compute imput rates
% input3_rate = zeros(size(input1));
% for i = 1:timesteps-1
%     input3_rate(i) = (input3(i+1) - input3(i))/dt;
% end
=======
clear all;
close all;
clc;

%% Sim Parameters
num_chord = 8;
ref_chord = 0.2743;
V_inf = 10;
ds = 1/num_chord;
dt = ds*ref_chord/V_inf;

%% Sim control
simulation_time = 10;
timesteps = floor(simulation_time/dt);

%% State-space matrices
A_sys = load("A_fsys.mat").arr;
B_sys = load("B_fsys.mat").arr;
C_sys = load("C_fsys.mat").arr;
D_sys = load("D_fsys.mat").arr;

sys_size = size(A_sys,1);


%% Generate gust
%H5_gust = load("../model/discretegusts/H5gust.mat").arr;
%H10_gust = load("../model/discretegusts/H10gust.mat").arr;
%H15_gust = load("../model/discretegusts/H15gust.mat").arr;
H20_gust = load("H20gust.mat").arr;

%% Select gust
gust_select = H20_gust;

%% Create ROM
T_rom = load("T_rom.mat").T_rom;
Ti_rom = load("Ti_rom.mat").Ti_rom;
A_rsys = T_rom*A_sys*Ti_rom;

B_rsys = T_rom*B_sys;

C_rsys = C_sys*Ti_rom;

rgust_select = T_rom*gust_select;

rsys_size = size(A_rsys,1);

%% Load LQR Control
K_optimal = load("K_optimal.mat").K_optimal;

%% ROM Close Loop Time marching simulation
% Initialize states
x_rold = zeros(rsys_size,1);
plunge_rclose = zeros(timesteps,1);
pitch_rclose = zeros(timesteps,1);
bend_rclose = zeros(timesteps,1);
input1 = zeros(timesteps,1);
input2 = zeros(timesteps,1);
input3 = zeros(timesteps,1);
T = zeros(timesteps,1);

for t = 1:timesteps
    % Compute "measurements"
    x_rnew = A_rsys*x_rold + rgust_select(:,t) - B_rsys*K_optimal*x_rold;
    y = C_rsys * x_rnew;
    plunge_rclose(t,1) = y(1,1);
    pitch_rclose(t,1) = y(2,1);
    bend_rclose(t,1) = y(3,1);
    inputs = -K_optimal*x_rold;
    input1(t,1) = inputs(1);
    input2(t,1) = inputs(2);
    input3(t,1) = inputs(3);
    T(t,1) = t*dt;
    x_rold = x_rnew;
end

%% FULL Close Loop Time marching simulation
% Initialize states
x_old = zeros(sys_size,1);
plunge_close = zeros(timesteps,1);
pitch_close = zeros(timesteps,1);
bend_close = zeros(timesteps,1);
input1 = zeros(timesteps,1);
input2 = zeros(timesteps,1);
input3 = zeros(timesteps,1);
T = zeros(timesteps,1);

for t = 1:timesteps
    % Compute "measurements"
    x_new = A_sys*x_old + gust_select(:,t) - B_sys*K_optimal*T_rom*x_old;
    y = C_sys * x_new;
    plunge_close(t,1) = y(1,1);
    pitch_close(t,1) = y(2,1);
    bend_close(t,1) = y(3,1);
    inputs = -K_optimal*T_rom*x_old;
    input1(t,1) = inputs(1);
    input2(t,1) = inputs(2);
    input3(t,1) = inputs(3);
    T(t,1) = t*dt;
    x_old = x_new;
end

%% Plot results

figure(1);
plot(T,plunge_close, "k", LineWidth=3);
hold on;
plot(T,plunge_rclose, "--r", LineWidth=3);
xlabel("time (s)")
ylabel("\Delta h (m)")
hold off;
grid on;
set(gcf,'position',[300,300,500,450])
fontsize(35, "points")
xlim([0 simulation_time])

figure(2);
plot(T,pitch_close/pi*180, "k", LineWidth=3);
hold on;
plot(T,pitch_rclose/pi*180, "--r", LineWidth=3);
xlabel("time (s)")
ylabel("\Delta \alpha (deg)")
hold off;
grid on;
set(gcf,'position',[300,300,500,450])
fontsize(35, "points")
xlim([0 simulation_time])

figure(3);
plot(T,bend_close/0.5*100, "k", LineWidth=3);
hold on;
plot(T,bend_rclose/0.5*100, "--r", LineWidth=3);
xlabel("time (s)")
ylabel("\Delta z_{tip} (% b/2)")
hold off;
grid on;
set(gcf,'position',[300,300,500,450])
fontsize(35, "points")
xlim([0 simulation_time])

% figure(8);
% plot(T, input1/pi*180, LineWidth=3);
% hold on;
% plot(T, input2/pi*180, LineWidth=3);
% plot(T, input3/pi*180, LineWidth=3);
% xlabel("time (s)")
% ylabel("\delta (deg)")
% legend(["CS1" "CS2" "CS3"])
% grid on;
% set(gcf,'position',[300,300,500,450])
% fontsize(35, "points")
% xlim([0 simulation_time])

%% Compute imput rates
% input3_rate = zeros(size(input1));
% for i = 1:timesteps-1
%     input3_rate(i) = (input3(i+1) - input3(i))/dt;
% end
>>>>>>> c339e5e208710fac150003b1e3154d27225865e9
% input3_rate = input3_rate *180 / pi;