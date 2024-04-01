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
B_dist = eye(sys_size);
B_aug = cat(2,B_sys, eye(sys_size));

D_aug = zeros(3,21);


%% Generate gust
%H5_gust = load("../model/discretegusts/H5gust.mat").arr;
%H10_gust = load("../model/discretegusts/H10gust.mat").arr;
%H15_gust = load("../model/discretegusts/H15gust.mat").arr;
H20_gust = load("H20gust.mat").arr;

%% Select gust
gust_select = H20_gust;

%% Open Loop Time marching simulation
% Initialize states
x_old = zeros(sys_size,1);
plunge_open = zeros(timesteps,1);
pitch_open = zeros(timesteps,1);
bend_open = zeros(timesteps,1);
T = zeros(timesteps,1);

% Setup input
u = [0; 0; 0];

for t = 1:timesteps
    % Compute "Measurements"
    U = cat(1,u,gust_select(:,t));
    x_new = A_sys*x_old + B_sys*u + B_dist * gust_select(:,t);
    y = C_sys*x_new;
    plunge_open(t,1) = y(1,1);
    pitch_open(t,1) = y(2,1);
    bend_open(t,1) = y(3,1);
    T(t,1) = t*dt;
    x_old = x_new;

end

%% Create ROM
T_rom = load("T_rom.mat").T_rom;
Ti_rom = load("Ti_rom.mat").Ti_rom;
A_rsys = T_rom*A_sys*Ti_rom;

B_rsys = T_rom*B_sys;
B_rdist = T_rom*B_dist;

C_rsys = C_sys*Ti_rom;

rgust_select = gust_select;

rsys_size = size(A_rsys,1);

%% ROM pen Loop Time marching simulation
% Initialize states
x_rold = zeros(rsys_size,1);
plunge_ropen = zeros(timesteps,1);
pitch_ropen = zeros(timesteps,1);
bend_ropen = zeros(timesteps,1);
T = zeros(timesteps,1);

% Setup input
u = [0; 0; 0];

for t = 1:timesteps
    % Compute "Measurements"
    U = cat(1,u,gust_select(:,t));
    x_rnew = A_rsys*x_rold + B_rsys*u + B_rdist * rgust_select(:,t);
    y = C_rsys*x_rnew;
    plunge_ropen(t,1) = y(1,1);
    pitch_ropen(t,1) = y(2,1);
    bend_ropen(t,1) = y(3,1);
    T(t,1) = t*dt;
    x_rold = x_rnew;

end

%% Plot Open Loop
figure(1)
plot(T,plunge_open, 'k', LineWidth=1);
hold on;
plot(T,plunge_ropen, '--r', LineWidth=1);
xlabel("time (s)")
ylabel("plunge displacement (m)")
fig.theme = "light"
grid on;

figure(2)
plot(T,pitch_open/pi*180, 'k', LineWidth=1);
hold on;
plot(T,pitch_ropen/pi*180, '--r', LineWidth=1);
xlabel("time (s)")
ylabel("pitch angle (deg)")
grid on;
% 

figure(3)
plot(T,bend_ropen*200, 'k', LineWidth=1);
hold on;
plot(T,bend_ropen*200, '--r', LineWidth=1);
xlabel("time (s)")
ylabel("relative displacement (% b/2)")
grid on;

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
B_dist = eye(sys_size);
B_aug = cat(2,B_sys, eye(sys_size));

D_aug = zeros(3,21);


%% Generate gust
%H5_gust = load("../model/discretegusts/H5gust.mat").arr;
%H10_gust = load("../model/discretegusts/H10gust.mat").arr;
%H15_gust = load("../model/discretegusts/H15gust.mat").arr;
H20_gust = load("H20gust.mat").arr;

%% Select gust
gust_select = H20_gust;

%% Open Loop Time marching simulation
% Initialize states
x_old = zeros(sys_size,1);
plunge_open = zeros(timesteps,1);
pitch_open = zeros(timesteps,1);
bend_open = zeros(timesteps,1);
T = zeros(timesteps,1);

% Setup input
u = [0; 0; 0];

for t = 1:timesteps
    % Compute "Measurements"
    U = cat(1,u,gust_select(:,t));
    x_new = A_sys*x_old + B_sys*u + B_dist * gust_select(:,t);
    y = C_sys*x_new;
    plunge_open(t,1) = y(1,1);
    pitch_open(t,1) = y(2,1);
    bend_open(t,1) = y(3,1);
    T(t,1) = t*dt;
    x_old = x_new;

end

%% Create ROM
T_rom = load("T_rom.mat").T_rom;
Ti_rom = load("Ti_rom.mat").Ti_rom;
A_rsys = T_rom*A_sys*Ti_rom;

B_rsys = T_rom*B_sys;
B_rdist = T_rom*B_dist;

C_rsys = C_sys*Ti_rom;

rgust_select = gust_select;

rsys_size = size(A_rsys,1);

%% ROM pen Loop Time marching simulation
% Initialize states
x_rold = zeros(rsys_size,1);
plunge_ropen = zeros(timesteps,1);
pitch_ropen = zeros(timesteps,1);
bend_ropen = zeros(timesteps,1);
T = zeros(timesteps,1);

% Setup input
u = [0; 0; 0];

for t = 1:timesteps
    % Compute "Measurements"
    U = cat(1,u,gust_select(:,t));
    x_rnew = A_rsys*x_rold + B_rsys*u + B_rdist * rgust_select(:,t);
    y = C_rsys*x_rnew;
    plunge_ropen(t,1) = y(1,1);
    pitch_ropen(t,1) = y(2,1);
    bend_ropen(t,1) = y(3,1);
    T(t,1) = t*dt;
    x_rold = x_rnew;

end

%% Plot Open Loop
figure(1)
plot(T,plunge_open, 'k', LineWidth=1);
hold on;
plot(T,plunge_ropen, '--r', LineWidth=1);
xlabel("time (s)")
ylabel("plunge displacement (m)")
fig.theme = "light"
grid on;

figure(2)
plot(T,pitch_open/pi*180, 'k', LineWidth=1);
hold on;
plot(T,pitch_ropen/pi*180, '--r', LineWidth=1);
xlabel("time (s)")
ylabel("pitch angle (deg)")
grid on;
% 

figure(3)
plot(T,bend_ropen*200, 'k', LineWidth=1);
hold on;
plot(T,bend_ropen*200, '--r', LineWidth=1);
xlabel("time (s)")
ylabel("relative displacement (% b/2)")
grid on;

>>>>>>> c339e5e208710fac150003b1e3154d27225865e9
