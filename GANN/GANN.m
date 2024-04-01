close all; clear all; clc;

global weights1;
global weights2;
global weights3;

weights1 = load("trained_model\model3\network_weights.mat").weights1;
weights2 = load("trained_model\model3\network_weights.mat").weights2;
weights3 = load("trained_model\model3\network_weights.mat").weights3;

%% Sim Parameters
num_chord = 8;
ref_chord = 0.2743;
V_inf = 10;
ds = 1/num_chord;
dt = ds*ref_chord/V_inf;

%% Sim control
simulation_time = 5;
timesteps = floor(simulation_time/dt);

%% State-space matrices
A_sys = load("../model/reduced/A_rsys.mat").A_rsys;
B_sys = load("../model/reduced/B_rsys.mat").B_rsys;
C_sys = load("../model/reduced/C_rsys.mat").C_rsys;
D_sys = load("../model/reduced/D_rsys.mat").D_rsys;

truncated_state = size(A_sys,1);
B_aug = cat(2,B_sys, eye(truncated_state));

D_aug = zeros(3,21);

%% Define systems
% MIMO systems
rsys = ss(A_sys, B_sys, C_sys, D_sys, dt);  % All 3 CS
raug = ss(A_sys, B_aug, C_sys, D_aug, dt);

%% Generate gust
H5_gust = load("../model/discretegusts/H5gust.mat").arr;
H10_gust = load("../model/discretegusts/H10gust.mat").arr;
H15_gust = load("../model/discretegusts/H15gust.mat").arr;
H20_gust = load("../model/discretegusts/H20gust.mat").arr;
T_rom = load("../model/reduced/T_rom.mat").T_rom;

gust1 = load("../model/contgust/contgust1.mat").gustz;
gust2 = load("../model/contgust/contgust2.mat").gustz;
gust3 = load("../model/contgust/contgust3.mat").gustz;
gust4 = load("../model/contgust/contgust4.mat").gustz;

gusta = zeros(2528, length(gust1));
gustb = zeros(2528, length(gust1));
gustc = zeros(2528, length(gust1));
gustd = zeros(2528, length(gust1));

for i= 1:timesteps
    gusta(1:208, i) = 0.006*gust1(i);
    gustb(1:208, i) = 0.006*gust2(i);
    gustc(1:208, i) = 0.006*gust3(i);
    gustd(1:208, i) = 0.006*gust4(i);
end   
%%
rom_gust5 = T_rom * H5_gust;
rom_gust10 = T_rom * H10_gust;
rom_gust15 = T_rom * H15_gust;
rom_gust20 = T_rom * H20_gust;

rom_gusta = T_rom * gusta;
rom_gustb = T_rom * gustb;
rom_gustc = T_rom * gustc;
rom_gustd = T_rom * gustd;
%% Create Kalman filter
Vd = 0.001;
Vn = [0.5, 0, 0; 0, 0.005, 0; 0, 0, 0.002];
[kalmf, Kf, P] = kalman(raug,Vd, Vn);

%% Select gust
gust_select = rom_gustd;

%% Open Loop Time marching simulation
% Initialize states
x_old = zeros(truncated_state,1);
xhat_old = zeros(truncated_state,1);
plunge_open = zeros(timesteps,1);
pitch_open = zeros(timesteps,1);
bend_open = zeros(timesteps,1);
plungehat_open = zeros(timesteps,1);
pitchhat_open = zeros(timesteps,1);
bendhat_open = zeros(timesteps,1);
T = zeros(timesteps,1);

% Setup input
u = [0; 0; 0];
uDIST = 0.01*randn(18, timesteps);
uNOISE = 0.1*randn(3, timesteps);

for t = 1:timesteps
    % Compute "Measurements"
    U = cat(1,u,gust_select(:,t));
    x_new = A_sys*x_old + B_aug*U + Vd*uDIST(:,t); %+ rom_gust5(:,t); 
    y = C_sys*x_new + Vn*uNOISE(:,t);
    plunge_open(t,1) = y(1,1);
    pitch_open(t,1) = y(2,1);
    bend_open(t,1) = y(3,1);
    T(t,1) = t*dt;
    x_old = x_new;

    % Estimate states
    xhat_new = (A_sys-Kf*C_sys)*xhat_old + B_aug*U + Kf*y;
    yhat = C_sys * xhat_new;
    plungehat_open(t,1) = yhat(1,1);
    pitchhat_open(t,1) = yhat(2,1);
    bendhat_open(t,1) = yhat(3,1);
    xhat_old = xhat_new;
end

%% Close Loop Time marching simulation
% Initialize states
x_old_close = zeros(truncated_state,1);
xhat_old_close = zeros(truncated_state,1);
y_close = zeros(1,3);
plunge_close = zeros(timesteps,1);
pitch_close = zeros(timesteps,1);
bend_close = zeros(timesteps,1);
plungehat_close = zeros(timesteps,1);
pitchhat_close = zeros(timesteps,1);
bendhat_close = zeros(timesteps,1);
input1 = zeros(timesteps,1);
input2 = zeros(timesteps,1);
input3 = zeros(timesteps,1);
T = zeros(timesteps,1);

for t = 1:timesteps
    u = controller(y_close);
    % Compute "measurements"
    x_new_close = A_sys*x_old_close + gust_select(:,t) + B_sys*u;
    y_close = C_sys * x_new_close;
    plunge_close(t,1) = y_close(1,1);
    pitch_close(t,1) = y_close(2,1);
    bend_close(t,1) = y_close(3,1);
    inputs = u;
    input1(t,1) = inputs(1);
    input2(t,1) = inputs(2);
    input3(t,1) = inputs(3);
    T(t,1) = t*dt;
    x_old_close = x_new_close;

    % Estimate states
    U = cat(1,inputs,gust_select(:,t));
    xhat_new_close = (A_sys-Kf*C_sys)*xhat_old_close + B_aug*U + Kf*y;
    %yhat = C_sys * xhat_new;
    % plungehat_open(t,1) = yhat(1,1);
    % pitchhat_open(t,1) = yhat(2,1);
    % bendhat_open(t,1) = yhat(3,1);
    xhat_old_close = xhat_new_close;
end

%% RMSE Reduction
rsme_open = sqrt(mean(bendhat_open.^2));
rsme_close = sqrt(mean(bend_close.^2));
change = (rsme_close - rsme_open)/rsme_open*100;

%% Plot results

figure(5);
plot(T,plungehat_open, "k--", LineWidth=3);
hold on;
plot(T,plunge_close, "r", LineWidth=3);
%legend(["Open loop" "Closed-loop"])
xlabel("time (s)")
ylabel("\Delta h (m)")
hold off;
grid on;
set(gcf,'position',[300,300,500,500])
fontsize(25, "points")
xlim([0 simulation_time])
% ylim([0 4.5])

figure(6);
plot(T,pitchhat_open/pi*180, "k--", LineWidth=3);
hold on;
plot(T,pitch_close/pi*180, "r", LineWidth=3);
%legend(["Open loop" "Close loop"])
xlabel("time (s)")
ylabel("\Delta \alpha (deg)")
hold off;
grid on;
set(gcf,'position',[300,300,500,500])
fontsize(25, "points")
xlim([0 simulation_time])
% ylim([-12 1])

figure(7);
plot(T,bendhat_open*200, "k--", LineWidth=3);
hold on;
plot(T,bend_close/0.5*100, "r", LineWidth=3);
legend(["Open-loop" "Closed-loop"])
xlabel("time (s)")
ylabel("\Delta z_{tip} (% b/2)")
hold off;
grid on;
set(gcf,'position',[300,300,500,500])
fontsize(25, "points")
xlim([0 simulation_time])
% ylim([-0.7 0.7])
ylim([-0.25 0.25])

figure(8);
plot(T, input1/pi*180, LineWidth=3);
hold on;
plot(T, input2/pi*180, LineWidth=3);
plot(T, input3/pi*180, LineWidth=3);
xlabel("time (s)")
ylabel("\delta (deg)")
legend(["CS1" "CS2" "CS3"])
grid on;
set(gcf,'position',[300,300,500,500])
fontsize(25, "points")
xlim([0 simulation_time])
ylim([-4.5, 2.5])

function u = controller(x)
    global weights1;
    global weights2;
    global weights3;
    
    x = reshape(x,[1,3]);
    layer1 = tanh(x*weights1);
    layer2 = tanh(layer1*weights2);
    layer3 = 20*pi/180*tanh(layer2*weights3);

    u = reshape(layer3, [3,1]);
end