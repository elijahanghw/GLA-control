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
simulation_time = 5;
timesteps = floor(simulation_time/dt)

%% State-space matrices
A_sys = load("A_rsys.mat").A_rsys;
B_sys = load("B_rsys.mat").B_rsys;
C_sys = load("C_rsys.mat").C_rsys;
D_sys = load("D_rsys.mat").D_rsys;

truncated_state = size(A_sys,1);

B_sys1 = B_sys(1:end, 1); % CS1
B_sys2 = B_sys(1:end, 2); % CS2
B_sys3 = B_sys(1:end, 3); % CS3

C_sys1 = C_sys(1, 1:end); % Plunge
C_sys2 = C_sys(2, 1:end); % Pitch
C_sys3 = C_sys(3, 1:end); % Bend

D_sys1 = D_sys(1:end, 1);

%% Define systems
% MIMO systems
rsys = ss(A_sys, B_sys, C_sys, D_sys, dt);  % All 3 CS

% SIMO systems
rsys1_123 = ss(A_sys, B_sys1, C_sys, D_sys1, dt); % input: CS1, output: plunge pitch bend
rsys2_123 = ss(A_sys, B_sys2, C_sys, D_sys1, dt); % input: CS2, output: plunge pitch bend
rsys3_123 = ss(A_sys, B_sys3, C_sys, D_sys1, dt); % input: CS3, output: plunge pitch bend

% SISO systems
rsys1_1 = ss(A_sys, B_sys1, C_sys1, 0, dt); % input: CS1, output: plunge
rsys1_2 = ss(A_sys, B_sys1, C_sys2, 0, dt); % input: CS1, output: pitch
rsys1_3 = ss(A_sys, B_sys1, C_sys3, 0, dt); % input: CS1, output: bend

rsys2_1 = ss(A_sys, B_sys2, C_sys1, 0, dt); % input: CS2, output: plunge
rsys2_2 = ss(A_sys, B_sys2, C_sys2, 0, dt); % input: CS2, output: pitch
rsys2_3 = ss(A_sys, B_sys2, C_sys3, 0, dt); % input: CS2, output: bend

rsys3_1 = ss(A_sys, B_sys3, C_sys1, 0, dt); % input: CS3, output: plunge
rsys3_2 = ss(A_sys, B_sys3, C_sys2, 0, dt); % input: CS3, output: pitch
rsys3_3 = ss(A_sys, B_sys3, C_sys3, 0, dt); % input: CS3, output: bend

%% Generate gust
discrete_gust = load("discretegusts/H5gust.mat").arr;
T_rom = load("T_rom.mat").T_rom;
rom_gust = T_rom * discrete_gust;
no_gust = zeros(size(rom_gust));
%% Bode plot
w= {1, 100};

figure(1)
bode(rsys, w);

figure(2) % bode plot for cs1
bode(rsys(1,1), w);
hold on;
bode(rsys(2,1), w);
bode(rsys(3,1), w);
legend(["Plunge" "Pitch" "Tip Bending"])

figure(3) % bode plot for cs2
bode(rsys(1,2), w);
hold on;
bode(rsys(2,2), w);
bode(rsys(3,2), w);
legend(["Plunge" "Pitch" "Tip Bending"])

figure(4) % bode plot for cs3
bode(rsys(1,3), w);
hold on;
bode(rsys(2,3), w);
bode(rsys(3,3), w);
legend(["Plunge" "Pitch" "Tip Bending"])

%% Open Loop Time marching simulation
% Initialize states
x_old = zeros(truncated_state,1);
rplunge = zeros(timesteps,1);
rpitch = zeros(timesteps,1);
rbend = zeros(timesteps,1);
T = zeros(timesteps,1);

% Setup input
U = [0; 0; 0];

for t = 1:timesteps
    x_new = A_sys*x_old + B_sys*U + rom_gust(:,t);
    y = C_sys * x_new;
    rplunge(t,1) = y(1,1);
    rpitch(t,1) = y(2,1);
    rbend(t,1) = y(3,1);
    T(t,1) = t*dt;
    x_old = x_new;
end

%% LQR Control

sys_select = rsys;

Q = 100*eye(18);
R = 5*eye(3);
%Q = 100*diag(abs(sys_select.C));
%R = 10;
[K,S,P] = lqr(sys_select,Q,R);
P_cont = log(P)/dt;

%% Close Loop Time marching simulation
% Initialize states
x_old_close = zeros(truncated_state,1);
rplunge_close = zeros(timesteps,1);
rpitch_close = zeros(timesteps,1);
rbend_close = zeros(timesteps,1);
input1 = zeros(timesteps,1);
input2 = zeros(timesteps,1);
input3 = zeros(timesteps,1);
T = zeros(timesteps,1);

for t = 1:timesteps
    x_new_close = (sys_select.A - sys_select.B*K)*x_old_close + rom_gust(:,t);
    y = C_sys * x_new_close;
    rplunge_close(t,1) = y(1,1);
    rpitch_close(t,1) = y(2,1);
    rbend_close(t,1) = y(3,1);
    inputs = -K*x_old_close;
    input1(t,1) = inputs(1);
    input2(t,1) = inputs(2);
    input3(t,1) = inputs(3);
    T(t,1) = t*dt;
    x_old_close = x_new_close;
end

%% Plot results

figure(5);
plot(T,rplunge);
hold on;
plot(T,rplunge_close);
legend(["Open loop" "Close loop"])
xlabel("time (s)")
ylabel("plunge displacement (m)")
hold off;

figure(6);
plot(T,rpitch/pi*180);
hold on;
plot(T,rpitch_close/pi*180);
legend(["Open loop" "Close loop"])
xlabel("time (s)")
ylabel("pitch angle (deg)")
hold off;

figure(7);
plot(T,rbend/0.5*100);
hold on;
plot(T,rbend_close/0.5*100);
legend(["Open loop" "Close loop"])
xlabel("time (s)")
ylabel("relative displacement (% b/2)")
hold off;

figure(8);
plot(T, input1/pi*180);
hold on;
plot(T, input2/pi*180);
plot(T, input3/pi*180);
xlabel("time (s)")
ylabel("CS deflection (deg)")
legend(["CS1" "CS2" "CS3"])