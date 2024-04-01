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
A_sys = load("../model/reduced/A_rsys.mat").A_rsys;
B_sys = load("../model/reduced/B_rsys.mat").B_rsys;
C_sys = load("../model/reduced/C_rsys.mat").C_rsys;
D_sys = load("../model/reduced/D_rsys.mat").D_rsys;

truncated_state = size(A_sys,1);
B_aug = cat(2,B_sys, eye(truncated_state));

D_sys = zeros(1,1);


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

gust_input = zeros(2528,1);
gust_input(1:208,1) = 1;

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

rom_input = T_rom*gust_input;

%% Open Loop SS
open_sys = ss(A_sys, rom_input, C_sys, D_sys, dt)

% Bode plot
w= {1, 100};

% figure(2)
% bode(open_sys, w);
[open_mag, open_phase, open_w] = bode(open_sys, w);

% hold on;
% grid on;


%% Closed Loop SS
K_optimal = load("past_trainings/lqr7/K_optimal.mat").K_optimal;

close_sys = ss(A_sys - 0.5*B_sys*K_optimal, rom_input, C_sys, D_sys, dt)

% Bode plot
w= {1, 100};

% figure(2)
[closed_mag, closed_phase, closed_w] = bode(close_sys, w);

% legend(["Open-loop" "Closed-loop"])

%% Plot Magnitude and phase

open_mag1 = reshape(open_mag(1,:,:), [], 1);
open_mag2 = reshape(open_mag(2,:,:), [], 1);
open_mag3 = reshape(open_mag(3,:,:), [], 1);
open_mag_sum = open_mag1 + open_mag2 + open_mag3;

closed_mag1 = reshape(closed_mag(1,:,:), [], 1);
closed_mag2 = reshape(closed_mag(2,:,:), [], 1);
closed_mag3 = reshape(closed_mag(3,:,:), [], 1);
closed_mag_sum = closed_mag1 + closed_mag2 + closed_mag3;

open_phase1 = reshape(open_phase(1,:,:), [], 1);
open_phase2 = reshape(open_phase(2,:,:), [], 1);
open_phase3 = reshape(open_phase(3,:,:), [], 1);
open_phase_sum = open_mag1 + open_phase2 + open_phase3;

closed_phase1 = reshape(closed_phase(1,:,:), [], 1);
closed_phase2 = reshape(closed_phase(2,:,:), [], 1);
closed_phase3 = reshape(closed_phase(3,:,:), [], 1);
closed_phase_sum = closed_phase1 + closed_phase2 + closed_phase3;

%%

figure(1);
subplot(2,1,1);
plot(open_w, mag2db(open_mag3), "LineWidth",3);
hold on;
plot(closed_w, mag2db(closed_mag3), "LineWidth",3);
grid on;
xlabel("Frequency (Rad/s)")
ylabel("Magnitude (dB)")
legend(["Open-loop" "Closed-loop"]);
fontsize(20, "points");
ylim([-70, 10])

subplot(2,1,2);
plot(open_w, mag2db(open_phase3), "LineWidth",3);
hold on;
plot(closed_w, mag2db(closed_phase3), "LineWidth",3);
grid on;
xlabel("Frequency (Rad/s)")
ylabel("Phase (deg)")
% legend(["Open-loop" "Closed-loop"]);
fontsize(20, "points");
ylim([45, 70])

