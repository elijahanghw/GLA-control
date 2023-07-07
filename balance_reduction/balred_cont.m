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
gust1 = load("../model/contgust/contgust1.mat").gustz;
gust2 = load("../model/contgust/contgust2.mat").gustz;
gust3 = load("../model/contgust/contgust3.mat").gustz;
gust4 = load("../model/contgust/contgust4.mat").gustz;

gusta = zeros(length(A_sys), length(gust1));
gustb = zeros(length(A_sys), length(gust1));
gustc = zeros(length(A_sys), length(gust1));
gustd = zeros(length(A_sys), length(gust1));
gusttest = zeros(length(A_sys), length(gust1));

for i= 1:timesteps
    gusta(1:208, i) = 0.006*gust1(i);
    gustb(1:208, i) = 0.006*gust2(i);
    gustc(1:208, i) = 0.006*gust3(i);
    gustd(1:208, i) = 0.006*gust4(i);
end    

for i = 1:timesteps
    prob = rand
    if rand < 0.3
        gusttest(1:208,i) = -0.01;
    elseif rand > 0.7
          gusttest(1:208,i) = 0.01;  
    else
        gusttest(1:208,i) = 0;
    end
end

U = [0; 0; 0];

%% Full system time marching simulation
% Initialize states
x_old = zeros(num_states,1);
fplunge = zeros(timesteps,1);
fpitch = zeros(timesteps,1);
fbend = zeros(timesteps,1);
T = zeros(timesteps,1);

for t = 1:timesteps
    x_new = fsys.A*x_old + fsys.B*U + gusta(:,t);
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

A_rsys = rsys.A;
B_rsys = rsys.B;
C_rsys = rsys.C;
D_rsys = rsys.D;

T_rom = T(1:truncated_state, 1:end);
Ti_rom = Ti(1:end, 1:truncated_state);

%% Gusts
rom_gusta = T_rom * gusta;
rom_gustb = T_rom * gustb;
rom_gustc = T_rom * gustc;
rom_gustd = T_rom * gustd;
rom_gusttest = T_rom * gusttest;


%% ROM Time marching simulation
% Initialize states
x_old = zeros(truncated_state,1);
rplunge = zeros(timesteps,5);
rpitch = zeros(timesteps,5);
rbend = zeros(timesteps,5);
T = zeros(timesteps,5);

for t = 1:timesteps
    x_new = rsys.A*x_old + rsys.B*U + rom_gusta(:,t);
    y = rsys.C * x_new;
    rplunge(t,1) = y(1,1);
    rpitch(t,1) = y(2,1);
    rbend(t,1) = y(3,1);
    T(t,1) = t*dt;
    x_old = x_new;
end

x_old = zeros(truncated_state,1);
for t = 1:timesteps
    x_new = rsys.A*x_old + rsys.B*U + rom_gustb(:,t);
    y = rsys.C * x_new;
    rplunge(t,2) = y(1,1);
    rpitch(t,2) = y(2,1);
    rbend(t,2) = y(3,1);
    T(t,2) = t*dt;
    x_old = x_new;
end

x_old = zeros(truncated_state,1);
for t = 1:timesteps
    x_new = rsys.A*x_old + rsys.B*U + rom_gustc(:,t);
    y = rsys.C * x_new;
    rplunge(t,3) = y(1,1);
    rpitch(t,3) = y(2,1);
    rbend(t,3) = y(3,1);
    T(t,3) = t*dt;
    x_old = x_new;
end

x_old = zeros(truncated_state,1);
for t = 1:timesteps
    x_new = rsys.A*x_old + rsys.B*U + rom_gustd(:,t);
    y = rsys.C * x_new;
    rplunge(t,4) = y(1,1);
    rpitch(t,4) = y(2,1);
    rbend(t,4) = y(3,1);
    T(t,4) = t*dt;
    x_old = x_new;
end

x_old = zeros(truncated_state,1);
for t = 1:timesteps
    x_new = rsys.A*x_old + rsys.B*U + rom_gusttest(:,t);
    y = rsys.C * x_new;
    rplunge(t,5) = y(1,1);
    rpitch(t,5) = y(2,1);
    rbend(t,5) = y(3,1);
    T(t,5) = t*dt;
    x_old = x_new;
end
%% Plot and compare
figure(2);
hold on;
plot(T(:,1),fplunge, "k", "DisplayName", "2528 states", "LineWidth", 3);
plot(T(:,1),rplunge(:,1), "r--", "DisplayName","18 states", "LineWidth", 3);
grid on;
%legend();
xlabel("time (s)")
ylabel("Plunge (m)")
hold off;

figure(3);
hold on;
plot(T(:,1),fpitch/pi*180, "k", "DisplayName", "2528 states", "LineWidth", 3);
plot(T(:,1),rpitch(:,1)/pi*180, "r--", "DisplayName","18 states", "LineWidth", 3);
grid on;
%legend();
xlabel("time (s)")
ylabel("Pitch (deg)")
hold off;

figure(4);
hold on;
plot(T(:,1),fbend*200, "k", "DisplayName", "2528 states", "LineWidth", 2);
plot(T(:,1),rbend(:,1)*200, "r--", "DisplayName","18 states", "LineWidth", 2);
grid on;
legend();
xlabel("time (s)")
ylabel("Relative tip displacement (% b/2)")
hold off;

%% Plot different responses
figure(5);
hold on;
plot(T(:,1),rplunge(:,1), "DisplayName", "Gust A", "LineWidth", 3);
plot(T(:,2),rplunge(:,2), "DisplayName", "Gust B", "LineWidth", 3);
plot(T(:,3),rplunge(:,3), "DisplayName", "Gust C", "LineWidth", 3);
%plot(T(:,4),rplunge(:,4), "DisplayName", "Gust D", "LineWidth", 2);
%plot(T(:,5),rplunge(:,5), "DisplayName", "Gust Test", "LineWidth", 2);
grid on;
%legend();
xlabel("time (s)")
ylabel("\Delta h (m)")
xlim([0 5])
hold off;

figure(6);
hold on;
plot(T(:,1),rpitch(:,1)/pi*180, "DisplayName", "Gust A", "LineWidth", 3);
plot(T(:,2),rpitch(:,2)/pi*180, "DisplayName", "Gust B", "LineWidth", 3);
plot(T(:,3),rpitch(:,3)/pi*180, "DisplayName", "Gust C", "LineWidth", 3);
%plot(T(:,4),rpitch(:,4)/pi*180, "DisplayName", "Gust D", "LineWidth", 2);
%plot(T(:,5),rpitch(:,5)/pi*180, "DisplayName", "Gust Test", "LineWidth", 2);
grid on;
%legend();
xlabel("time (s)")
ylabel("\Delta \alpha (deg)")
xlim([0 5])
hold off;

figure(7);
hold on;
plot(T(:,1),rbend(:,1)*200, "DisplayName", "Gust A", "LineWidth", 3);
plot(T(:,2),rbend(:,2)*200, "DisplayName", "Gust B", "LineWidth", 3);
plot(T(:,3),rbend(:,3)*200, "DisplayName", "Gust C", "LineWidth", 3);
%plot(T(:,4),rbend(:,4)*200, "DisplayName", "Gust D", "LineWidth", 2);
%plot(T(:,5),rbend(:,5)*200, "DisplayName", "Gust Test", "LineWidth", 2);
grid on;
legend();
xlabel("time (s)")
ylabel("\Delta z_{tip} (% b/2)")
xlim([0 5])
hold off;

%% Bode plot
% w= {1, 100};
% 
% figure(8)
% bode(rsys, w);
% 
% fig2 = figure(9) % bode plot for cs1
% opts = bodeoptions
% opts.Title.String = 'CS1';
% opts.Title.FontSize = 10;
% opts.FreqUnits = 'Hz';
% bodeplot(rsys(1,1), w, opts);
% hold on;
% bodeplot(rsys(2,1), w, opts);
% bodeplot(rsys(3,1), w, opts);
% %legend(["Plunge" "Pitch" "Tip Bending"])
% grid on;
% fontsize(fig2, scale=1.6)
% 
% 
% fig3 = figure(10) % bode plot for cs2
% opts = bodeoptions
% opts.Title.String = 'CS2';
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
% opts.Title.String = 'CS3';
% opts.Title.FontSize = 10;
% opts.FreqUnits = 'Hz';
% bodeplot(rsys(1,3), w, opts);
% hold on;
% bode(rsys(2,3), w, opts);
% bode(rsys(3,3), w, opts);
% legend(["Plunge" "Pitch" "Tip Bending"])
% grid on;
% fontsize(fig4, scale=1.6)