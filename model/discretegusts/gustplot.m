close all;
clear all;
clc;

t = linspace(0,5,1000);
s = 10*t;
Uds = 0.2*10;

%% Gust 5
H = 5;
gust5 = Uds/2*(1-cos(pi*s/H));
gust5(2*H/10*200:end) = 0;

%% Gust 10
H = 10;
gust10 = Uds/2*(1-cos(pi*s/H));
gust10(2*H/10*200:end) = 0;

%% Gust 15
H = 15;
gust15 = Uds/2*(1-cos(pi*s/H));
gust15(2*H/10*200:end) = 0;

%% Gust 20
H = 20;
gust20 = Uds/2*(1-cos(pi*s/H));
gust20(2*H/10*200:end) = 0;


%% Plot
figure()
hold on;
plot(s,gust5, LineWidth=1.5)
plot(s,gust10, LineWidth=1.5)
plot(s,gust15, LineWidth=1.5)
plot(s,gust20, LineWidth=1.5)

grid on;
xlabel("Distance (m)")
ylabel("w_g (m/s)")
legend(["H = 5m" "H = 10m" "H = 15m" "H = 20m"])
