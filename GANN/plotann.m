close all; clear all; clc;

time = load("time.mat").T;

plunge_h10_open = load("plunge_h10_open.mat").plungehat_open;
plunge_h10_close = load("plunge_h10_ann.mat").plunge_close;
plunge_h15_open = load("plunge_h15_open.mat").plungehat_open;
plunge_h15_close = load("plunge_h15_ann.mat").plunge_close;
plunge_h20_open = load("plunge_h20_open.mat").plungehat_open;
plunge_h20_close = load("plunge_h20_ann.mat").plunge_close;

pitch_h10_open = load("pitch_h10_open.mat").pitchhat_open;
pitch_h10_close = load("pitch_h10_ann.mat").pitch_close;
pitch_h15_open = load("pitch_h15_open.mat").pitchhat_open;
pitch_h15_close = load("pitch_h15_ann.mat").pitch_close;
pitch_h20_open = load("pitch_h20_open.mat").pitchhat_open;
pitch_h20_close = load("pitch_h20_ann.mat").pitch_close;

figure(1)
hold on;
plot(time, plunge_h10_open, "--", LineWidth=3, Color="#0072BD")
plot(time, plunge_h10_close, "-", LineWidth=3, Color="#0072BD")

plot(time, plunge_h15_open, "--", LineWidth=3, Color="#D95319")
plot(time, plunge_h15_close, "-", LineWidth=3, Color="#D95319")

plot(time, plunge_h20_open, "--", LineWidth=3, Color="#EDB120")
plot(time, plunge_h20_close, "-", LineWidth=3, Color="#EDB120")

grid on;

xlabel("time (s)")
ylabel("\Delta h (m)")
legend(["H=10m open-loop" "H=10m closed-loop" "H=15m open-loop" "H=15m closed-loop" ...
    "H=20m open-loop" "H=20m closed-loop"], location="eastoutside")

set(gcf,'position',[300,300,1000,450])
fontsize(25, "points")

figure(2)
hold on;
plot(time, pitch_h10_open/pi*180, "--", LineWidth=3, Color="#0072BD")
plot(time, pitch_h10_close/pi*180, "-", LineWidth=3, Color="#0072BD")

plot(time, pitch_h15_open/pi*180, "--", LineWidth=3, Color="#D95319")
plot(time, pitch_h15_close/pi*180, "-", LineWidth=3, Color="#D95319")

plot(time, pitch_h20_open/pi*180, "--", LineWidth=3, Color="#EDB120")
plot(time, pitch_h20_close/pi*180, "-", LineWidth=3, Color="#EDB120")

grid on;

xlabel("time (s)")
ylabel("\Delta \alpha (deg)")
legend(["H=10m open-loop" "H=10m closed-loop" "H=15m open-loop" "H=15m closed-loop" ...
    "H=20m open-loop" "H=20m closed-loop"], location="eastoutside")

set(gcf,'position',[300,300,1000,450])
fontsize(25, "points")