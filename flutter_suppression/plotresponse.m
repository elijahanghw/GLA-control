close all; clear all; clc;

open_loop = readmatrix("displacements_H10_open.csv");
time = open_loop(:,1);
plunge_open = open_loop(:,2);
pitch_open = open_loop(:,3);
bend_open = open_loop(:,4);

lqg = readmatrix("displacements_H10_ANN.csv");
time = lqg(:,1);
plunge_lqg = lqg(:,2);
pitch_lqg = lqg(:,3);
bend_lqg = lqg(:,4);
CS1 = lqg(:,5);
CS2 = lqg(:,6);
CS3 = lqg(:,7);


figure(1)
plot(time, plunge_open, 'k', LineWidth=3)
hold on;
plot(time, plunge_lqg, 'r', LineWidth=3)
grid on;
%legend(["Open-loop" "Closed-loop"])
xlabel("time (s)")
ylabel("\Delta h (m)")
set(gcf,'position',[300,300,500,450])
fontsize(25, "points")

figure(2)
plot(time, pitch_open, 'k', LineWidth=3)
hold on;
plot(time, pitch_lqg, 'r', LineWidth=3)
grid on;
xlabel("time (s)")
ylabel("\Delta \alpha (deg)")
set(gcf,'position',[300,300,500,450])
fontsize(25, "points")

figure(3)
plot(time, bend_open, 'k', LineWidth=3)
hold on;
plot(time, bend_lqg, 'r', LineWidth=3)
legend(["Open-loop" "Closed-loop"])
grid on;
xlabel("time (s)")
ylabel("\Delta z_{tip} (% b/2)")
set(gcf,'position',[300,300,500,450])
fontsize(25, "points")

figure(4)
plot(time, CS1, LineWidth=3)
hold on;
plot(time, CS2, LineWidth=3)
plot(time, CS3, LineWidth=3)
legend(["CS1" "CS2" "CS3"])
xlabel("time (s)")
ylabel("\delta (deg)")
grid on;
set(gcf,'position',[300,300,500,450])
fontsize(25, "points")