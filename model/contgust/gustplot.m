close all;
clear all;
clc;

gust1 = load("contgust1.mat").gustz;
gust2 = load("contgust2.mat").gustz;
gust3 = load("contgust3.mat").gustz;
%gust4 = load("contgust4.mat").gustz;

t = linspace(0,12, length(gust1));

plot(t, 0.006*gust1, LineWidth=1)
hold on;
plot(t, 0.006*gust2, LineWidth=1)
plot(t, 0.006*gust3, LineWidth=1)
%plot(t, 0.006*gust4, LineWidth=1)
legend("Gust A", "Gust B", "Gust C")
ylabel("w_g (m/s)")
xlabel('time (s)')
xlim([0 5])
grid on