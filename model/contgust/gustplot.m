close all;
clear all;
clc;

gust1 = load("contgust1.mat").gustz;
gust2 = load("contgust2.mat").gustz;
gust3 = load("contgust3.mat").gustz;
gust4 = load("contgust4.mat").gustz;

t = linspace(0,12, length(gust1));

plot(t, 0.006*gust1, LineWidth=2)
hold on;
plot(t, 0.006*gust2, LineWidth=2)
plot(t, 0.006*gust3, LineWidth=2)
plot(t, 0.006*gust4, LineWidth=2)
legend("Gust A", "Gust B", "Gust C", "Gust D")
legend('Location','eastoutside')
ylabel("w_g (m/s)")
xlabel('time (s)')
xlim([0 5])
%ylim([-1 1])
grid on
set(gcf,'position',[300,300,500,450])
fontsize(25, "points")