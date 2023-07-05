gust = out.simout.Data
gustx = gust(:,1);
gusty = gust(:,2);
gustz = gust(:,3);
t = linspace(0,12,length(gustx));

plot(t,gustx, LineWidth=1)
hold on;
plot(t,gusty, LineWidth=1)
plot(t,gustz, LineWidth=1)
legend('x', 'y', 'z')