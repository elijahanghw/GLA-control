close all;
clear all;
clc;

openloop = readmatrix("eigenvalues.csv");
closeloop = readmatrix("eigenvalues_lqg.csv");
openloop = openloop(1:53088, :);
closeloop = closeloop(1:53088, :);

scatter(openloop(:,2), openloop(:,3), [], openloop(:,1), 'x');
hold on;
scatter(closeloop(:,2), closeloop(:,3), [], closeloop(:,1));
colormap jet
hold off;
%% Extract Modes (open)

mode_1_x = [-55 -10];
mode_1_y = [4 25];
mode_1 = openloop(:,2) >= mode_1_x(1) & openloop(:,2) <= mode_1_x(2) & openloop(:,3) >= mode_1_y(1) & openloop(:,3) <= mode_1_y(2);
mode_1 = openloop(mode_1,:);
mode_1((mode_1(:,1) < 8 & mode_1(:,3) > 10), :) = [];
mode_1((mode_1(:,1) < 10 & mode_1(:,2) < -20), :) = [];
damping_1 = mode_1(:,2)./sqrt(mode_1(:,2).^2 + mode_1(:,3).^2);

mode_2_x = [-3 16];
mode_2_y = [20 35];
mode_2 = openloop(:,2) >= mode_2_x(1) & openloop(:,2) <= mode_2_x(2) & openloop(:,3) >= mode_2_y(1) & openloop(:,3) <= mode_2_y(2);
mode_2 = openloop(mode_2,:);
damping_2 = mode_2(:,2)./sqrt(mode_2(:,2).^2 + mode_2(:,3).^2);

mode_3_x = [-56 -13];
mode_3_y = [13 56];
mode_3 = openloop(:,2) >= mode_3_x(1) & openloop(:,2) <= mode_3_x(2) & openloop(:,3) >= mode_3_y(1) & openloop(:,3) <= mode_3_y(2);
mode_3 = openloop(mode_3,:);
mode_3((mode_3(:,1) < 10 & mode_3(:,2) < -30), :) = [];
mode_3((mode_3(:,1) > 10 & mode_3(:,3) < 25), :) = [];
mode_3((mode_3(:,1) < 8 & mode_3(:,3) > 22), :) = [];
mode_3((mode_3(:,1) < 11 & mode_3(:,3) > 38), :) = [];
mode_3((mode_3(:,1) < 13.2 & mode_3(:,3) > 44), :) = [];
mode_3((mode_3(:,1) > 8 & mode_3(:,3) < 14), :) = [];
mode_3((mode_3(:,1) > 11.9 & mode_3(:,2) > -15), :) = [];
mode_3((mode_3(:,1) > 9.9 & mode_3(:,3) < 17.2), :) = [];
damping_3 = mode_3(:,2)./sqrt(mode_3(:,2).^2 + mode_3(:,3).^2);

mode_4_x = [-16 -6];
mode_4_y = [42 58];
mode_4 = openloop(:,2) >= mode_4_x(1) & openloop(:,2) <= mode_4_x(2) & openloop(:,3) >= mode_4_y(1) & openloop(:,3) <= mode_4_y(2);
mode_4 = openloop(mode_4,[1 2 3]);
damping_4 = mode_4(:,2)./sqrt(mode_4(:,2).^2 + mode_4(:,3).^2);

figure(2)
hold on;
plot(mode_1(:,2), mode_1(:,3), ':k', "LineWidth", 1.5)
plot(mode_2(:,2), mode_2(:,3), ':k', "LineWidth", 1.5)
plot(mode_3(:,2), mode_3(:,3), ':k', "LineWidth", 1.5)
scatter(mode_1(:,2), mode_1(:,3), 100, mode_1(:,1), "filled")
scatter(mode_2(:,2), mode_2(:,3), 100, mode_2(:,1), "filled")
scatter(mode_3(:,2), mode_3(:,3), 100, mode_3(:,1), "filled")
%scatter(mode_4(:,2), mode_4(:,3), [], mode_4(:,1), "x")

%% Extract Modes (closed)

cmode_1_x = [-31 -8.5];
cmode_1_y = [6 15.3];
cmode_1 = closeloop(:,2) >= cmode_1_x(1) & closeloop(:,2) <= cmode_1_x(2) & closeloop(:,3) >= cmode_1_y(1) & closeloop(:,3) <= cmode_1_y(2);
cmode_1 = closeloop(cmode_1,:);
cmode_1((cmode_1(:,1) < 6 & cmode_1(:,3) > 10), :) = [];
cmode_1((cmode_1(:,1) > 10 & cmode_1(:,2) > -15), :) = [];
cdamping_1 = mode_1(:,2)./sqrt(mode_1(:,2).^2 + mode_1(:,3).^2);

cmode_2_x = [-6.8 4];
cmode_2_y = [10 36];
cmode_2 = closeloop(:,2) >= cmode_2_x(1) & closeloop(:,2) <= cmode_2_x(2) & closeloop(:,3) >= cmode_2_y(1) & closeloop(:,3) <= cmode_2_y(2);
cmode_2 = closeloop(cmode_2,:);
cdamping_2 = cmode_2(:,2)./sqrt(cmode_2(:,2).^2 + cmode_2(:,3).^2);

cmode_3_x = [-40 -11.7];
cmode_3_y = [13.6 39.8];
cmode_3 = closeloop(:,2) >= cmode_3_x(1) & closeloop(:,2) <= cmode_3_x(2) & closeloop(:,3) >= cmode_3_y(1) & closeloop(:,3) <= cmode_3_y(2);
cmode_3 = closeloop(cmode_3,:);
cmode_3((cmode_3(:,1) < 10 & cmode_3(:,2) < -30), :) = [];
cmode_3((cmode_3(:,1) > 10 & cmode_3(:,3) < 25), :) = [];
cmode_3((cmode_3(:,1) < 8 & cmode_3(:,3) > 22), :) = [];
cmode_3((cmode_3(:,1) < 11 & cmode_3(:,3) > 38), :) = [];
cmode_3((cmode_3(:,1) < 13.2 & cmode_3(:,3) > 44), :) = [];
cmode_3((cmode_3(:,1) > 8 & cmode_3(:,3) < 14), :) = [];
cmode_3((cmode_3(:,1) < 9 & cmode_3(:,3) > 36.5), :) = [];
cmode_3((cmode_3(:,1) < 6.5 & cmode_3(:,2) < 18.5), :) = [];
cmode_3((cmode_3(:,1) > 9.9 & cmode_3(:,3) < 15), :) = [];
cdamping_3 = cmode_3(:,2)./sqrt(cmode_3(:,2).^2 + cmode_3(:,3).^2);

cmode_4_x = [-45 -15];
cmode_4_y = [35 71];
cmode_4 = closeloop(:,2) >= cmode_4_x(1) & closeloop(:,2) <= cmode_4_x(2) & closeloop(:,3) >= cmode_4_y(1) & closeloop(:,3) <= cmode_4_y(2);
cmode_4 = closeloop(cmode_4,[1 2 3]);
cdamping_4 = cmode_4(:,2)./sqrt(cmode_4(:,2).^2 + cmode_4(:,3).^2);

figure(2)
hold on;
plot(cmode_1(:,2), cmode_1(:,3), ':k', "LineWidth", 1.5)
plot(cmode_2(:,2), cmode_2(:,3), ':k', "LineWidth", 1.5)
plot(cmode_3(:,2), cmode_3(:,3), ':k', "LineWidth", 1.5)
scatter(cmode_1(:,2), cmode_1(:,3), 100, cmode_1(:,1), 'filled', 's')
scatter(cmode_2(:,2), cmode_2(:,3), 100, cmode_2(:,1), 'filled', 's')
scatter(cmode_3(:,2), cmode_3(:,3), 100, cmode_3(:,1), 'filled', 's')

%scatter(cmode_4(:,2), cmode_4(:,3), [], cmode_4(:,1), 'filled')

plot([0 0], [0 50], 'LineStyle', '--', 'Color', 'k')
xlabel("Re(\lambda)")
ylabel("Im(\lambda)")
colorbar
colormap jet
grid on
h = colorbar;
ylabel(h, 'Freestream velocity (m/s)')


qw{1} = scatter(nan, nan, 100, 'filled', "k");
qw{2} = scatter(nan, nan, 100, 'filled', "ks");
legend([qw{:}], {'Open-loop','Closed-loop'}, 'location', 'best')

fontsize(20, "points");
hold off;

%%
figure(3)
subplot(2,1,1)
hold on;
% plot(mode_1(:,1), mode_1(:,2), '-s')
% plot(mode_2(:,1), mode_2(:,2), '-s')
% plot(mode_3(:,1), mode_3(:,2), '-s')
% plot(mode_4(:,1), mode_4(:,2), '-s')

%plot(mode_1(:,1), damping_1, '-s')
plot(mode_2(:,1), damping_2, '-s', "LineWidth",3)
plot(cmode_2(:,1), cdamping_2, '-s', "LineWidth",3)
%plot(mode_3(:,1), damping_3, '-s')
%plot(mode_4(:,1), damping_4, '-s')
plot([5 20], [0 0], 'LineStyle', '--', 'Color', 'k')

legend(["Open-loop" "Closed-loop"])
xlabel("Velocity (m/s)")
ylabel("Damping (\zeta)")
xlim([5 15])
grid on;
fontsize(20, "points");
hold off;

subplot(2,1,2)
hold on;
%plot(mode_1(:,1), mode_1(:,3)/(2*pi), '-s')
plot(mode_2(:,1), mode_2(:,3)/(2*pi), '-s', "LineWidth",3)
plot(cmode_2(:,1), cmode_2(:,3)/(2*pi), '-s', "LineWidth",3)
%plot(mode_3(:,1), mode_3(:,3)/(2*pi), '-s')
%plot(mode_4(:,1), mode_4(:,3)/(2*pi), '-s')
legend(["Open-loop" "Closed-loop"])
xlabel("Velocity (m/s)")
ylabel("Frequency (Hz)")
xlim([5 15])
grid on;
fontsize(20, "points");
hold off;