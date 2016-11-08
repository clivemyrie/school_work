function lect9_1
%%
clc;close all; clear all;
load_uavsim
% Run simulation

%%

figure; hold on
plot(out.time_s, out.p_dps, 'b')
plot(out.time_s, out.p_gyro_dps, 'r')
legend('roll rate','gyro roll rate')
xlabel('time, s');ylabel('roll rate, deg/s');
grid on
title('Problem 1, Part C')



end