function lect9_3
%%
clc;close all; clear all;
load_uavsim
% Run simulation

%%

figure; hold on
plot(out.time_s, out.yaw_mag_deg)
plot(out.time_s, out.yaw_deg)
legend('yaw mag measurement', 'yaw')
xlabel('time, s');ylabel('yaw, deg');
grid on;
title('Problem 3, Part B')



end