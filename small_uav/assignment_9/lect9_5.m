function lect9_5
%%
clc;close all; clear all;
load_uavsim
% Run simulation

%%

figure; hold on
plot(out.time_s, out.diff_press_Npm2)
xlabel('time, s');ylabel('Diff. Pressure, N/m^2');
grid on;
title('Problem 5, Part B')



end