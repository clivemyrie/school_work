function lect9_4
%%
clc;close all; clear all;
load_uavsim
% Run simulation

%%

figure; hold on
plot(out.time_s, out.static_press_Npm2)
xlabel('time, s');ylabel('Static Pressure, N/m^2');
grid on;
title('Problem 4, Part B')



end