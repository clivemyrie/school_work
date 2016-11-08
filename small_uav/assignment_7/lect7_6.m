function lect7_6
%%
clc; clear all; close all

load_uavsim
% Run simulation

%% Problem 6, Part A - With Gusting

figure
plot(out.time_s, out.roll_deg);
grid on; title('Problem 6, Part A - With Gusting')
xlabel('time, s');ylabel('roll, deg')

figure
plot(out.time_s, out.p_dps);
grid on; title('Problem 6, Part A - With Gusting')
xlabel('time, s');ylabel('p, deg/sec')

figure
plot(out.time_s, out.da_deg);
grid on; title('Problem 6, Part A - With Gusting')
xlabel('time, s');ylabel('aileron deflection, deg')

S = stepinfo(out.roll_deg, out.time_s, 45)

%% Problem 6, Part B - No Gusting

figure(1); hold on
plot(out.time_s, out.roll_deg);
grid on; title('Problem 6, Part B - No Gusting, e\_phi\_max = 45, zeta = 0.9')
xlabel('time, s');ylabel('roll, deg')

figure(2); hold on
plot(out.time_s, out.da_deg);
grid on; title('Problem 6, Part B - No Gusting, e\_phi\_max = 45, zeta = 0.9')
xlabel('time, s');ylabel('aileron deflection, deg')

end