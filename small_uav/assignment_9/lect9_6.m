function lect9_6
%%
clc;close all; clear all;
load_uavsim
% Run simulation

%%

figure; hold on
plot(out.time_s, out.north_m)
plot(out.time_s, out.north_gps_m)
legend('true north pos', 'gps north pos')
xlabel('time, s');ylabel('Position, m');
grid on;
title('Problem 6, Part B')

figure; hold on
plot(out.time_s, out.east_m)
plot(out.time_s, out.east_gps_m)
legend('true east pos', 'gps east pos')
xlabel('time, s');ylabel('Position, m');
grid on;
title('Problem 6, Part C')

figure; hold on
plot(out.time_s, out.alt_m)
plot(out.time_s, out.alt_gps_m)
legend('true altitude', 'gps altitude')
xlabel('time, s');ylabel('Position, m');
grid on;
title('Problem 6, Part D')

figure; hold on
plot(out.time_s, out.groundspeed_mps)
plot(out.time_s, out.groundspeed_gps_mps)
legend('true groundspeed', 'gps groundspeed')
xlabel('time, s');ylabel('Groundspeed, m/s');
grid on;
title('Problem 6, Part E')

figure; hold on
plot(out.time_s, out.course_deg)
plot(out.time_s, out.course_gps_deg)
legend('true course', 'gps course')
xlabel('time, s');ylabel('Course, deg');
grid on;
title('Problem 6, Part F')


end