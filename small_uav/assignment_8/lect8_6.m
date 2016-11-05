function lect8_6
clear;clc;close all;
format compact
load_uavsim

%Run simulation

%% Problem 6, Lateral

figure;hold on;
plot(out.time_s, out.course_deg, 'b.');
plot(out.time_s, out.course_cmd_deg, 'r.');
legend('course', 'course cmd')
xlabel('time, s'); ylabel('course, deg')
grid on
title('Problem 6 - Lateral Course')

figure;hold on;
plot(out.time_s, out.roll_deg, 'b.');
plot(out.time_s, out.roll_cmd_deg, 'r.');
legend('roll', 'roll cmd')
xlabel('time, s'); ylabel('roll, deg')
grid on
title('Problem 6 - Lateral Roll')

figure;hold on;
plot(out.time_s, out.da_deg, 'b.');
xlabel('time, s'); ylabel('Aileron Deflection, deg')
grid on
title('Problem 6 - Lateral Aileron')

% Problem 6, Longitudinal

figure;hold on;
plot(out.time_s, out.airspeed_mps, 'b.');
plot(out.time_s, out.airspeed_cmd_mps, 'r.');
legend('airspeed', 'airspeed cmd')
xlabel('time, s'); ylabel('speed, m/s')
grid on
title('Problem 6 - Longitudinal Airspeed')

figure;hold on;
plot(out.time_s, out.throttle, 'b.');
xlabel('time, s'); ylabel('throttle, [0-1]')
grid on
title('Problem 6 - Longitudinal Throttle')

figure;hold on;
plot(out.time_s, out.de_deg, 'b.');
xlabel('time, s'); ylabel('Elevator, deg')
grid on
title('Problem 6 - Longitudinal Elevator')

figure;hold on;
plot(out.time_s, out.pitch_deg, 'b.');
plot(out.time_s, out.pitch_cmd_deg, 'r.');
legend('pitch', 'pitch cmd')
xlabel('time, s'); ylabel('pitch, deg')
grid on
title('Problem 6 - Longitudinal Pitch')

figure;hold on;
plot(out.time_s, out.alt_m, 'b.');
plot(out.time_s, out.alt_cmd_m, 'r.');
legend('alt', 'alt cmd')
xlabel('time, s'); ylabel('height, m')
grid on
title('Problem 6 - Longitudinal Altitude')
end