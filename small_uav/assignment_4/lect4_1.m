function lect4_1
%%
close all
figure; hold on; subplot(5,1,1)
plot(out.time_s,out.alt_m);
xlabel('time, s'); ylabel('alt, meters')
grid on
title('Problem 1, Part C')

subplot(5,1,2);
plot(out.time_s,out.airspeed_mps);
xlabel('time, s'); ylabel('airspeed, m/s')
grid on

subplot(5,1,3);
plot(out.time_s,out.pitch_deg);
xlabel('time, s'); ylabel('pitch, degrees')
grid on

subplot(5,1,4);
plot(out.time_s,out.q_dps);
xlabel('time, s'); ylabel('pitch rate, deg/s')
grid on

subplot(5,1,5);
plot(out.time_s,out.alpha_deg);
xlabel('time, s'); ylabel('alpha, degrees')
grid on