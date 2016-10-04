function lect4_1
%%

figure;
plot(out.time_s,out.alt_m);
xlabel('time, s'); ylabel('alt, meters')
grid on

figure;
plot(out.time_s,out.airspeed_mps);
xlabel('time, s'); ylabel('airspeed, m/s')
grid on

figure;
plot(out.time_s,out.pitch_deg);
xlabel('time, s'); ylabel('pitch, degrees')
grid on

figure;
plot(out.time_s,out.q_dps);
xlabel('time, s'); ylabel('pitch rate, deg/s')
grid on

figure;
plot(out.time_s,out.alpha_deg);
xlabel('time, s'); ylabel('alpha, degrees')
grid on