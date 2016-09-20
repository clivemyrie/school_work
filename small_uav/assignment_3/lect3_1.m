function lect3_1
%%
close all
subplot(3,1,1)
plot(out.time_s, out.alt_m); xlabel('time, s'); ylabel('altitude, m')
subplot(3,1,2)
plot(out.time_s, out.east_m); xlabel('time, s'); ylabel('east, m')
subplot(3,1,3)
plot(out.time_s, out.north_m); xlabel('time, s'); ylabel('north, m')

end