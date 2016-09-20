function lect3_2
%%
close all
subplot(3,1,1)
plot(out.time_s, out.roll_deg); xlabel('time, s'); ylabel('roll, deg')
subplot(3,1,2)
plot(out.time_s, out.pitch_deg); xlabel('time, s'); ylabel('pitch, deg')
subplot(3,1,3)
plot(out.time_s, out.yaw_deg); xlabel('time, s'); ylabel('yaw, deg')

end