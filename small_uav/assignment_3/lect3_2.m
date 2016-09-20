function lect3_2
%%
close all
subplot(3,1,1)
plot(out.time_s, out.roll_deg); xlabel('time, s'); ylabel('roll, deg')
grid on
title('Problem 2 Part C')
subplot(3,1,2)
plot(out.time_s, out.pitch_deg); xlabel('time, s'); ylabel('pitch, deg')
grid on
subplot(3,1,3)
plot(out.time_s, out.yaw_deg); xlabel('time, s'); ylabel('yaw, deg')
grid on

end