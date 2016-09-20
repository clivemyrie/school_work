function lect3_3
%%
close all;
plot(out.north_m, out.alt_m); xlabel('north, m'); ylabel('alt, m')
grid on
title('Problem 3, Part A')
max_alt = max(out.alt_m)
idx = abs(out.alt_m) < 0.1;
north_pos_at_impact = out.north_m(idx)

min_ground_speed = min(out.groundspeed_mps)
max_ground_speed = max(out.groundspeed_mps)

end