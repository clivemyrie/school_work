% uavsim_trajectory_control.m
%
% Generation of forces and moments acting on vehicle for uavsim
%
% Inputs:
%   Wind in NED frame
%   Control surfaces
%   UAV States
%   Time
%
% Outputs:
%   Forces in body frame
%   Moments in body frame
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%
function out = uavsim_trajectory_control(uu, P)

% Extract variables from input vector uu
%   uu = [wind_ned(1:3); deltas(1:4); x(1:12); time(1)];
k=(1:3);          wind_ned=uu(k);   % Total wind vector, ned, m/s
k=k(end)+(1:4);   deltas=uu(k);     % Control surface commands: [delta_e delta_a delta_r delta_t]
k=k(end)+(1:12);  x=uu(k);          % states
k=k(end)+(1);     time=uu(end);       % Simulation time, s

% Extract state variables from x
pn    = uu(1);   % North position, m
pe    = uu(2);   % East position, m
pd    = uu(3);   % Down position, m
u     = x(4);   % body-x groundspeed component, m/s
v     = x(5);   % body-y groundspeed component, m/s
w     = x(6);   % body-z groundspeed component, m/s
phi   = x(7);   % EulerAngle: roll, rad
theta = x(8);   % EulerAngle: pitch, rad
psi   = x(9);   % EulerAngle: yaw, rad
p     = x(10);  % body rate about x, rad/s
q     = x(11);  % body rate about y, rad/s
r     = x(12);  % body rate about z, rad/s

% Combine states to vector form for convenience
P_ned = [pn; pe; pd];   % NED position, m
vg_b  = [u; v; w];      % Groundspeed vector, body frame, m/s
w_b   = [p; q; r];      % body rates about x,y,z, rad/s

% Extract control commands from deltas
delta_e = deltas(1); % Elevator, radians
delta_a = deltas(2); % Aileron, radians
delta_r = deltas(3); % Rudder, radians
delta_t = deltas(4); % Throttle, 0-1

% Your code goes below...
R_ned2b = eulerToRotationMatrix(phi,theta,psi);

% compute wind vector in body frame (wind_ned is an input)
wind_b = R_ned2b*wind_ned;

% compute airspeed Va, angle-of-attack alpha, side-slip beta
[Va, alpha, beta] = makeVaAlphaBeta(vg_b - wind_b);

persistent curr_wpt_idx;

if(time == 0.0)
    curr_wpt_idx = 1;
end

% Waypoints
wp_east = [0 400 500 500 0]; %m
wp_north = [100 200 300 500 600]; %m
wp_alt = [50 60 90 90 60]; %m
wp_speed = [13 13 13 16 16]; %m/s

%disp('=======================================')
%disp(['time ' num2str(time)])
%disp(['idx ' num2str(curr_wpt_idx)])
dist_to_wpt = sqrt((pn-wp_north(curr_wpt_idx))^2 + (pe-wp_east(curr_wpt_idx))^2)

if(dist_to_wpt <=20)
    curr_wpt_idx = curr_wpt_idx + 1;
    if (curr_wpt_idx > 5)
        curr_wpt_idx = 1;
    end
end

Va_c = wp_speed(curr_wpt_idx);
h_c = wp_alt(curr_wpt_idx);

%Vg = norm(vg_ned);
%gamma=asin(-vg_ned(3)/Vg); % radians
%course=asin(vg_ned(2)/Vg/cos(gamma)); % radians

n_diff = wp_north(curr_wpt_idx) - pn;
e_diff = wp_east(curr_wpt_idx) - pe;

chi_c = (pi/2) - atan2(n_diff, e_diff);
%chi_c_deg = chi_c *180/pi

% Compile function output
out = [Va_c; h_c; chi_c]; % Length 3+3=6

end
