% uavsim_control.m
%
% Flight control logic for uavsim
%
% Inputs:
%   Trajectory commands
%   State Feedbacks
%   Time
%
% Outputs:
%   Control surface commands
%   Autopilot state commands (for logging and plotting)
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%
function out = uavsim_control(uu,P)

% Extract variables from input vector uu
%   uu = [traj_cmds(1:3); estimates(1:23); time(1)];
k=(1:3);         traj_cmds=uu(k); % Trajectory Commands
k=k(end)+(1:23); estimates=uu(k); % Feedback state estimates
k=k(end)+(1);    time=uu(k);      % Simulation time, s

% Extract variables from traj_cmds
Va_c     = traj_cmds(1);  % commanded airspeed (m/s)
h_c      = traj_cmds(2);  % commanded altitude (m)
chi_c    = traj_cmds(3);  % commanded course (rad)

% Extract variables from estimates
pn_hat       = estimates(1);  % inertial North position, m
pe_hat       = estimates(2);  % inertial East position, m
h_hat        = estimates(3);  % altitude, m
Va_hat       = estimates(4);  % airspeed, m/s
phi_hat      = estimates(5);  % roll angle, rad
theta_hat    = estimates(6);  % pitch angle, rad
psi_hat      = estimates(7);  % yaw angle, rad
p_hat        = estimates(8);  % body frame roll rate, rad/s
q_hat        = estimates(9);  % body frame pitch rate, rad/s
r_hat        = estimates(10); % body frame yaw rate, rad/s
Vn_hat       = estimates(11); % north speed, m/s
Ve_hat       = estimates(12); % east speed, m/s
Vd_hat       = estimates(13); % downward speed, m/s
wn_hat       = estimates(14); % wind North, m/s
we_hat       = estimates(15); % wind East, m/s
future_use   = estimates(16:23);

% Initialize controls to trim (to be set with autopilot logic)
%delta_e=P.delta_e0;
delta_e=P.delta_e0;
delta_a=P.delta_a0;
delta_r=P.delta_r0;
delta_t=P.delta_t0;


% Set "first-time" flag, which is used to initialize autopilot integrators
firstTime=(time==0);

% Initialize autopilot commands (may be overwritten with autopilot logic)
phi_c = 45*pi/180; % For lect7_5
delta_a = PIR_roll_hold(phi_c, phi_hat, p_hat, firstTime, P); % For lect7_5
theta_c = 0;



% Flight control logic

% Lect 6_3 -- start
% if mod(time,20)<10
%     h_c=50;
% else
%     h_c=51;
% end

% Next line for Lect 6_4
h_c = 50;

P.altitude_kp = 0.23; % kp>0
P.altitude_ki = 0.08; % ki>0
P.altitude_kd = 0; % <-- Don?t use
theta_c = PIR_alt_hold_using_pitch(h_c, h_hat, 0, firstTime, P);
% Lect 6_3 -- end

% e.g.
%    delta_e = PIR_pitch_hold(theta_c, theta_hat, q_hat, firstTime, P);

% Lect 6_2 -- start
%theta_c = 20*pi/180; % only for 6_2
P.pitch_kp = -0.8; % kp<0
P.pitch_ki = -0.00; % ki<=0
P.pitch_kd = -0.1; % kd<0
delta_e = PIR_pitch_hold(theta_c, theta_hat, q_hat, firstTime, P);
% Lect 6_2 -- end

% Compile vector of control surface deflections
delta = [ ...
    delta_e; ...
    delta_a; ...
    delta_r; ...
    delta_t; ...
    ];

% Override control delta with manual flight delta
if P.manual_flight_flag
    delta = uavsim_manual_control(time,P);
end

% Compile autopilot commands for logging/vis
ap_command = [ ...
    Va_c; ...
    h_c; ...
    chi_c; ...
    phi_c; ...
    theta_c;
    0; ... % future use
    0; ... % future use
    0; ... % future use
    0; ... % future use
    ];

% Compile output vector
out=[delta; ap_command]; % 4+9=13

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%
%   Autopilot controllers in UAVSIM are based on "PI with rate feedback".
%   For convenience, we'll refer to "PI with rate feedback" as "PIR".
%
%   u = PIR_xxx(y_c, y, y_dot, init_flag, dt)
%     Inputs:
%       y_c:    Closed loop command
%       y:      Current system response
%       y_dot:  Rate feedback (derivative of y)
%       init_flag:  1: initialize integrator, 0: otherwise
%       dt:     Time step, seconds
%     Outputs:
%       u:      Controller output (input to Gplant)
%
%                .------.           .---- Limit on plant input
%             .->| ki/s |---.       |     (a.k.a. limit on controller output)
%             |  '------'   |+      v
%   Input     |  .------. + v  +    -. u  .------. .---.     .---.  Output
%    ---->( )-'->|  kp  |->( )->( )--|--->|Gplant|-| s |--.--|1/s|--.--->
%   y_c    ^     '------'        ^  -'    '------' '---'  |  '---'  |  y
%         -|                    -|         .------.       |         |
%          |                     '---------|  kd  |<------'y_dot    |
%          |                               '------'                 |
%          '--------------------------------------------------------'
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pitch_hold
%   - regulate pitch using elevator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u = PIR_pitch_hold(theta_c, theta_hat, q_hat, init_flag, P)

% Set up PI with rate feedback
y_c = theta_c; % Command
y = theta_hat; % Feedback
y_dot = q_hat; % Rate feedback
kp = P.pitch_kp;
ki = P.pitch_ki;
kd = P.pitch_kd;
u_lower_limit = -P.delta_e_max;
u_upper_limit = +P.delta_e_max;

% Initialize integrator (e.g. when t==0)
persistent error_int;
if( init_flag )
    error_int = 0;
end

% Perform "PI with rate feedback"
error = y_c - y;  % Error between command and response
error_int = error_int + P.Ts*error; % Update integrator
u = kp*error + ki*error_int - kd*y_dot;

% Output saturation & integrator clamping
%   - Limit u to u_upper_limit & u_lower_limit
%   - Clamp if error is driving u past limit
if u > u_upper_limit
    u = u_upper_limit;
    if ki*error>0
        error_int = error_int - P.Ts*error;
    end
elseif u < u_lower_limit
    u = u_lower_limit;
    if ki*error<0
        error_int = error_int - P.Ts*error;
    end
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% alt_hold
%   - regulate altitude using pitch
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u = PIR_alt_hold_using_pitch(h_c, h_hat, not_used, init_flag, P)

% Set up PI with rate feedback
y_c = h_c;   % Command
y = h_hat;   % Feedback
y_dot = 0;   % Rate feedback
kp = P.altitude_kp;
ki = P.altitude_ki;
kd = P.altitude_kd;
u_lower_limit = -P.theta_max;
u_upper_limit = +P.theta_max;

% Initialize integrator (e.g. when t==0)
persistent error_int;
if( init_flag )
    error_int = 0;
end

% Perform "PI with rate feedback"
error = y_c - y;  % Error between command and response
error_int = error_int + P.Ts*error; % Update integrator
u = kp*error + ki*error_int - kd*y_dot;

% Output saturation & integrator clamping
%   - Limit u to u_upper_limit & u_lower_limit
%   - Clamp if error is driving u past limit
if u > u_upper_limit
    u = u_upper_limit;
    if ki*error>0
        error_int = error_int - P.Ts*error;
    end
elseif u < u_lower_limit
    u = u_lower_limit;
    if ki*error<0
        error_int = error_int - P.Ts*error;
    end
end

end

function u = PIR_roll_hold(phi_c, phi_hat, p_hat, init_flag, P)

% Set up PI with rate feedback
y_c = phi_c;   % Command
y = phi_hat;   % Feedback
y_dot = 0;   % Rate feedback
kp = P.roll_kp;
ki = P.roll_ki;
kd = P.roll_kd;
u_lower_limit = -P.delta_a_max;
u_upper_limit = +P.delta_a_max;

% Initialize integrator (e.g. when t==0)
persistent error_int;
if( init_flag )
    error_int = 0;
end

% Perform "PI with rate feedback"
error = y_c - y;  % Error between command and response
error_int = error_int + P.Ts*error; % Update integrator
u = kp*error + ki*error_int - kd*y_dot;

% Output saturation & integrator clamping
%   - Limit u to u_upper_limit & u_lower_limit
%   - Clamp if error is driving u past limit
if u > u_upper_limit
    u = u_upper_limit;
    if ki*error>0
        error_int = error_int - P.Ts*error;
    end
elseif u < u_lower_limit
    u = u_lower_limit;
    if ki*error<0
        error_int = error_int - P.Ts*error;
    end
end

end
