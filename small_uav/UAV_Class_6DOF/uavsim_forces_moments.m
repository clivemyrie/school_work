% uavsim_forces_moments.m
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
function out = uavsim_forces_moments(uu, P)

    % Extract variables from input vector uu
    %   uu = [wind_ned(1:3); deltas(1:4); x(1:12); time(1)];
    k=(1:3);          wind_ned=uu(k);   % Total wind vector, ned, m/s
    k=k(end)+(1:4);   deltas=uu(k);     % Control surface commands: [delta_e delta_a delta_r delta_t]
    k=k(end)+(1:12);  x=uu(k);          % states
    k=k(end)+(1);     time=uu(k);       % Simulation time, s

    % Extract state variables from x
    pn    = x(1);   % North position, m
    pe    = x(2);   % East position, m
    pd    = x(3);   % Down position, m
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
    
    % compute wind vector in body frame (wind_ned is an input)
    wind_b = zeros(3,1);

    % compute airspeed Va, angle-of-attack alpha, side-slip beta
    Va=0;
    alpha=0;
    beta=0;

    % Longitudinal Aero Coefficients
    C_L = 0;
    C_D = 0;
    C_m = 0;

    % Lateral Aero Coefficients
    C_Y = 0;
    C_ell = 0;
    C_n   = 0;

    % Create and combine Forces
    f_grav_ned = P.mass * [0; 0; P.gravity]; % Newtons
    R_ned2b = eulerToRotationMatrix(phi,theta,psi);
    f_grav_b = R_ned2b*f_grav_ned;
    f_aero_b = zeros(3,1);
    f_prop_b = zeros(3,1);
    f_b = f_grav_b + f_aero_b + f_prop_b;

    % Create and combine Moments    
    m_aero_b = zeros(3,1);
    m_prop_b = zeros(3,1);
    m_b = m_aero_b + m_prop_b;

    % Compile function output
    out = [f_b; m_b]; % Length 3+3=6
    
end
