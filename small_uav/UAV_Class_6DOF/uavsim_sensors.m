% uavsim_sensors.m
%
% Generation of sensor measurements for uavsim
%
% Inputs:
%   Forces and Moments (used to create accelerometer measurement)
%   UAV States
%   Wind vector
%   Time
%
% Outputs:
%   Sensor Measurements
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%   
function out = uavsim_sensors(uu, P)

    % Extract variables from input vector uu
    %   uu = [f_and_m(1:6); x(1:12); wind_ned(1:3); time(1)];
    k=(1:6);           f_and_m=uu(k);   % Forces and Moments, body
    k=k(end)+(1:12);   x=uu(k);         % states
    k=k(end)+(1:3);    wind_ned=uu(k);  % wind vector, ned, m/s
    k=k(end)+(1);      time=uu(k);      % Simulation time, s

    % Extract forces and moments from f_and_m
    fb_x = f_and_m(1); % Total force along body x, N
    fb_y = f_and_m(2); % Total force along body y, N
    fb_z = f_and_m(3); % Total force along body z, N
    mb_x = f_and_m(4); % Total moment about body x, N-m
    mb_y = f_and_m(5); % Total moment about body y, N-m
    mb_z = f_and_m(6); % Total moment about body z, N-m

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

    % Gyro Measurements
    gyro_noise_pqr = randn(3,1)*P.sigma_noise_gyro;
    p_gyro = p + gyro_noise_pqr(1); % rad/s
    q_gyro = q + gyro_noise_pqr(2); % rad/s
    r_gyro = r + gyro_noise_pqr(3); % rad/s

    % Accelerometer Measurements
    accel_noise_xyz = randn(3,1)*P.sigma_noise_accel;
    ax_accel= fb_x/P.mass + accel_noise_xyz(1); % m/s^2
    ay_accel= fb_y/P.mass + accel_noise_xyz(2); % m/s^2
    az_accel= fb_z/P.mass + accel_noise_xyz(3); % m/s^2

    % Barometric Pressure Altimeter (Note: don't overwrite P structure!)
    P0 = 101325;  % Standard pressure at sea level, N/m^2
    R = 8.31432;  % Universal gas constant for air, N-m/(mol-K)
    M = 0.0289644;% Standard molar mass of atmospheric air, kg/mol
    T = 5/9*(P.air_temp_F-32)+273.15; % Air temperature in Kelvin
    persistent bias_static_press
    if(time==0)
        bias_static_press = P.sigma_bias_static_press*randn;
    end
    true_static_press = P0*exp(-M*P.gravity/R/T); % True static pressure at UAV altitude (above sea level), N/m^2
    static_press = true_static_press + bias_static_press + randn*P.sigma_bias_static_press; % Measured static pressure, N/m^2

    % Airspeed Pitot Measurment for axially mounted pitot tube
    persistent bias_diff_press
    if(time==0)
        bias_diff_press = P.sigma_bias_diff_press*randn;
    end
    Va = norm([u;v;w] - wind_ned);
    true_diff_press = 0.5 * P.rho * Va*Va; % True differential pressure at UAV airspeed
    diff_press = true_diff_press + bias_diff_press + randn*P.sigma_bias_diff_press; % Measured differential pressure, N/m^2

    % Magnetometer Measurement
    persistent bias_mag
    if(time==0)
        bias_mag = P.sigma_bias_mag*randn;
    end
    psi_mag= psi + bias_mag + randn(1,1)*P.sigma_bias_mag; % Magnetometer measurement, rad

    % GPS Position and Velocity Measurements
    gps_error(1) = randn*P.sigma_eta_gps_north;
    gps_error(2) = randn*P.sigma_eta_gps_east;
    gps_error(3) = randn*P.sigma_eta_gps_alt;
    persistent time_gps_prev ...
               gps_north_error gps_east_error gps_alt_error ...
               pn_gps pe_gps alt_gps Vn_gps Ve_gps Vd_gps
    if(time==0)
        gps_north_error = gps_error(1);
        gps_east_error = gps_error(2);
        gps_alt_error = gps_error(3);
        time_gps_prev = -inf; % Force update at time==0
    end
    if(time>time_gps_prev+P.Ts_gps)
        
        % Gauss-Markov growth of GPS position errors
        gps_north_error = exp(-P.Ts_gps/P.tau_gps)*gps_north_error + P.sigma_eta_gps_north*randn*sqrt(P.Ts_gps);
        gps_east_error  = exp(-P.Ts_gps/P.tau_gps)*gps_east_error + P.sigma_eta_gps_east*randn*sqrt(P.Ts_gps);
        gps_alt_error   = exp(-P.Ts_gps/P.tau_gps)*gps_alt_error + P.sigma_eta_gps_alt*randn*sqrt(P.Ts_gps);

        % GPS Position Measurements
        pn_gps = pn + gps_north_error;
        pe_gps = pe + gps_east_error;
        alt_gps= -pd + gps_alt_error;

        % GPS Velocity Measurements
        Vn_gps = u + randn*P.sigma_noise_gps_speed;
        Ve_gps = v + randn*P.sigma_noise_gps_speed;
        Vd_gps = w + randn*P.sigma_noise_gps_speed;

        time_gps_prev = time;
    end

    % Compile output vector
    out = [ ...
            pn_gps; ...
            pe_gps; ...
            alt_gps;...
            Vn_gps; ...
            Ve_gps; ...
            Vd_gps; ...
            p_gyro; ...
            q_gyro; ...
            r_gyro; ...
            ax_accel;...
            ay_accel;...
            az_accel;...
            static_press; ...
            diff_press; ...
            psi_mag;...        
            0; % future use
            0; % future use
            0; % future use
          ]; % Length: 18

end