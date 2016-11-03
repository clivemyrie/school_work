function [P, trim_solution] = compute_longitudinal_trim(P)
% Compute longitudinal trim condition for an aircraft in uavsim.
% Specifically, uses a minimization routine to find the angle-of-attack,
% elevator and throttle values that minimize the magnitudes of pitching
% moment and the forces along body x & z.
%
%    P = compute_longitudinal_trim(P)
%          sets initial states in P to a longitudinal trim condition
%          (e.g. P.theta0, P.delta_e0, etc.)
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%   

    % Note to student: You will need to complete/correct the subfunction
    % cost_function().  Look for <code here>.
    % You should not have to change anything in the
    % primary function, but please try to understand the code.
    %  -jdb
    
    % Initial "guess" of trim condition
    alphaStar = 0;    % Angle of attack, radians
    delta_eStar = 0;  % Elevator deflection, radians
    delta_aStar = 0;  % Aileron deflection, radians
    delta_tStar = 0;  % Throttle deflection
    initial_guess = [alphaStar delta_eStar delta_aStar delta_tStar];
    
    % Vary trim guesses to find values which minimize cost_function()
    % Uses Matlab routine fminsearch():
    %     Given a function "Jcost=cost_function(trim_test,P)",
    %     fminsearch() will try to find a local minimum of Jcost, starting
    %     from the condition "initial_guess".  fminsearch() will output the
    %     best trim_test it finds.
    trim_condition = ...    % Best set of alpha, delta_e and delta_t found
        fminsearch( ...
            @(trim_test)( cost_function(trim_test,P) ), ... % Inline function to minimize (see below)
            initial_guess, ... % Starting guess
            optimset('TolFun',1e-24)); % User-defined options: Stop when Jcost<1e-24

    % Above, we used an inline function because the function we want to
    % minimuze "cost_function" requires two inputs (trim_test & P), 
    % but fminsearch() only works with single-input functions.
    %
    % A note on inline functions in Matlab:
    %   You can create an inline function from the command line using @ symbol:
    %      e.g.
    %         func = @(u)(u(1) + u(2)^2);
    %      As a result, you can call the function "func":
    %         y = func([4 5]); % outputs y = 4+5^2 = 29
    %      Also, you can make use of other parameters that exist at the
    %      time of creating the inline function:
    %         p = 100;
    %         func2 = @(u)(u(1) + u(2)^2 + p);
    %         y = func2([4 5]); % outputs y = 4+5^2+100 = 129
    %   From this it should be clearer that an inline function defined as:
    %         cost_func = @(trim_test)( cost_function(trim_test,P) )
    %   would result in the following two equivalent function calls:
    %         Jcost = cost_function(trim_test,P);
    %         Jcost = cost_func(trim_test);  % calls: cost_function(trim_test,P), using the P in the workspace. 
    
    clear trim_solution

    % Extract "trim" values from fminsearch output
    alphaStar = trim_condition(1);
    delta_eStar = trim_condition(2);
    delta_aStar = trim_condition(3);
    delta_tStar = trim_condition(4);
    
    
    % Re-run cost function at trim_condition to acquire full state vector (x) 
    % and control deflections (u) at trim condition
    [Jcost, x, deltas] = cost_function(trim_condition,P);
    
    % Set initial states in P structure using trim output
    P.pn0    = x(1);
    P.pe0    = x(2);
    P.pd0    = x(3);
    P.u0     = x(4);
    P.v0     = x(5);
    P.w0     = x(6);
    P.phi0   = x(7);
    P.theta0 = x(8);
    P.psi0   = x(9);
    P.p0     = x(10);
    P.q0     = x(11);
    P.r0     = x(12);
    
    % Set initial "trim" alpha (used in making autopilot gains)
    P.alpha0 = alphaStar;

    % Set initial control deflections in P structure using trim output
    P.delta_e0 = deltas(1);
    P.delta_a0 = deltas(2);
    P.delta_r0 = deltas(3);
    P.delta_t0 = deltas(4);
        
    % Display whether a valid trim was found
    if Jcost < 1e-6 % Goal is Jcost<1e-24, but any "small" Jcost (e.g. Jcost<1e-6) is okay.
%         fprintf('******************************************************************\n');
%         fprintf('  Trim condition found, Jcost = %e\n',Jcost);
%         fprintf('  Longitudinal trim: alpha=%.4f deg, de=%.4f deg, da=%.4f deg, dt=%.4f\n', ...
%                   alphaStar*180/pi, delta_eStar*180/pi, delta_aStar*180/pi, delta_tStar);
%         fprintf('******************************************************************\n');
        trim_solution.alpha = alphaStar*180/pi;
        trim_solution.delta_e = delta_eStar*180/pi;
        trim_solution.delta_a = delta_aStar*180/pi;
        trim_solution.delta_t = delta_tStar;
        
        trim_solution.valid = 1;
        trim_solution.cost = Jcost;
        
%         P.alpha0 = alphaStar;
%         P.delta_e0 = delta_eStar;
%         P.delta_t0 = delta_tStar;
    else
%         fprintf('******************************************************************\n');
%         fprintf('  WARNING: Valid trim condition NOT found, Jcost = %e\n',Jcost);
%         fprintf('  Result invalid: alpha=%.4f deg, de=%.4f deg, da=%.4f deg, dt=%.4f\n', ...
%                   alphaStar*180/pi, delta_eStar*180/pi, delta_aStar*180/pi, delta_tStar);
%         fprintf('******************************************************************\n');
%         error('Trim condition not found')
%         disp('Trim condition not found')
        
        trim_solution.valid = 0;
        trim_solution.cost = Jcost;
    end
        
end

% Function to minimize to achieve straight-and-level longitudinal trim.
%  Generates Jcost from the output of uavsim_forces_moments().
function [Jcost, x, deltas] = cost_function(trim_test,P)
    
    % Extract values from trim_test
    alpha=trim_test(1);
    delta_e=trim_test(2);
    delta_a=trim_test(3);
    delta_t=trim_test(4);
    

    % Trimmer will assume zero wind
    wind_ned = zeros(3,1);
    
    % Construct control deflections vector
    %   delta_a and delta_r are zeroed.
    deltas = [delta_e; delta_a; 0; delta_t];

    % Construct state vector
    % Need to retain original position and yaw for initialization.
    x=zeros(12,1);
    x(1) = P.pn0;    % North position, m
    x(2) = P.pe0;    % East position, m
    x(3) = P.pd0;    % Down position, m
    x(4) = P.Va0 * cos(alpha);  % u (Hint: Use P.Va0 and alpha)
    x(5) = 0;            % v
    x(6) = P.Va0 * sin(alpha);  % w
    x(7) = 0;            % phi
    x(8) = alpha;  % theta
    x(9) = P.psi0;       % psi
    x(10)= 0;            % p
    x(11)= 0;  % q
    x(12)= 0;            % r

    time = 0;

    % Call uavsim_forces_and_moments and generate Jcost appropriately
    uu = [wind_ned;deltas;x;time]; % input to uavsim_force_moments
    f_and_m = uavsim_forces_moments(uu, P); % call uavsim_forces_moments routine
    fx = f_and_m(1);
    fz = f_and_m(3);
    ell = f_and_m(4);
    m = f_and_m(5);
    %Jcost = fx*fx + fz*fz + m*m + ell*ell;
    Jcost = fx*fx + fz*fz + m*m;

    % Propeller force is non-linear with delta_t.  Results outside of [0 1]
    % are invalid, so bump up Jcost outside of the valid ranges.
    if delta_t<0
        Jcost = Jcost + 100*abs(delta_t);
    elseif delta_t>1
        Jcost = Jcost + 100*(delta_t-1);
    end
        
end
