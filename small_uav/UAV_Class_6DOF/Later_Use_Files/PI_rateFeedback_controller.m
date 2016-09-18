function u = PI_rateFeedback_controller(y_c, y, y_dot, init_flag, dt)
% PI_rateFeedback_controller - Implementation of PI with rate feedback
%   u = PI_rateFeedback_controller(y_c, y, y_dot, init_flag, dt)
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

% Set up PI with rate feedback
% (Obviously this would be a more generally useful function if the gains
% and limits were passed into the routine, but we'll just hardcode them
% because we're only going to use this particular file for Homework 2.)
kp = nan;
ki = nan;
kd = nan;
u_lower_limit = nan;
u_upper_limit = nan;

% Initialize integrator (e.g. when t==0)
% (The integrator output, error_int, is decleared as a "persistent"
% variable.  "persistent" variables retain their value between function
% calls.)
persistent error_int;
if( init_flag )   
	error_int = 0;
end  

% Error between command and response


% Update the integrator (Euler integration)


% Perform "PI with rate feedback"
u = 0;

% Output saturation & integrator clamping
%   - Limit u to u_upper_limit & u_lower_limit
%   - Clamp if error is driving u past limit

