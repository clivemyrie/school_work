function Gcl = PI_rateFeedback_TF(Gplant,kp,ki,kd)
% PI_rateFeedback_TF - Transfer Function for PI with rate feedback
%   Gcl = PI_rateFeedback_TF(Gplant,kp,ki,kd)
%     Inputs:
%       Gplant: Plant transfer function
%       kp:     Proportional gain
%       ki:     Integral gain
%       kd:     Derivative gain
%     Outputs:
%       Gcl:    Closed-loop transfer function from Input to Output
%                         
%                .------.        
%             .->| ki/s |---.     
%             |  '------'   |+      
%   Input     |  .------. + v  +      .------. .---.     .---.  Output
%    ---->( )-'->|  kp  |->( )->( )-->|Gplant|-| s |--.--|1/s|--.--->
%          ^     '------'        ^    '------' '---'  |  '---'  |
%         -|                    -|     .------.       |         |
%          |                     '-----|  kd  |<------'         |
%          |                           '------'                 |
%          '----------------------------------------------------'
%

s=tf('s');

% Closed Loop Transfer Function from Input to Output
Ginner = <code here>;
Gcl = <code here>;

% Minimum Realization (numerically cancel matching poles and zeros)
Gcl      = minreal(Gcl);
