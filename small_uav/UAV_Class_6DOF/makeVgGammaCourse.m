% makeVgGammaCourse.m: Create groundspeed, flight path angle (gamma) and 
%   course from a ground-relative vector in NED coordinates
%
%   [Vg gamma course] = makeVgGammaCourse(vg_ned)
%      Inputs: 
%        vg_ned:  3-element vector representing ground-relative velocity
%                 in NED coordinates
%      Outputs:
%        Vg:      Airspeed (scalar). (units are same as input vector)
%        gamma:   Vertical flight path angle, radians (+Up)
%        course:  Course (horiz. flight path angle), radians (+East-of-North)
%
function [Vg, gamma, course] = makeVgGammaCourse(vg_ned)

    % Replace the following with appropriate code (use atan2!)
    Vg = norm(vg_ned);
    gamma=asin(-vg_ned(3)/Vg); % radians
    course=asin(vg_ned(2)/Vg/cos(gamma)); % radians

end
