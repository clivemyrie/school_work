% Create a 3x3 rotation matrix from Euler angles.
% Uses ZYX rotation order:
%   Rotation 1: psi about z
%   Rotation 2: theta about y
%   Rotation 3: phi about x
%
% R = eulerToRotationMatrix(phi,theta,psi)
%       Inputs:
%         phi:   rotation about x, radians
%         theta: rotation about y, radians
%         psi:   rotation about z, radians
%       Output:
%         R:     3x3 rotation matrix (i.e. direction cosine matrix)
%
% Example: For an aircraft using standard nomenclature
%          (phi: roll, theta: pitch, psi: yaw), the resulting rotation
%          matrix expresses the rotation from a North-East-Down coordinate
%          frame to an aircraft body coordinate frame
%
%          R_ned2b = eulerToRotationMatrix(phi,theta,psi)
%
function R = eulerToRotationMatrix(phi,theta,psi)

% Replace the following with appropriate code
R = eye(3);
sinPsi = sin(psi);
cosPsi = cos(psi);
sinTheta = sin(theta);
cosTheta = cos(theta);
sinPhi = sin(phi);
cosPhi = cos(phi);

R = [cosTheta*cosPsi,cosTheta*sinPsi,-sinTheta;
    sinPhi*sinTheta*cosPsi-cosPhi*sinPsi,sinPhi*sinTheta*sinPsi+cosPhi*cosPsi,sinPhi*cosTheta;
    cosPhi*sinTheta*cosPsi+sinPhi*sinPsi,cosPhi*sinTheta*sinPsi-sinPhi*cosPsi,cosPhi*cosTheta];



end
