function [theta, ok] = rrr_spherical_inverse_kinematics(geometry, eul)
% RRR_SPHERICAL_INVERSE_KINEMATICS Analytical inverse for a 3R spherical linkage
%
% OBJECTIVE:
%   Calculate the joint angles required for a planar 3R serial linkage (RRR manipulator)
%   to reach a desired end-effector position and orientation.
%
% INPUTS:
%   geometry - 1x3 vector [alpha1, alpha2, alpha3] arc angles (deg)
%   eul      - 1x3 vector [Yaw, Pitch, Roll] Z-Y-X Euler angles (deg)
%
% OUTPUTS:
%   theta    - 2x3 array of [theta1, theta2, theta3] solutions in degrees
%              Rows correspond to elbow-up and elbow-down branches.
%              NaN returned if singular configuration.
%   ok       - flag (1 = valid solution, 0 = singular configuration)
%
% USAGE EXAMPLE:
%   [theta,ok]=rrr_spherical_inverse_kinematics([50 20 30],[-108 60 -72])
%        theta =
%          -40.9959   81.8370 -102.2426
%            7.5328  -81.8370   31.7337
%        ok =
%             1
%
% BY:
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Last Update: 2025/05/23
% Contact: lionel.birglen@polymtl.ca
%
% Code provided under GNU Affero General Public License v3.0

% Convert to radians
a1 = deg2rad(geometry(1));
a2 = deg2rad(geometry(2));
a3 = deg2rad(geometry(3));
yaw   = deg2rad(eul(1));
pitch = deg2rad(eul(2));
roll  = deg2rad(eul(3));

% Initialize ok value
ok=1;

% Target rotation matrix
R = rotz(yaw) * roty(pitch) * rotx(roll);

% Define base axes
X=[1;0;0];
Y=[0;1;0];
Z=[0;0;1];

% The end-effector axis is the fourth column of P
P4=R*Z; %=zalpha3

% Then, the axis of the last joint can be established
P3=R*roty(-a3)*Z; %=zalpha2=ztheta3

% Check if position is feasible
if abs((cos(a1)*cos(a2)-P3'*Z)/(sin(a1)*sin(a2)))>1
    warning('Unreachable configuration');
    theta = NaN(2,3);
    ok = 0;
    return;
end

% Compute second joint angle, two solutions
theta2a=+acos((cos(a1)*cos(a2)-P3'*Z)/(sin(a1)*sin(a2)));
theta2b=-acos((cos(a1)*cos(a2)-P3'*Z)/(sin(a1)*sin(a2)));

% For each of these two solution, compute first angle through 4 quadrant arctan
Ua=[cos(a1)*sin(a2)*cos(theta2a)+sin(a1)*cos(a2);sin(a2)*sin(theta2a);0];
Ub=[cos(a1)*sin(a2)*cos(theta2b)+sin(a1)*cos(a2);sin(a2)*sin(theta2b);0];

Va=[-sin(a2)*sin(theta2a);cos(a1)*sin(a2)*cos(theta2a)+sin(a1)*cos(a2);0];
Vb=[-sin(a2)*sin(theta2b);cos(a1)*sin(a2)*cos(theta2b)+sin(a1)*cos(a2);0];

theta1a=atan2((P3'*Va)/(Va'*Va),(P3'*Ua)/(Ua'*Ua));
theta1b=atan2((P3'*Vb)/(Vb'*Vb),(P3'*Ub)/(Ub'*Ub));

% Then, third joint angle can be computed
% This part could be simplified: to do
Q2a= rotz(theta1a)*roty(a1)*rotz(theta2a)*roty(a2);
Qatemp=inv(Q2a)*R*roty(-a3);
Q2b= rotz(theta1b)*roty(a1)*rotz(theta2b)*roty(a2);
Qbtemp=inv(Q2b)*R*roty(-a3);

theta3a=atan2(Qatemp(2,1),Qatemp(1,1));
theta3b=atan2(Qbtemp(2,1),Qbtemp(1,1));

% Convert output to degrees
theta = rad2deg([theta1a, theta2a, theta3a; ...
    theta1b, theta2b, theta3b]);
end

% --- helper rotations ---
function R = rotx(a)
R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end

function R = roty(a)
R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end

function R = rotz(a)
R = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
end
