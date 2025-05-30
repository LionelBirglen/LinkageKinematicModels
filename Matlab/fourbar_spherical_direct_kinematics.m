function [alpha,ok,P1,P2] = fourbar_spherical_direct_kinematics(arcAngles, theta)
% FOURBAR_SPHERICAL_DIRECT_KINEMATICS Direct kinematics of a spherical four-bar linkage
%
%   alpha = FOURBAR_SPHERICAL_DIRECT_KINEMATICS(arcAngles, theta) returns the
%   two possible output crank angles phi (in degrees) for a spherical four-bar linkage
%   given its link spherical arc lengths and the input crank angle.
%
%   Inputs:
%     arcAngles : 1x4 vector of spherical link arc angles [eta1, eta2, eta3, eta4] (degrees)
%                 where
%                   eta1 = arc between output and coupler axes (output link)
%                   eta2 = arc between coupler axes (coupler link)
%                   eta3 = arc between input and coupler axes (input link)
%                   eta4 = arc between fixed axes (ground link)
%     theta     : input crank angle (degrees) measured from fixed link
%
%   Output:
%     alpha     : 1x2 vector of the two possible output crank angles (degrees)
%                 measured from fixed link
%
%   Example :
%   [A,B,C,D]=fourbar_spherical_direct_kinematics([45 60 45 90],120)
%       A =
%           -26.8990  108.6857
%        B =
%             1
%        C =
%                 0    0.6306    0.7071    1.0000
%                 0   -0.3199    0.6124         0
%            1.0000    0.7071    0.3536    0.0000
%       D =
%                 0   -0.2265    0.7071    1.0000
%                 0    0.6698    0.6124         0
%            1.0000    0.7071    0.3536    0.0000
%
% BY:
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Last Update: 2025/05/30
% Contact: lionel.birglen@polymtl.ca
%
% Code provided under GNU Affero General Public License v3.0

% Extract arcs (degrees -> radians)
eta1 = deg2rad(arcAngles(1));
eta2 = deg2rad(arcAngles(2));
eta3 = deg2rad(arcAngles(3));
eta4 = deg2rad(arcAngles(4));

% Convert input angle to radians
theta_rad = deg2rad(theta);

% Default output is ok
ok=1;

% Setting up known axes
P1(1:3,1)=[0;0;1];
P2(1:3,1)=[0;0;1];
P1(1:3,4)=roty(rad2deg(eta4))*[0; 0; 1];
P2(1:3,4)=roty(rad2deg(eta4))*[0; 0; 1];

% Compute B axis
b=(roty(rad2deg(eta4))*rotz(theta)*roty(rad2deg(eta3)))*[0; 0; 1];
P1(1:3,3)=b;
P2(1:3,3)=b;
% Correspond to the following sequence of rotations around mobile axes:
% start from z
% turn eta4 around y
% turn theta around z'
% turn eta3 around y''

% Allows to define beta the arc length OB
beta=acos([0;0;1]'*b); 

% Compute COB angle (delta) using spherical law of cosines and sines
delta=atan2(sin(theta_rad)*sin(eta3)/sin(beta),(cos(eta3)-cos(eta4)*cos(beta))/(sin(eta4)*sin(beta)));

% Compute angle AOB (epsilon) using spherical law of cosines
K=(cos(eta2)-cos(eta1)*cos(beta))/(sin(eta1)*sin(beta));
epsilon=acos(K);

% Check for feasible real solutions
if abs(K) > 1
    %warning('Unreachable configuration');
    alpha=NaN(2);
    ok=0;
    return;
end

% Compute two possible solutions
a1 = (delta - epsilon);
a2 = (delta + epsilon);

% Define last axis
P1(1:3,2)=(rotz(rad2deg(a1))*roty(rad2deg(eta1)))*[0; 0; 1];
P2(1:3,2)=(rotz(rad2deg(a2))*roty(rad2deg(eta1)))*[0; 0; 1];

% Convert outputs to degrees in range [-180,180]
alpha = (rad2deg([a1, a2]));
end

% --- Helper rotation functions (in degrees) ---
function R = rotx(a)
a = deg2rad(a);
R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end

function R = roty(a)
a = deg2rad(a);
R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end

function R = rotz(a)
a = deg2rad(a);
R = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
end

