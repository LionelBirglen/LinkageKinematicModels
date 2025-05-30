function [theta,ok,P1,P2] = fourbar_spherical_inverse_kinematics(arcAngles, alpha)
% FOURBAR_SPHERICAL_INVERSE_KINEMATICS Inverse kinematics of a spherical four-bar linkage
%
% OBJECTIVE:
%   theta = FOURBAR_SPHERICAL_DIRECT_KINEMATICS(arcAngles, alpha) returns the
%   two possible input crank angles theta (in degrees) for a spherical four-bar linkage
%   given its link spherical arc lengths and the output crank angle.
%
% INPUTS:
%     arcAngles : 1x4 vector of spherical link arc angles [eta1, eta2, eta3, eta4] (degrees)
%                 where
%                   eta1 = arc between output and coupler axes (output link)
%                   eta2 = arc between coupler axes (coupler link)
%                   eta3 = arc between input and coupler axes (input link)
%                   eta4 = arc between fixed axes (ground link)
%     alpha     : output crank angle (degrees) measured from fixed link
%
% OUTPUTS:
%     theta     : 1x2 vector of the two possible input crank angles (degrees)
%                 measured from fixed link
%
% USAGE EXAMPLE:
%   >> [A,B,C,D]=fourbar_spherical_inverse_kinematics([45 60 45 90],-26.8990)
%       A =
%           120.0001  288.6858
%        B =
%             1
%        C =
%                 0    0.6306    0.7071    1.0000
%                 0   -0.3199    0.6124         0
%            1.0000    0.7071    0.3536    0.0000
%       D =
%                 0    0.6306    0.7071    1.0000
%                 0   -0.3199   -0.6698         0
%            1.0000    0.7071   -0.2265    0.0000
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
alpha_rad = deg2rad(alpha);

% Default output is ok
ok=1;

% Setting up known axes
P1(1:3,1)=[0;0;1];
P2(1:3,1)=[0;0;1];
c=(roty(rad2deg(eta4))*[0; 0; 1]);
P1(1:3,4)=c;
P2(1:3,4)=c;

% Compute A axis
a=(rotz(alpha)* roty(rad2deg(eta1))*[0; 0; 1]);
P1(1:3,2)=a;
P2(1:3,2)=a;
%Correspond to the following sequence of rotations around mobile axes:
% start from z
% turn alpha around z
% turn eta1 around y'

% Allows to define beta the arc length AC
beta=acos(c'*a);    

% Compute OCA angle (delta) using spherical law of cosines and sines
delta=atan2(sin(alpha_rad)*sin(eta1)/sin(beta),(cos(eta1)-cos(eta4)*cos(beta))/(sin(eta4)*sin(beta)));

% Compute angle ACB (epsilon) using spherical law of cosines
K=(cos(eta2)-cos(eta3)*cos(beta))/(sin(eta3)*sin(beta));
epsilon=acos(K);

% Check for feasible real solutions
if abs(K) > 1
%    warning('Unreachable configuration');
    theta=NaN(2);
    ok=0;
    return;
end

% Compute two possible solutions
t1 = (pi-delta - epsilon);
t2 = (pi-delta + epsilon);

% Define last axis
P1(1:3,3)=(roty(rad2deg(eta4))*rotz(rad2deg(t1))* roty(rad2deg(eta3))*[0; 0; 1]);
P2(1:3,3)=(roty(rad2deg(eta4))*rotz(rad2deg(t2))* roty(rad2deg(eta3))*[0; 0; 1]);

% Convert outputs to degrees in range [-180,180]
theta = (rad2deg([t1, t2]));
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