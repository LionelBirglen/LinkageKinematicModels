function [P, R_full, eul] = rrr_spherical_direct_kinematics(geometry, angles)
% SPHERICAL_DIRECT_KINEMATICS Computes joint and end-effector positions and orientation for a 3R spherical linkage.
%
% OBJECTIVE:
%   Calculate the joint axes coordinates of a planar 3R serial linkage (RRR manipulator)
%   given the joint angles
%
% INPUTS:
%   geometry - 1x3 vector [alpha1, alpha2, alpha3] representing arc angles (in degrees)
%              between:
%              - base and joint 1
%              - joint 1 and joint 2
%              - joint 2 and end-effector
%   angles   - 1x3 vector [theta1, theta2, theta3] of joint angles (in degrees)
%              - theta1 is rotation around Z axis (global)
%              - theta2 is rotation around the axis after alpha1
%              - theta3 is rotation around the axis after alpha2%
%
% OUTPUTS:
%   P       - 3x4 matrix of 3D positions of:
%                - joint axis 1 direction (column 1)
%                - joint axis 2 direction (column 2)
%                - joint axis 3 direction (column 3)
%                - end-effector direction (column 4)
%   R_full  - 3x3 rotation matrix of the end-effector frame relative to base
%   eul     - Euler angles of the end-effector using the ZYX convention (yaw (Z), pitch (Y), roll (X))
%
% USAGE EXAMPLE:
%   [P,R,E]=rrr_spherical_direct_kinematics([50 20 30],[-41 81.8 -102.2])
%        P =
%                 0    0.5781    0.7890    0.8218
%                 0   -0.5026   -0.2373   -0.5485
%            1.0000    0.6428    0.5667    0.1543
%        R =
%           -0.1547    0.5484    0.8218
%           -0.4753    0.6879   -0.5485
%           -0.8661   -0.4755    0.1543
%        E =
%         -108.0243   60.0090  -72.0237
%
% BY:
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Last Update: 2025/05/23
% Contact: lionel.birglen@polymtl.ca
%
% Code provided under GNU Affero General Public License v3.0

% Ensure column vectors
alpha = geometry(:);
theta = angles(:);

% Initialize P
P = zeros(3, 4);

% Base axis in xz-plane
P(:,1) = [0; 0; 1];

% First rotation and arc
R1 = rotz(theta(1));
Q1 = roty(alpha(1));
P(:,2) = R1 * Q1 * [0; 0; 1];

% Second rotation and arc
R2 = rotz(theta(2));
Q2 = roty(alpha(2));
P(:,3) = R1 * Q1 * R2 * Q2 * [0; 0; 1];

% Third rotation and arc, and full orientation
R3 = rotz(theta(3));
Q3 = roty(alpha(3));
R_full = R1 * Q1 * R2 * Q2 * R3 * Q3;
P(:,4) = R_full * [0; 0; 1];

 % Compute Euler angles (yaw (Z), pitch (Y), roll (X))
eul=rad2deg(rotm2eul(R_full));

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
