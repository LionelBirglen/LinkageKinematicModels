function [P1, P2, P3] = rrr_direct_kinematics(L1, L2, L3, theta1, theta2, theta3)
% RRR_DIRECT_KINEMATICS - Compute forward kinematics of a planar RRR manipulator
%
% OBJECTIVE:
%   Compute the positions of each joint and the end-effector for a planar
%   3R serial linkage (RRR manipulator) given the link lengths and joint
%   angles.
%
% INPUTS:
%   L1, L2, L3   - lengths of link 1, link 2, and link 3 (units, e.g., mm)
%   theta1       - absolute angle of link 1 relative to base frame (rad)
%   theta2       - relative angle between link 1 and link 2 (rad)
%   theta3       - relative angle between link 2 and link 3 (rad)
%
% OUTPUTS:
%   P1 - [x1, y1] coordinates of the first joint (end of link 1)
%   P2 - [x2, y2] coordinates of the second joint (end of link 2)
%   P3 - [x3, y3] coordinates of the end-effector (end of link 3)
%
% USAGE EXAMPLE:
%   [P1, P2, P3] = rrr_direct_kinematics(57, 46, 51, deg2rad(39), deg2rad(37), deg2rad(40))
%       P1 =
%           44.2973   35.8713
%       P2 =
%           55.4257   80.5049
%       P3 =
%           33.0688  126.3434
%
% BY: 
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Last Update: 2025/05/15
% Contact: lionel.birglen@polymtl.ca
% 
% Code provided under GNU Affero General Public License v3.0


% Base joint at origin
P0 = [0, 0];

% Compute absolute angles of each link
A1 = theta1;                   % Absolute orientation of link 1
A2 = theta1 + theta2;          % Absolute orientation of link 2
A3 = A2 + theta3;              % Absolute orientation of link 3

% Compute joint positions via vector addition
P1 = P0 + L1 * [cos(A1), sin(A1)]; % Position of joint 1
P2 = P1 + L2 * [cos(A2), sin(A2)]; % Position of joint 2
P3 = P2 + L3 * [cos(A3), sin(A3)]; % Position of end-effector
end