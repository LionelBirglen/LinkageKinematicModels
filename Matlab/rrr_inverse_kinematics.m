function sol = rrr_inverse_kinematics(geo, Px, Py, phi, elbow_config)
% RRR_INVERSE_KINEMATICS - Inverse kinematics of a planar RRR serial manipulator
%
% OBJECTIVE:
%   Compute the joint angles required for a planar 3R serial chain to reach
%   a desired end-effector position and orientation.
%
% INPUTS:
%   geo          - geometry struct or vector:
%                  Struct fields: .L1, .L2, .L3  (link lengths, consistent units)
%                  Vector form:   [L1, L2, L3]
%   Px, Py       - desired end-effector position
%   phi          - desired end-effector orientation (rad)
%   elbow_config - +1 for elbow-up, -1 for elbow-down
%
% OUTPUT:
%   sol - scalar struct with fields:
%     .Positions.O  - [x;y] base pivot (always [0;0])
%     .Positions.A  - [x;y] end of link 1 / joint 1
%     .Positions.B  - [x;y] end of link 2 / joint 2
%     .Positions.P  - [x;y] end-effector = [Px;Py]
%     .theta1       - joint 1 absolute angle (rad)
%     .theta2       - joint 2 relative angle (rad)
%     .theta3       - joint 3 relative angle (rad)
%     .phi          - end-effector orientation = phi (rad)
%     .valid        - true if target is reachable, false otherwise
%
% USAGE EXAMPLE:
%   geo = struct('L1',57,'L2',46,'L3',51);
%   sol = rrr_inverse_kinematics(geo, 35, 125, deg2rad(116), 1)
%     sol.theta1 = 0.6571  (rad)
%     sol.theta2 = 0.6447  (rad)
%     sol.theta3 = 0.7228  (rad)
%     sol.valid  = 1
%
% BY:
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Last Update: 2025/11/05
% Contact: lionel.birglen@polymtl.ca
%
% Code provided under GNU Affero General Public License v3.0

[L1, L2, L3] = rrr_parse_geo(geo);

Wx = Px - L3 * cos(phi);
Wy = Py - L3 * sin(phi);
R2 = Wx^2 + Wy^2;
cos_theta2 = (R2 - L1^2 - L2^2) / (2 * L1 * L2);

sol.Positions.O = [0; 0];
sol.Positions.A = [NaN; NaN];
sol.Positions.B = [NaN; NaN];
sol.Positions.P = [Px; Py];
sol.theta1      = NaN;
sol.theta2      = NaN;
sol.theta3      = NaN;
sol.phi         = phi;
sol.valid       = false;

if abs(cos_theta2) > 1, return; end

theta2 = atan2(elbow_config * sqrt(1 - cos_theta2^2), cos_theta2);
k1     = L1 + L2 * cos(theta2);
k2     = L2 * sin(theta2);
theta1 = atan2(Wy, Wx) - atan2(k2, k1);
theta3 = phi - (theta1 + theta2);

A1 = theta1;
A2 = theta1 + theta2;
A  = [L1*cos(A1); L1*sin(A1)];
B  = A + [L2*cos(A2); L2*sin(A2)];

sol.Positions.A = A;
sol.Positions.B = B;
sol.theta1      = theta1;
sol.theta2      = theta2;
sol.theta3      = theta3;
sol.valid       = true;
end

function [L1, L2, L3] = rrr_parse_geo(geo)
if isnumeric(geo)
    if numel(geo) < 3
        error('rrr_inverse_kinematics:BadGeo','geo vector must have at least 3 elements [L1 L2 L3].');
    end
    g  = geo(:).';
    L1 = g(1); L2 = g(2); L3 = g(3);
elseif isstruct(geo)
    L1 = geo.L1; L2 = geo.L2; L3 = geo.L3;
else
    error('rrr_inverse_kinematics:BadGeo','geo must be a numeric vector [L1 L2 L3] or a struct with fields L1,L2,L3.');
end
end
