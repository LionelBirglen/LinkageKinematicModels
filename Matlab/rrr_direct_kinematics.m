function sol = rrr_direct_kinematics(geo, theta1, theta2, theta3)
% RRR_DIRECT_KINEMATICS - Direct kinematics of a planar RRR serial manipulator
%
% OBJECTIVE:
%   Compute joint and end-effector positions for a planar 3R serial chain
%   given link lengths and joint angles.
%
% INPUTS:
%   geo    - geometry struct or vector:
%            Struct fields: .L1, .L2, .L3  (link lengths, consistent units)
%            Vector form:   [L1, L2, L3]
%   theta1 - absolute angle of link 1 w.r.t. base frame (rad)
%   theta2 - relative angle between link 1 and link 2 (rad)
%   theta3 - relative angle between link 2 and link 3 (rad)
%
% OUTPUT:
%   sol - scalar struct with fields:
%     .Positions.O  - [x;y] base pivot (always [0;0])
%     .Positions.A  - [x;y] end of link 1 / joint 1
%     .Positions.B  - [x;y] end of link 2 / joint 2
%     .Positions.P  - [x;y] end-effector (end of link 3)
%     .phi          - absolute end-effector orientation (rad) = theta1+theta2+theta3
%     .valid        - always true for direct kinematics
%
% USAGE EXAMPLE:
%   geo = struct('L1',57,'L2',46,'L3',51);
%   sol = rrr_direct_kinematics(geo, deg2rad(39), deg2rad(37), deg2rad(40))
%     sol.Positions.P =
%         33.0688
%        126.3434
%     sol.phi = 2.0071   (rad)
%
% BY:
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Last Update: 2025/11/05
% Contact: lionel.birglen@polymtl.ca
%
% Code provided under GNU Affero General Public License v3.0

[L1, L2, L3] = rrr_parse_geo(geo);

A1 = theta1;
A2 = theta1 + theta2;
A3 = A2    + theta3;

O  = [0; 0];
A  = O  + L1 * [cos(A1); sin(A1)];
B  = A  + L2 * [cos(A2); sin(A2)];
P  = B  + L3 * [cos(A3); sin(A3)];

sol.Positions.O = O;
sol.Positions.A = A;
sol.Positions.B = B;
sol.Positions.P = P;
sol.phi         = A3;
sol.valid       = true;
end

function [L1, L2, L3] = rrr_parse_geo(geo)
if isnumeric(geo)
    if numel(geo) < 3
        error('rrr_direct_kinematics:BadGeo','geo vector must have at least 3 elements [L1 L2 L3].');
    end
    g  = geo(:).';
    L1 = g(1); L2 = g(2); L3 = g(3);
elseif isstruct(geo)
    L1 = geo.L1; L2 = geo.L2; L3 = geo.L3;
else
    error('rrr_direct_kinematics:BadGeo','geo must be a numeric vector [L1 L2 L3] or a struct with fields L1,L2,L3.');
end
end
