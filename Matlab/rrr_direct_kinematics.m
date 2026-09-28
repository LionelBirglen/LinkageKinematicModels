function sol = rrr_direct_kinematics(geo, theta)
% RRR_DIRECT_KINEMATICS - Direct kinematics of a planar RRR serial manipulator
%
% INPUTS:
%   geo   - geometry struct or vector:
%           Struct fields: .L1, .L2, .L3  (link lengths, consistent units)
%           Vector form:   [L1, L2, L3]
%   theta - 1×3 vector [theta1, theta2, theta3] (rad)
%           theta1 : absolute angle of link 1 w.r.t. base frame
%           theta2 : relative angle between link 1 and link 2
%           theta3 : relative angle between link 2 and link 3
%
% OUTPUT:
%   sol - scalar struct with fields:
%     .Positions.O  - [x;y] base pivot (always [0;0])
%     .Positions.A  - [x;y] end of link 1 / joint 1
%     .Positions.B  - [x;y] end of link 2 / joint 2
%     .Positions.P  - [x;y] end-effector (end of link 3)
%     .Twists.xiO/A/B - zero-pitch twist coordinates [1; E*rOQ]
%     .phi          - absolute end-effector orientation (rad) = theta1+theta2+theta3
%     .valid        - always true for direct kinematics
%
% USAGE EXAMPLE:
%   geo = struct('L1',57,'L2',46,'L3',51);
%   sol = rrr_direct_kinematics(geo, deg2rad([39 37 40]))
%     sol.Positions.P =
%         33.0688
%        126.3434
%     sol.phi = 2.0071   (rad)
%
% BY:
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025

[L1, L2, L3] = rrr_parse_geo(geo);

theta1 = theta(1);
theta2 = theta(2);
theta3 = theta(3);

A1 = theta1;
A2 = theta1 + theta2;
A3 = A2    + theta3;

O  = [0; 0];
A  = O  + L1 * [cos(A1); sin(A1)];
B  = A  + L2 * [cos(A2); sin(A2)];
P  = B  + L3 * [cos(A3); sin(A3)];

E = [0 -1; 1 0];

sol.Positions.O = O;
sol.Positions.A = A;
sol.Positions.B = B;
sol.Positions.P = P;
sol.Twists.xiO  = [1; 0; 0];
sol.Twists.xiA  = [1; E*A];
sol.Twists.xiB  = [1; E*B];
sol.phi         = atan2(sin(A3), cos(A3));  % wrapped to [-pi, pi]
sol.valid       = true;
end

function [L1, L2, L3] = rrr_parse_geo(geo)
if isnumeric(geo)
    g = geo(:).'; L1=g(1); L2=g(2); L3=g(3);
elseif isstruct(geo)
    L1=geo.L1; L2=geo.L2; L3=geo.L3;
else
    error('rrr_direct_kinematics:BadGeo', ...
        'geo must be a numeric vector [L1 L2 L3] or a struct with fields L1,L2,L3.');
end
end
