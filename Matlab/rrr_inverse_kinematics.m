function sol = rrr_inverse_kinematics(geo, target)
% RRR_INVERSE_KINEMATICS - Inverse kinematics of a planar RRR serial manipulator
%
% INPUTS:
%   geo    - geometry struct or vector:
%            Struct fields: .L1, .L2, .L3  (link lengths, consistent units)
%            Vector form:   [L1, L2, L3]
%   target - 1×3 vector [Px, Py, phi]
%            Px, Py : desired end-effector position
%            phi    : desired end-effector orientation (rad)
%
% OUTPUT:
%   sol - 1×2 struct array:
%         sol(1) = elbow-up   configuration (elbow_config = +1)
%         sol(2) = elbow-down configuration (elbow_config = -1)
%   Each entry has fields:
%     .Positions.O  - [x;y] base pivot (always [0;0])
%     .Positions.A  - [x;y] end of link 1 / joint 1
%     .Positions.B  - [x;y] end of link 2 / joint 2
%     .Positions.P  - [x;y] end-effector = [Px;Py]
%     .Twists.xiO/A/B - zero-pitch twist coordinates [1; E*rOQ]
%     .theta        - [theta1; theta2; theta3] joint angles (rad)
%     .phi          - end-effector orientation (rad)
%     .valid        - true if target is reachable, false otherwise
%
% USAGE EXAMPLE:
%   geo = struct('L1',57,'L2',46,'L3',51);
%   sol = rrr_inverse_kinematics(geo, [35, 125, deg2rad(116)])
%   sol(1)  % elbow-up   solution
%   sol(2)  % elbow-down solution
%
% BY:
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025

[L1, L2, L3] = rrr_parse_geo(geo);

Px  = target(1);
Py  = target(2);
phi = target(3);

Wx = Px - L3 * cos(phi);
Wy = Py - L3 * sin(phi);
R2 = Wx^2 + Wy^2;
cos_theta2 = (R2 - L1^2 - L2^2) / (2 * L1 * L2);

E = [0 -1; 1 0];

nan2 = [NaN; NaN];
nan3 = [NaN; NaN; NaN];

% Preallocate both solutions (1=elbow-up, 2=elbow-down)
sol = repmat(struct( ...
    'Positions', struct('O',[0;0],'A',nan2,'B',nan2,'P',[Px;Py]), ...
    'Twists',    struct('xiO',[1;0;0],'xiA',nan3,'xiB',nan3), ...
    'theta',     nan2, ...
    'phi',       phi, ...
    'valid',     false), 1, 2);

if abs(cos_theta2) > 1
    return;
end

for k = 1:2
    elbow_config = 3 - 2*k;   % k=1 → +1 (elbow-up), k=2 → -1 (elbow-down)

    theta2 = atan2(elbow_config * sqrt(1 - cos_theta2^2), cos_theta2);
    k1     = L1 + L2 * cos(theta2);
    k2     = L2 * sin(theta2);
    theta1 = atan2(Wy, Wx) - atan2(k2, k1);
    theta3 = phi - (theta1 + theta2);
    % Wrap all angles to [-pi, pi]
    theta1 = atan2(sin(theta1), cos(theta1));
    theta2 = atan2(sin(theta2), cos(theta2));
    theta3 = atan2(sin(theta3), cos(theta3));

    A1 = theta1;
    A2 = theta1 + theta2;
    A  = [L1*cos(A1); L1*sin(A1)];
    B  = A + [L2*cos(A2); L2*sin(A2)];

    sol(k).Positions.A = A;
    sol(k).Positions.B = B;
    sol(k).Twists.xiO  = [1; 0; 0];
    sol(k).Twists.xiA  = [1; E*A];
    sol(k).Twists.xiB  = [1; E*B];
    sol(k).theta       = [theta1; theta2; theta3];
    sol(k).valid       = true;
end
end

function [L1, L2, L3] = rrr_parse_geo(geo)
if isnumeric(geo)
    g = geo(:).'; L1=g(1); L2=g(2); L3=g(3);
elseif isstruct(geo)
    L1=geo.L1; L2=geo.L2; L3=geo.L3;
else
    error('rrr_inverse_kinematics:BadGeo', ...
        'geo must be a numeric vector [L1 L2 L3] or a struct with fields L1,L2,L3.');
end
end
