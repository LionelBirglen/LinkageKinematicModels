function [theta, phi, A] = fourbar_inverse_kinematics(c, b, a, d, alpha, config)
% FOURBAR_INVERSE_KINEMATICS - Compute inverse kinematics of a planar four-bar linkage
%
% OBJECTIVE:
%   Compute the input crank angle, coupler angle, and coupler endpoint position
%   for a planar four-bar mechanism given desired output angle, link lengths,
%   and configuration.
%
% INPUTS:
%   c      - length of input crank (link c)
%   b      - length of coupler link (link b)
%   a      - length of output link (link a)
%   d      - length of fixed ground link between joints O and C
%   alpha  - desired output link angle at joint O relative to ground (rad)
%   config - +1 for open configuration, -1 for crossed configuration
%
% OUTPUTS:
%   theta - crank angle at joint C relative to ground (rad)
%   phi   - coupler link angle relative to ground (rad)
%   A     - [x, y] coordinates of coupler end point (joint A)
%
% USAGE EXAMPLE:
%   [theta, phi, A] = fourbar_inverse_kinematics(92, 88, 81, 151, deg2rad(30), -1)
%        theta =
%           -2.5994
%        phi =
%            1.5941
%        A =
%           70.1481   40.5000
%
% BY: 
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Last Update: 2025/05/15
% Contact: lionel.birglen@polymtl.ca
% 
% Code provided under GNU Affero General Public License v3.0


% Fixed ground pivot positions
O = [0, 0];             % Ground pivot at O (origin)
C = [d, 0];             % Ground pivot at C

% Desired coupler endpoint position
A = a * [cos(alpha), sin(alpha)];

% Compute intersections of two circles: circle at A with radius b and circle at C with radius c
[B1, B2, valid] = circle_intersections(A, b, C, c);
if ~valid
    error('Linkage cannot close — invalid geometry.');
end

% Select intersection based on configuration
if config == 1
    B = B1;  % open configuration
else
    B = B2;  % crossed configuration
end

% Compute crank angle theta from C to B
vec_CB = B - C;
theta = atan2(vec_CB(2), vec_CB(1));

% Compute coupler angle phi from B to A
vec_AB = A - B;
phi = atan2(vec_AB(2), vec_AB(1));

% Normalize angles to [-pi, pi]
theta = atan2(sin(theta), cos(theta));
phi   = atan2(sin(phi), cos(phi));
end

function [P1, P2, valid] = circle_intersections(c1, r1, c2, r2)
% CIRCLE_INTERSECTIONS - Compute intersection points of two circles
%
% INPUTS:
%   c1, r1 - center and radius of first circle
%   c2, r2 - center and radius of second circle
%
% OUTPUTS:
%   P1, P2 - 1x2 vectors of intersection points
%   valid  - true if intersections exist, false otherwise
d = norm(c2 - c1);            % Distance between circle centers
if d > r1 + r2 || d < abs(r1 - r2)
    P1 = []; P2 = []; valid = false;
    return;
end

a = (r1^2 - r2^2 + d^2) / (2 * d);   % Distance from c1 to line of intersection
h = sqrt(r1^2 - a^2);                % Height from line to intersection points

p2 = c1 + a * (c2 - c1) / d;         % Point along line between centers

offset = h * [0 -1; 1 0] * (c2 - c1)' / d;
P1 = p2 + offset';                   % First intersection point
P2 = p2 - offset';                   % Second intersection point
valid = true;
end

function out = ternary(cond, a, b)
% TERNARY - Return one of two values based on a condition
%
% INPUTS:
%   cond - logical condition
%   a    - output if cond is true
%   b    - output if cond is false
%
% OUTPUT:
%   out  - a if cond, otherwise b
if cond
    out = a;
else
    out = b;
end
end
