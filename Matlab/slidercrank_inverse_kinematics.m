function [phi, B] = slidercrank_inverse_kinematics(a, b, x_slider, config, slider_angle)
% SLIDERCRANK_INVERSE_KINEMATICS - Compute inverse kinematics of a planar slider-crank mechanism
%
% OBJECTIVE:
%   Determine the crank angle and crank-coupler joint coordinates needed
%   to achieve a specified slider displacement along an adjustable
%   prismatic axis in a slider-crank mechanism.
%
% INPUTS:
%   a            - length of crank link (units, e.g., mm)
%   b            - length of coupler link (units, e.g., mm)
%   x_slider     - desired scalar displacement of slider along its axis
%   config       - +1 for elbow-down configuration, -1 for elbow-up
%   slider_angle - orientation of prismatic joint axis relative to base (rad)
%
% OUTPUTS:
%   phi - computed crank angle relative to base frame (rad)
%   B   - 1x2 [x, y] position of crank-coupler joint
%
% USAGE EXAMPLE:
%   [phi, B] = slidercrank_inverse_kinematics(70, 100, 50, -1, deg2rad(30))
%        phi =
%           -1.4277
%        B =
%            9.9795  -69.2850
%
% BY: 
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Last Update: 2025/05/15
% Contact: lionel.birglen@polymtl.ca
% 
% Code provided under GNU Affero General Public License v3.0


% Ground pivot at origin
O = [0, 0];

% Unit vector along slider axis
slider_vec = [cos(slider_angle), sin(slider_angle)];

% Slider joint coordinate in base frame
P = x_slider * slider_vec;

% Compute intersections of circles:
%   Circle1: centered at O, radius = a (crank reach)
%   Circle2: centered at P, radius = b (coupler reach)
[B1, B2, valid] = circle_intersections(O, a, P, b);
if ~valid
    error('Slider-crank cannot close — invalid geometry.');
end

% Select crank-coupler joint position based on config
B = ternary(config == 1, B1, B2);

% Compute crank angle from origin to joint B
vec_OB = B - O;
phi    = atan2(vec_OB(2), vec_OB(1));
% Normalize angle to [-pi, pi]
phi    = atan2(sin(phi), cos(phi));
end

% Helper: intersect two circles
function [P1, P2, valid] = circle_intersections(c1, r1, c2, r2)
% CIRCLE_INTERSECTIONS - Compute intersection points of two circles
%
% INPUTS:
%   c1, r1 - center and radius of first circle
%   c2, r2 - center and radius of second circle
%
% OUTPUTS:
%   P1, P2 - 1x2 coordinates of intersection points
%   valid  - true if intersection exists, false otherwise

d = norm(c2 - c1);  % distance between centers
% Check for valid intersection
if d > (r1 + r2) || d < abs(r1 - r2)
    P1 = []; P2 = []; valid = false;
    return;
end

% Distance from c1 to chord along center line
a = (r1^2 - r2^2 + d^2) / (2 * d);
% Height from chord to intersection points
h = sqrt(r1^2 - a^2);

% Base point on line between centers
p2 = c1 + a * (c2 - c1) / d;

% Offset vector perpendicular to line for intersections
offset = h * [0 -1; 1 0] * (c2 - c1)' / d;
P1 = p2 + offset';
P2 = p2 - offset';
valid = true;
end

% Helper: ternary operator
function out = ternary(cond, aVar, bVar)
% TERNARY - Return one of two values based on condition
%   cond ? aVar : bVar
if cond
    out = aVar;
else
    out = bVar;
end
end
