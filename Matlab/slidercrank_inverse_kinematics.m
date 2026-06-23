function sol = slidercrank_inverse_kinematics(geo, x_slider, config)
% SLIDERCRANK_INVERSE_KINEMATICS - Inverse kinematics of a planar slider-crank mechanism
%
% INPUTS:
%   geo      - geometry struct or vector:
%              Struct fields: .a (crank), .b (coupler), .c (extension B->P),
%                             .slider_angle (prismatic axis angle, rad)
%              Vector form:   [a, b, c, slider_angle]
%   x_slider - desired slider displacement of B along its axis
%   config   - +1 elbow-down, -1 elbow-up
%
% OUTPUT:
%   sol - struct with fields:
%     .Positions.O   - [x;y] fixed ground revolute (always [0;0])
%     .Positions.A   - [x;y] crank-coupler revolute (end of crank)
%     .Positions.B   - [x;y] coupler-slider pin (on rail)
%     .Positions.P   - [x;y] end of extension link B->P
%     .phi           - crank angle (rad)
%     .x_slider      - slider displacement (same as input)
%     .slider_dir    - [cos;sin] unit vector along slider axis
%     .valid         - true if solution exists
%
% USAGE EXAMPLE:
%   geo = struct('a',70,'b',100,'c',30,'slider_angle',0);
%   sol = slidercrank_inverse_kinematics(geo, 136, 1)
%     sol.phi = 0.7854  (rad, approx deg2rad(45))
%     sol.valid = true
%
% BY:
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Contact: lionel.birglen@polymtl.ca
% Code provided under GNU Affero General Public License v3.0

[a, b, c, slider_angle] = sc_parse_geo(geo);

O          = [0; 0];
slider_dir = [cos(slider_angle); sin(slider_angle)];
B          = x_slider * slider_dir;

sol.Positions.O  = O;
sol.Positions.A  = [NaN; NaN];
sol.Positions.B  = B;
sol.Positions.P  = B + c * slider_dir;
sol.phi          = NaN;
sol.x_slider     = x_slider;
sol.slider_dir   = slider_dir;
sol.valid        = false;

[A1, A2, valid] = sc_circle_intersections(O, a, B, b);
if ~valid, return; end

A   = sc_ternary(config == 1, A1, A2);
phi = atan2(A(2) - O(2), A(1) - O(1));
phi = atan2(sin(phi), cos(phi));

sol.Positions.A = A;
sol.Positions.B = B;
sol.Positions.P = B + c * slider_dir;
sol.phi         = phi;
sol.valid       = true;
end

function [a, b, c, slider_angle] = sc_parse_geo(geo)
if isnumeric(geo)
    if numel(geo) < 4
        error('slidercrank_inverse_kinematics:BadGeo','geo vector must have 4 elements [a b c slider_angle].');
    end
    g = geo(:).';
    a = g(1); b = g(2); c = g(3); slider_angle = g(4);
elseif isstruct(geo)
    a = geo.a; b = geo.b; c = geo.c; slider_angle = geo.slider_angle;
else
    error('slidercrank_inverse_kinematics:BadGeo','geo must be a numeric vector [a b c slider_angle] or a struct.');
end
end

function [P1, P2, valid] = sc_circle_intersections(c1, r1, c2, r2)
d = norm(c2 - c1);
if d > (r1 + r2) || d < abs(r1 - r2)
    P1 = [0;0]; P2 = [0;0]; valid = false; return;
end
a_val = (r1^2 - r2^2 + d^2) / (2 * d);
h     = sqrt(max(0, r1^2 - a_val^2));
p2    = c1 + a_val * (c2 - c1) / d;
perp  = h * [0 -1; 1 0] * (c2 - c1) / d;
P1    = p2 + perp;
P2    = p2 - perp;
valid = true;
end

function out = sc_ternary(cond, a, b)
if cond, out = a; else, out = b; end
end
