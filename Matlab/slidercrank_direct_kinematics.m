function sol = slidercrank_direct_kinematics(geo, phi, config)
% SLIDERCRANK_DIRECT_KINEMATICS - Direct kinematics of a planar slider-crank mechanism
%
% INPUTS:
%   geo    - geometry struct or vector:
%            Struct fields: .a (crank), .b (coupler), .c (extension B->P),
%                           .slider_angle (prismatic axis angle, rad)
%            Vector form:   [a, b, c, slider_angle]
%   phi    - crank angle relative to base frame (rad)
%   config - +1 elbow-down, -1 elbow-up (default +1)
%
% OUTPUT:
%   sol - struct with fields:
%     .Positions.O   - [x;y] fixed ground revolute (always [0;0])
%     .Positions.A   - [x;y] crank-coupler revolute (end of crank)
%     .Positions.B   - [x;y] coupler-slider pin (moves along rail)
%     .Positions.P   - [x;y] end of extension link B->P
%     .x_slider      - scalar displacement of B along slider axis
%     .phi           - crank angle (rad), same as input
%     .slider_dir    - [cos;sin] unit vector along slider axis
%     .valid         - true if configuration closes
%
% USAGE EXAMPLE:
%   geo = struct('a',70,'b',100,'c',30,'slider_angle',0);
%   sol = slidercrank_direct_kinematics(geo, deg2rad(45), 1)
%     sol.x_slider = 136.3882
%     sol.Positions.A = [49.4975; 49.4975]
%     sol.Positions.B = [136.3882; 0]
%     sol.Positions.P = [166.3882; 0]
%
% BY:
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Contact: lionel.birglen@polymtl.ca
% Code provided under GNU Affero General Public License v3.0

if nargin < 3, config = +1; end
[a, b, c, slider_angle] = sc_parse_geo(geo);

O = [0; 0];
A = a * [cos(phi); sin(phi)];

slider_dir    = [cos(slider_angle); sin(slider_angle)];
slider_normal = [-sin(slider_angle); cos(slider_angle)];

perp_dist     = dot(A - O, slider_normal);
perp_sign     = sign(perp_dist);
perp_dist_abs = abs(perp_dist);

sol.Positions.O  = O;
sol.Positions.A  = [NaN; NaN];
sol.Positions.B  = [NaN; NaN];
sol.Positions.P  = [NaN; NaN];
sol.x_slider     = NaN;
sol.phi          = phi;
sol.slider_dir   = slider_dir;
sol.valid        = false;

if perp_dist_abs > b, return; end

perp_point = A - perp_sign * perp_dist_abs * slider_normal;
along_dist = sqrt(b^2 - perp_dist_abs^2);

B1 = perp_point + along_dist * slider_dir;
B2 = perp_point - along_dist * slider_dir;
B  = sc_ternary(config == 1, B1, B2);
P  = B + c * slider_dir;

sol.Positions.A = A;
sol.Positions.B = B;
sol.Positions.P = P;
sol.x_slider    = dot(B - O, slider_dir);
sol.valid       = true;
end

function [a, b, c, slider_angle] = sc_parse_geo(geo)
if isnumeric(geo)
    if numel(geo) < 4
        error('slidercrank_direct_kinematics:BadGeo','geo vector must have 4 elements [a b c slider_angle].');
    end
    g = geo(:).';
    a = g(1); b = g(2); c = g(3); slider_angle = g(4);
elseif isstruct(geo)
    a = geo.a; b = geo.b; c = geo.c; slider_angle = geo.slider_angle;
else
    error('slidercrank_direct_kinematics:BadGeo','geo must be a numeric vector [a b c slider_angle] or a struct.');
end
end

function out = sc_ternary(cond, a, b)
if cond, out = a; else, out = b; end
end
