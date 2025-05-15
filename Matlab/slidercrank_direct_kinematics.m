function [x_slider, B, P] = slidercrank_direct_kinematics(a, b, phi, slider_angle, config)
% SLIDERCRANK_DIRECT_KINEMATICS - Compute forward kinematics of a planar slider-crank mechanism
%
% OBJECTIVE:
%   Determine the slider displacement, crank-coupler joint location, and
%   coupler-slider joint location for a slider-crank with an adjustable
%   prismatic joint orientation.
%
% INPUTS:
%   a            - length of crank link (units, e.g., mm)
%   b            - length of coupler link (units, e.g., mm)
%   phi          - crank angle relative to base frame (rad)
%   slider_angle - orientation of prismatic joint axis relative to base (rad)
%   config       - +1 for elbow-down configuration, -1 for elbow-up (optional, default +1)
%
% OUTPUTS:
%   x_slider - scalar displacement of slider along its axis
%   B        - 1x2 [x, y] position of crank-coupler joint
%   P        - 1x2 [x, y] position of coupler-slider joint
%
% USAGE EXAMPLE:
%   [x_slider, B, P] = slidercrank_direct_kinematics(70, 100, deg2rad(45), deg2rad(0), 1)
%        x_slider =
%          136.3882
%        B =
%           49.4975   49.4975
%        P =
%          136.3882         0
%
% BY: 
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Last Update: 2025/05/15
% Contact: lionel.birglen@polymtl.ca
% 
% Code provided under GNU Affero General Public License v3.0


% Default to elbow-down if configuration not provided
if nargin < 5
    config = +1;
end

% Ground pivot at origin
O = [0, 0];

% Crank tip (joint between crank and coupler)
B = a * [cos(phi), sin(phi)];

% Unit vector along slider axis
slider_dir    = [cos(slider_angle), sin(slider_angle)];
% Unit normal to slider axis
slider_normal = [-sin(slider_angle), cos(slider_angle)];

% Perpendicular distance from crank tip to slider line
perp_dist = dot(B - O, slider_normal);
% Preserve sign for correct foot location
perp_sign = sign(perp_dist);
perp_dist = abs(perp_dist);

% Check reachability: coupler length must cover perpendicular gap
if perp_dist > b
    error('Slider-crank cannot close - coupler too short for this configuration.');
end

% Foot of perpendicular from crank tip onto slider line
perp_point = B - perp_sign * perp_dist * slider_normal;

% Along-axis distance from foot to coupler-slider joint
along_dist = sqrt(b^2 - perp_dist^2);

% Two potential slider positions along axis
P1 = perp_point + along_dist * slider_dir;
P2 = perp_point - along_dist * slider_dir;

% Select solution based on configuration
P = ternary(config == 1, P1, P2);

% Scalar slider displacement along its axis
x_slider = dot(P - O, slider_dir);
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