function [phi, alpha, A] = fourbar_direct_kinematics(c, b, a, d, theta, config)
% FOURBAR_DIRECT_KINEMATICS - Compute forward kinematics of a planar four-bar linkage
%
% OBJECTIVE:
%   Determine the coupler link angle, output link angle, and coupler endpoint
%   position for a planar four-bar mechanism given link lengths, crank angle,
%   and configuration.
%
% INPUTS:
%   c      - length of input crank (link c)
%   b      - length of coupler link (link b)
%   a      - length of output link (link a)
%   d      - length of fixed ground link between joints O and C
%   theta  - input crank angle at joint C relative to ground (rad)
%   config - +1 for open configuration, -1 for crossed configuration
%
% OUTPUTS:
%   phi   - absolute coupler link angle relative to ground (rad)
%   alpha - output link angle at joint O relative to ground (rad)
%   A     - [x, y] coordinates of coupler end point (joint A)
%
% USAGE EXAMPLE:
%   [phi, alpha, A] = fourbar_direct_kinematics(92, 88, 81, 151, deg2rad(106), 1)
%        phi =
%           -2.1173
%        alpha =
%            0.1644
%        A =
%           79.9084   13.2530
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

% Position of crank end (joint B)
B = C + c * [cos(theta), sin(theta)];

% Compute distance between B and O
R = norm(O - B);

% Check if linkage can close: triangle inequality
if R > (a + b) || R < abs(a - b)
    error('Linkage cannot close — invalid geometry.');
end

% Angle from B to O
angle_BO = atan2(O(2) - B(2), O(1) - B(1));

% Law of cosines to compute interior angle at joint B
epsilon = acos((b^2 + R^2 - a^2) / (2 * b * R));

% Coupler link angle phi based on configuration
phi = angle_BO + config * epsilon;

% Compute coupler endpoint A position
A = B + b * [cos(phi), sin(phi)];

% Output link angle alpha (angle O->A)
alpha = atan2(A(2), A(1));

% Normalize angles to [-pi, pi]
phi   = atan2(sin(phi), cos(phi));
alpha = atan2(sin(alpha), cos(alpha));
end