function [theta, phi, A, r_P] = fourbar_inverse_kinematics(geo, alpha, config)
% FOURBAR_INVERSE_KINEMATICS - Compute inverse kinematics of a planar four-bar linkage
%
% OBJECTIVE:
%   Compute the input crank angle, coupler angle, and coupler endpoint position
%   for a planar four-bar mechanism given desired output angle, link lengths,
%   and configuration.
%
% INPUTS:
%     geo     = [a, b, c, d, e, epsilon] (lengths in consistent units, epsilon in rad)
%               • a       = length of output link (O→A)
%               • b       = length of coupler link (B→A)
%               • c       = length of input crank (C→B)
%               • d       = length of fixed ground link (O→C)
%               • e       = distance A→P along coupler
%               • epsilon = angle between coupler and A→P (rad)
%     alpha   = desired output link angle (joint O→A) w.r.t. ground (rad)
%     config  = +1 for “open” configuration, –1 for “crossed” configuration
%
% OUTPUTS:
%     theta   = input‐crank angle (C→B) w.r.t. ground (rad)
%     phi     = coupler link angle (A→B) w.r.t. ground (rad)
%     A       = [x_A, y_A] coordinates of coupler endpoint A
%     r_P     = [x_P, y_P] coordinates of point P on coupler
%
% USAGE EXAMPLE:
%     geo = [81, 88, 92, 151, 10, pi/6];alpha = deg2rad(10);config = +1;[theta, phi, A, r_P] = fourbar_inverse_kinematics(geo, alpha, config)
%        theta =
%           1.8409
%        phi =
%            1.0117
%        A =
%           79.7694   14.0655
%        r_P =
%           80.1248   24.0592
%
% BY: 
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Last Update: 2025/05/15
% Contact: lionel.birglen@polymtl.ca
% 
% Code provided under GNU Affero General Public License v3.0


  %— Unpack geometry vector —
  a       = geo(1);  % output link O→A
  b       = geo(2);  % coupler link B→A
  c       = geo(3);  % input crank C→B
  d       = geo(4);  % ground link O→C
  e       = geo(5);  % distance A→P
  epsilon = geo(6);  % angle between coupler and A→P

  %— Define fixed pivots —
  O = [0, 0];
  C = [d, 0];

  %— Compute position of A from desired alpha —
  A = a * [cos(alpha), sin(alpha)];

  %— Find circle intersections:  
  %   Circle1: center=A, radius=b  (coupler B→A)  
  %   Circle2: center=C, radius=c  (input crank C→B)
  [B1, B2, valid] = circle_intersections(A, b, C, c);
  if ~valid
    error('Unreachable: output‐link angle α yields no valid input‐crank B.');
  end

  %— Pick the correct intersection based on config —
  if config == +1
    B = B1;   % open
  else
    B = B2;   % crossed
  end
  vec_BA = A - B;
  phi_old = atan2(vec_BA(2), vec_BA(1));        % angle of B→A
  phi_old = atan2(sin(phi_old), cos(phi_old));  % normalize

  %— Now set φ = angle of A→B = φ_old + π —
  phi = phi_old + pi;
  phi = atan2(sin(phi), cos(phi));              % normalize

  %— Compute input‐crank angle theta from C→B —
  vec_CB = B - C;
  theta = atan2(vec_CB(2), vec_CB(1));
  theta = atan2(sin(theta), cos(theta));        % normalize

  %— Compute P on the coupler using φ (A→B direction) —
  r_P = A + e * [cos(phi + epsilon), sin(phi + epsilon)];
end

%──────────────
function [P1, P2, valid] = circle_intersections(c1, r1, c2, r2)
% CIRCLE_INTERSECTIONS  Intersection points of two circles
%
%   [P1, P2, valid] = circle_intersections(c1, r1, c2, r2)
%
%   Inputs:
%     c1, r1  = center & radius of circle 1
%     c2, r2  = center & radius of circle 2
%
%   Outputs:
%     P1, P2  = two intersection points (1×2 row vectors). If tangent, P1==P2.
%     valid   = true if intersection exists, false otherwise.

  d = norm(c2 - c1);
  if d > (r1 + r2) || d < abs(r1 - r2)
    P1 = [];  P2 = [];  valid = false;
    return;
  end

  a = (r1^2 - r2^2 + d^2) / (2 * d);
  h = sqrt(max(r1^2 - a^2, 0));

  p2 = c1 + a * (c2 - c1) / d;
  offset = h * [0 -1; 1 0] * ((c2 - c1)' / d);

  P1 = p2 + offset';
  P2 = p2 - offset';
  valid = true;
end
