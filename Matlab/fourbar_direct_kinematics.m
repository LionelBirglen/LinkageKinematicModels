function [phi, alpha, A, r_P] = fourbar_direct_kinematics(geo, theta, config)
% FOURBAR_DIRECT_KINEMATICS - Compute forward kinematics of a planar four-bar linkage
%
% OBJECTIVE:
%   Determine the coupler link angle, output link angle, and coupler endpoint
%   position for a planar four-bar mechanism given link lengths, crank angle,
%   and configuration.
%
% INPUTS:
%     geo     = [a, b, c, d, e, epsilon] (all in consistent length units, epsilon in rad)
%               • a       = length of output link (O→A)
%               • b       = length of coupler link (B→A)
%               • c       = length of input crank (C→B)
%               • d       = length of fixed ground link (O→C)
%               • e       = distance along coupler from joint A toward B to point P
%                            (i.e. |A–P| = e)
%               • epsilon = angle between coupler’s centerline (B→A) and line A→P (rad).
%     theta   = input‐crank angle (joint C→B) relative to ground (rad)
%     config  = +1 for “open” configuration, –1 for “crossed” configuration
%
% OUTPUTS:
%     phi     = coupler link absolute angle (A→B) w.r.t. ground (rad)
%     alpha   = output‐link angle (O→A) w.r.t. ground (rad)
%     A       = [x_A, y_A] coordinates of coupler endpoint A
%     r_P     = [x_P, y_P] coordinates of point P on coupler
%
% USAGE EXAMPLE:
%   geo = [81, 88, 92, 151, 10, pi/6];theta = deg2rad(106);config = -1;[phi, alpha, A, r_P] = fourbar_direct_kinematics(geo, theta, config)
%        phi =
%           0.2023
%        alpha =
%            1.0623
%        A =
%           39.4367   70.7513
%        r_P =
%           46.9154   77.3897
%   
% BY: 
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Last Update: 2025/06/03
% Contact: lionel.birglen@polymtl.ca
% 
% Code provided under GNU Affero General Public License v3.0


  %— Unpack geometry vector —
  a       = geo(1);   % output link O→A
  b       = geo(2);   % coupler link B→A
  c       = geo(3);   % input crank C→B
  d       = geo(4);   % ground link O→C
  e       = geo(5);   % distance A→P along coupler
  epsilon = geo(6);   % angle between coupler (B→A) and A→P (rad)

  %— Define fixed pivots in world frame —
  O = [0, 0];         % ground pivot at O
  C = [d, 0];         % ground pivot at C = (d,0)

  %— Compute position of B (input crank end) —
  B = C + c * [cos(theta), sin(theta)];

  %— Distance BO and feasibility check —
  R = norm(O - B);
  if R > (a + b) || R < abs(a - b)
    error('Four-bar cannot close: invalid geometry or theta.');
  end

  %— Angle from B→O —
  angle_BO = atan2(O(2) - B(2), O(1) - B(1));

  %— Use law of cosines at B to get coupler angle offset epsilon_B —
  cos_arg = (b^2 + R^2 - a^2) / (2*b*R);
  cos_arg = min(max(cos_arg, -1), +1);  % clamp in [-1,1]
  epsilon_B = acos(cos_arg);

 %— Compute φ_old = angle of vector B→A w.r.t. ground —
  phi_old = angle_BO + config * epsilon_B;
  phi_old = atan2(sin(phi_old), cos(phi_old));  % normalize

  %— Now set φ = angle of vector A→B = φ_old + π —
  phi = phi_old + pi;
  phi = atan2(sin(phi), cos(phi));  % normalize to [−π, π]

  %— Compute position of A using φ_old (coupler length b from B→A) —
  A = B + b * [cos(phi_old), sin(phi_old)];

  %— Output‐link angle α (O→A) —
  alpha = atan2(A(2), A(1));
  alpha = atan2(sin(alpha), cos(alpha));  % normalize

  %— Compute P on the coupler using φ (A→B direction) —
  r_P = A + e * [cos(phi + epsilon), sin(phi + epsilon)];
end