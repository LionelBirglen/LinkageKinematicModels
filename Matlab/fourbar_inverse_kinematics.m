function sol = fourbar_inverse_kinematics(geo, alpha)
% FOURBAR_INVERSE_KINEMATICS - Compute inverse kinematics of a planar four-bar linkage
%
% OBJECTIVE:
%   Compute the input crank angle, coupler angle, and coupler endpoint position
%   for a planar four-bar mechanism given desired output angle, link lengths,
%   and configuration.
%
% INPUTS:
%     geo     = [a, b, c, d, e, epsilon, delta]
%               (lengths in consistent units, epsilon and delta in rad)
%               • a       = length of output link (O→A)
%               • b       = length of coupler link (B→A)
%               • c       = length of input crank (C→B)
%               • d       = length of fixed ground link (O→C)
%               • e       = distance A→P along coupler
%               • epsilon = angle between coupler (A→B) and A→P (rad)
%               • delta   = angle locating ground point C such that
%                           C = [d*cos(delta); d*sin(delta)] (rad)
%
%     alpha  = desired output‐link angle (O→A) w.r.t. horizontal (rad)
%     config = +1 for one assembly mode, −1 for the other (open/crossed)
%
% OUTPUTS:
%     theta  = input‐crank angle (C→B) w.r.t. horizontal (rad)
%     phi    = coupler angle (A→B) w.r.t. horizontal (rad)
%     A      = [x_A, y_A] coordinates of joint A
%     r_P    = [x_P, y_P] coordinates of point P on the coupler
%
%
% USAGE EXAMPLE:
%     geo = [81, 88, 92, 151, 10, pi/6, 0];alpha = deg2rad(10);config = +1;[theta, phi, A, r_P] = fourbar_inverse_kinematics(geo, alpha, config)
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


%--- Unpack geometry ---
a       = geo(1);   % output link O→A
b       = geo(2);   % coupler link B→A
c       = geo(3);   % input crank C→B
d       = geo(4);   % ground link O→C
e       = geo(5);   % distance A→P along coupler
epsilon = geo(6);   % angle between coupler (A→B) and A→P (rad)
delta   = geo(7);   % angle locating C: C = [d*cos(delta), d*sin(delta)]

%--- Ground pivots ---
O = [0, 0];                          %#ok<NASGU>  % kept for consistency if you use it elsewhere
C = [d*cos(delta), d*sin(delta)];    % NEW: C rotated by delta

%--- Desired position of A from alpha and a ---
A = a * [cos(alpha), sin(alpha)];

%--- Find B as intersection of circles centered at A and C ---
[B1, B2, validB] = circle_intersections(A, b, C, c);
if ~validB
    % No real solution: return NaNs
    sol(k).Positions.O    = O;
    sol(k).Positions.A    = A;
    sol(k).Positions.C    = C;
    sol(k).Positions.B    = [NaN NaN];
    sol(k).phi            = NaN;
    sol(k).P              = [NaN;NaN];
    sol(k).valid          = 0;
    sol(k).Twists.xiO     = [1;0;0];
    sol(k).Twists.xiA     = [1;E*rOA];
    sol(k).Twists.xiB     = [NaN;NaN];
    sol(k).Twists.xiC     = [1;E*rOC];
    sol(k).theta          = NaN;
    sol(k).alpha          = alpha;
    return;
end

% E matrix for planar cross products
E=[0 -1; 1 0];

% Planar twists and wrenches are defined by:
% xi=[wz;vx;vy];
% zeta=[fx;fy;tz];

% Definition of twist vectors
rOA=A';
rOC=C';

for k=1:2
    if k==1;B = B1;else B = B2; end;

    rOB=B';

    %--- Coupler angle phi (A→B) ---
    phi = atan2(B(2) - A(2), B(1) - A(1));
    
    %--- Input crank angle theta (C→B) ---
    theta = atan2(B(2) - C(2), B(1) - C(1));
    
    %--- Point P on the coupler, at distance e from A along (phi + epsilon) ---
    r_P = A + e * [cos(phi + epsilon), sin(phi + epsilon)];

    % Fill structure
    sol(k).Positions.O    = O;
    sol(k).Positions.A    = A;
    sol(k).Positions.C    = C;
    sol(k).Positions.B    = B;
    sol(k).phi            = phi;
    sol(k).P              = r_P';
    sol(k).valid          = 1;
    sol(k).Twists.xiO     = [1;0;0];
    sol(k).Twists.xiA     = [1;E*rOA];
    sol(k).Twists.xiB     = [1;E*rOB];
    sol(k).Twists.xiC     = [1;E*rOC];
    sol(k).theta          = theta;
    sol(k).alpha          = alpha;   
end

end

%=========================%
% Circle intersection sub %
%=========================%
function [P1, P2, valid] = circle_intersections(c1, r1, c2, r2)
% CIRCLE_INTERSECTIONS  Intersection points of two circles
%
%   [P1, P2, valid] = circle_intersections(c1, r1, c2, r2)
%
%   Inputs:
%     c1, r1  = center & radius of circle 1  (c1 is 1×2 row vector)
%     c2, r2  = center & radius of circle 2  (c2 is 1×2 row vector)
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