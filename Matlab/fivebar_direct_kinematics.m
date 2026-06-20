function sol = fivebar_direct_kinematics(geo, theta)
%FIVEBAR_DIRECT_KINEMATICS  Direct kinematics of a planar five‐bar linkage
%
%   sol = FIVEBAR_DIRECT_KINEMATICS(geo, theta) computes the joint positions,
%   the orientation of the chosen output link, the coordinates of a point of
%   interest on that output link, and a validity flag, for each of the two
%   possible assembly modes of a five‐bar mechanism.
%
%   INPUTS:
%     geo   – 1×8 vector of geometry parameters (all lengths in same units):
%             geo(1) = a     : length of the left input crank (O→A)
%             geo(2) = b     : length of the left coupler (A→B)  ← treated as “output link”
%             geo(3) = c     : length of the right coupler (C→B)
%             geo(4) = d     : length of the right input crank (D→C)
%             geo(5) = e     : distance between the two ground pivots (O–D)
%             geo(6) = alpha : angle of vector OD relative to the x-axis (in degrees)
%             geo(7) = h     : radial distance of the point of interest on the output link (A→B)
%             geo(8) = eta   : angle from vector AB to vector AP (in degrees)
%
%     theta – 1×2 vector of the two input‐crank angles (in degrees):
%             theta(1) = θ1 : absolute angle of left input crank (O→A) measured from x-axis
%             theta(2) = θ2 : absolute angle of right input crank (D→C) measured from x-axis
%
%   OUTPUT:
%     sol – 1×2 struct array (one element per assembly mode). Each sol(k) has fields:
%            .Positions = substructure containing:
%                           .O = [0;0]                          (fixed coordinates of left ground pivot)
%                           .D = [e*cosd(alpha); e*sind(alpha)] (fixed coordinates of right ground pivot)
%                           .A = [Ax; Ay]                       (coordinates of left crank endpoint)
%                           .B = [Bx; By]                       (coordinates of floating joint)
%                           .C = [Cx; Cy]                       (coordinates of right crank endpoint)
%           .Twists.Q   = [1; E*rOQ]           (coordinates of the zero-pitch twist at point Q=O,A,B,C,D)
%           .phi        = φ                    (orientation of the “output link” A→B, in degrees)
%           .P          = [Px; Py]             (coordinates of the point of interest on link A→B)
%           .theta      = [θ1; θ2]             (inputs in degrees)
%           .valid      = 0 or 1               (1 if a real intersection exists, 0 otherwise)
%
%   NOTES:
%     • We treat the left coupler (A→B) as the “output link” whose orientation φ is returned (in degrees).
%     • The point P is located on the link A→B at distance h from A, at an angle η (in degrees)
%       measured from the direction A→B (i.e. φ + η in the global frame).
%     • If there are no solutions, both sol(1).valid and sol(2).valid are set to 0, and the
%       Positions fields are filled with NaN.
%
%   Example:
%   % Given a five‐bar with
%   a  = 0.4;     % left crank length
%   b  = 0.5;     % left coupler (output) length
%   c  = 0.6;     % right coupler length
%   d  = 0.3;     % right crank length
%   e  = 1.0;     % ground pivot spacing
%   alpha  = 0.0; % ground pivot spacing
%   h  = 0.25;    % point P at 0.25 units along A→B
%   eta = 30;     % degrees measured from link A→B
%   geo = [0.4, 0.5, 0.6, 0.3, 1.0, 0, 0.25, 30];
%   theta = [45, -30]; % angles in degrees
%   sol = fivebar_direct_kinematics(geo, theta);
%   % sol(1) and sol(2) contain the two assembly modes (if valid = 1).
%
%   BY:
%   Prof. Lionel Birglen
%  Polytechnique Montreal, 2025
%   Last Update: 2025/11/05
%   Contact: lionel.birglen@polymtl.ca

% Unpack geometry
a     = geo(1);  % left input crank (O→A)
b     = geo(2);  % left coupler = "output link" (A→B)
c     = geo(3);  % right coupler (C→B)
d     = geo(4);  % right input crank (D→C)
e     = geo(5);  % ground spacing between O and D
alpha = geo(6);  % radial distance along link A→B for point P
h     = geo(7);  % radial distance along link A→B for point P
eta   = geo(8);  % angle (in degrees) from A→B at which P is located

% Convert input angles from degrees to radians for computation
theta1 = deg2rad(theta(1));  % left crank angle (radians)
theta2 = deg2rad(theta(2));  % right crank angle (radians)
eta_rad = deg2rad(eta);      % point angle relative to link (radians)

% Define fixed ground‐pivot coordinates
O = [0;  0];                            % left ground pivot
alpha_rad=deg2rad(alpha);
Dx = e*cos(alpha_rad);
Dy = e*sin(alpha_rad);
D = [Dx;  Dy];                          % right ground pivot

% Compute left crank endpoint A
Ax = a * cos(theta1);
Ay = a * sin(theta1);
A  = [Ax; Ay];

% Compute right crank endpoint C
Cx = Dx + d * cos(theta2);
Cy = Dy + d * sin(theta2);
C  = [Cx; Cy];

% E matrix for planar cross products
E=[0 -1; 1 0];

% Planar twists and wrenches are defined by:
% xi=[wz;vx;vy];
% zeta=[fx;fy;tz];

% Definition of twist vectors
rOA=A;
rOC=C;
rOD=D;

% Preallocate solution structure array (2 assembly modes)
sol = repmat(struct( ...
    'Positions', struct('O', nan(2,1), 'A', nan(2,1), 'B', nan(2,1), 'C', nan(2,1), 'D', nan(2,1)), ...
    'Twists', struct('xiO', nan(3,1), 'xiD', nan(3,1), 'xiA', nan(3,1), 'xiC', nan(3,1), 'xiB', nan(3,1)), ...
    'phi',      nan,     ...   % in degrees
    'P',        nan(2,1), ...
    'valid',    0), 1, 2);

% Distance between A and C
d_AC = norm(C - A);

% Check feasibility of intersection of two circles:
%   circle1: center A, radius b
%   circle2: center C, radius c
if (d_AC > (b + c)) || (d_AC < abs(b - c)) || (d_AC == 0 && b == c)

    error('Five-bar cannot close: invalid geometry or inputs.');

    % No valid intersection → both solutions invalid
    for k = 1:2
        sol(k).Positions.O    = O;
        sol(k).Positions.A    = A;
        sol(k).Positions.B    = [NaN; NaN];
        sol(k).Positions.C    = C;
        sol(k).Positions.D    = D;
        sol(k).phi            = NaN;
        sol(k).P              = [NaN; NaN];
        sol(k).valid          = 0;
        sol(k).Twists.xiO   = [1;0;0];
        sol(k).Twists.xiA   = [1;E*rOA];
        sol(k).Twists.xiB   = NaN(3,1);
        sol(k).Twists.xiC   = [1;E*rOC];
        sol(k).Twists.xiD   = [1;E*rOD];
        sol(k).theta        = [rad2deg(theta1);rad2deg(theta2)];
    end
    return;
end

% Compute l = distance along AC from A to line of centers of intersection
l = (b^2 - c^2 + d_AC^2) / (2 * d_AC);

% Height from that point to each intersection
h_int = sqrt(max(b^2 - l^2, 0));

% Point P2: projection of B onto the line AC
%   P2 = A + l * (C - A) / d_AC
P2 = A + (l / d_AC) * (C - A);

% Unit‐perpendicular vector from AC
ux = -(Cy - Ay) / d_AC;
uy =  (Cx - Ax) / d_AC;

% Two intersection points (B1, B2)
B1 = P2 + h_int * [ux; uy];
B2 = P2 - h_int * [ux; uy];

% Assemble solutions
B_candidates = {B1, B2};
for k = 1:2
    Bk = B_candidates{k};
    % Compute orientation φ of “output link” (A→Bk)
    dx = Bk(1) - Ax;
    dy = Bk(2) - Ay;
    phi_rad = atan2(dy, dx);
    phi_deg = rad2deg(phi_rad);

    % Compute coordinates of the point of interest P
    Px = Ax + h * cos(phi_rad + eta_rad);
    Py = Ay + h * sin(phi_rad + eta_rad);
    Pk = [Px; Py];

    % Definition of twist vectors
    rOB=Bk;

    % Fill structure
    sol(k).Positions.O    = O;
    sol(k).Positions.D    = D;
    sol(k).Positions.A    = A;
    sol(k).Positions.C    = C;
    sol(k).Positions.B    = Bk;
    sol(k).phi            = phi_deg;
    sol(k).P              = Pk;
    sol(k).valid          = 1;
    sol(k).Twists.xiO     = [1;0;0];
    sol(k).Twists.xiA     = [1;E*rOA];
    sol(k).Twists.xiB     = [1;E*rOB];
    sol(k).Twists.xiC     = [1;E*rOC];
    sol(k).Twists.xiD     = [1;E*rOD];
    sol(k).theta          = [rad2deg(theta1);rad2deg(theta2)];
end
end
