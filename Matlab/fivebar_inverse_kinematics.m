function invSol = fivebar_inverse_kinematics(geo, P_des)
%FIVEBAR_INVERSE_KINEMATICS  Solve inverse kinematics of a five‐bar linkage.
%   This version finds point A by intersecting:
%     • circle centered at O = [0;0] with radius a
%     • circle centered at P_des with radius h
%   Then, for each A, computes φ from the direction A→P, constructs B, and
%   finds C by intersecting:
%     • circle centered at D = [e*cosd(alpha); e*sind(alpha)] with radius d
%     • circle centered at B with radius c
%
%   invSol = FIVEBAR_INVERSE_KINEMATICS(geo, P_des) returns up to four
%   possible solutions in a struct array invSol with fields:
%     .theta     = [θ1; θ2] (in degrees)
%     .phi       = φ       (in degrees)
%     .Positions = substructure with fields .O, .D, .A, .B, .C
%     .P         = P_des
%     .valid     = 1 (for real‐valued solutions)
%
%   INPUT:
%     geo   – 1×8 vector: [a, b, c, d, e, alpha, h, eta]
%             a     = length of left input crank (O→A)
%             b     = length of left coupler (A→B) (output link)
%             c     = length of right coupler (C→B)
%             d     = length of right input crank (D→C)
%             e     = distance between ground pivots O and D
%             alpha = angle (deg) locating D from the +x axis
%             h     = radial distance of the point P from
%                     A along A→B
%             eta   = angle (in degrees) of P measured from the direction A→B
%
%     P_des – 2×1 vector [Px; Py] (target coordinates of point P)
%
%   OUTPUT:
%     invSol – 1×4 struct array (with up to 4 entries). Each entry has fields:
%              .theta     = [θ1; θ2] (in degrees)
%              .Positions = substructure containing:
%                           .O = [0;0]
%                           .D = [e*cosd(alpha); e*sind(alpha)]
%                           .A = [Ax; Ay]
%                           .B = [Bx; By]
%                           .C = [Cx; Cy]
%              .Twists.Q  = [1; E*rOQ]  coordinates of the zero-pitch twist at point Q=O,A,B,C,D
%              .phi       = φ       (orientation of A→B, in degrees)
%              .P         = [Px; Py]
%              .valid     = 1 if real solution(s) exist, else 0
%
%   NOTES:
%     • Angles are in degrees; distances in consistent units.
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
%   pos = [0.5298, 0.2439]; % desired position for P
%   sol = fivebar_inverse_kinematics(geo, pos);
%   % sol(1) to sol(4) contain the four assembly modes (if valid = 1).

    %-----------------------------
    % Unpack parameters
    %-----------------------------
    a     = geo(1);
    b     = geo(2);
    c     = geo(3);
    d     = geo(4);
    e     = geo(5);
    alpha = geo(6);  % in degrees
    h     = geo(7);
    eta   = geo(8);  % in degrees

    Px = P_des(1);
    Py = P_des(2);

    % Ground pivots
    O = [0; 0];
    D = [e*cosd(alpha); e*sind(alpha)];

    % E matrix for planar cross products
    E=[0 -1; 1 0];

    % Planar twists and wrenches are defined by:
    % xi=[wz;vx;vy];
    % zeta=[fx;fy;tz];

    % Constant vector for twists
    rOD=D;

    % Preallocate empty struct array
    invSol = struct('theta', {}, 'phi', {}, 'Positions', {}, 'Twists', {}, 'P', {}, 'valid', {});

    % 1) Find intersection A between circle(O,a) and circle(P,h)
    [A1, A2, okA] = circleCircleIntersect(0, 0, a, Px, Py, h);
    if ~okA
        error('Five-bar cannot close: invalid geometry or inputs.');
        
        % No feasible A; return empty set
        return;
    end

    % For each A candidate
    A_candidates = [A1, A2];
    for iA = 1:2
        Ai = A_candidates(:, iA);

        % φ from A→P (orientation of A→B)
        phi = atan2d(Py - Ai(2), Px - Ai(1)) - eta;

        % Build B along direction φ at distance b from A
        Bx = Ai(1) + b*cosd(phi);
        By = Ai(2) + b*sind(phi);
        Bi = [Bx; By];

        % 2) Find intersection C between circle(D,d) and circle(B,c)
        [C1, C2, okC] = circleCircleIntersect(D(1), D(2), d, Bx, By, c);
        if ~okC
            % No feasible C; skip
            continue;
        end

        % Two possible C per A → up to 4 total solutions
        C_candidates = [C1, C2];
        for iC = 1:2
            Ci = C_candidates(:, iC);

            % Compute input angles θ1 (OA) and θ2 (DC)
            theta1 = atan2d(Ai(2) - O(2), Ai(1) - O(1));       % angle of OA
            theta2 = atan2d(Ci(2) - D(2), Ci(1) - D(1));       % angle of DC

            % Renaming for twists
            rOA=Ai;
            rOB=Bi;
            rOC=Ci;

            sol.theta     = [theta1; theta2];
            sol.phi       = phi;
            sol.Positions = struct('O', O, 'D', D, 'A', Ai, 'B', Bi, 'C', Ci);
            sol.Twists    = struct('xiO', [1;0;0], 'xiD', [1;E*rOD], 'xiA', [1;E*rOA], 'xiB', [1;E*rOB], 'xiC', [1;E*rOC]);
            sol.P         = [Px; Py];
            sol.valid     = 1;

            invSol(end+1) = sol; %#ok<AGROW>
        end
    end

    % If none found, return empty (invSol = [])
end

%=============================%
% Helpers                     %
%=============================%
function [Pout1, Pout2, valid] = circleCircleIntersect(x0, y0, r0, x1, y1, r1)
%CIRCLECIRCLEINTERSECT  Intersections of two circles.
%   [P1, P2, valid] = CIRCLECIRCLEINTERSECT(x0,y0,r0, x1,y1,r1)
%   returns the two intersection points P1, P2 (each 2×1) and a boolean
%   valid flag. If circles are tangent, valid = true and P1 = P2. If no
%   intersection exists, valid = false and P1,P2 are NaN.

    % Initialize
    Pout1 = [NaN; NaN];
    Pout2 = [NaN; NaN];
    valid = false;

    dx = x1 - x0;
    dy = y1 - y0;
    d = hypot(dx, dy);

    % Check for solvability
    if d > (r0 + r1) || d < abs(r0 - r1) || (d == 0 && abs(r0 - r1) < eps)
        return;
    end

    % Distance from first center to line of intersection along the center line
    a = (r0^2 - r1^2 + d^2) / (2*d);
    h = sqrt(max(r0^2 - a^2, 0));

    % Point P2 on the line between centers
    x2 = x0 + (dx * a / d);
    y2 = y0 + (dy * a / d);

    % Offsets for intersection points
    rx = -dy * (h / d);
    ry =  dx * (h / d);

    Pout1 = [x2 + rx; y2 + ry];
    Pout2 = [x2 - rx; y2 - ry];
    valid = true;
end
