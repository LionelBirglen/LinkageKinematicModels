function sol = fourbar_inverse_kinematics(geo, alpha)
% FOURBAR_INVERSE_KINEMATICS - Inverse kinematics of a planar four-bar linkage
%
% INPUTS:
%   geo   - geometry, as a 1×7 numeric vector [a b c d e epsilon delta]
%           OR a struct with fields .a .b .c .d .e .epsilon .delta
%           a       : length of output link (O→A)
%           b       : length of coupler link (B→A)
%           c       : length of input crank (C→B)
%           d       : length of fixed ground link (O→C)
%           e       : distance A→P along coupler
%           epsilon : angle between coupler (A→B) and A→P (rad)
%           delta   : angle locating ground point C (rad)
%           Optional fields for additional points of interest:
%           h_q     : distance B→Q along the input crank link B-C
%           eta_q   : angle C-B-Q, measured at vertex B from ray B→C (rad)
%           h_r     : distance O→R along the output link O-A
%           eta_r   : angle A-O-R, measured at vertex O from ray O→A (rad)
%   alpha - desired output link angle O→A (rad)
%
% OUTPUT:
%   sol - 1×2 struct array (one element per assembly mode). Each has fields:
%     .Positions.O  - [x;y] left ground pivot (always [0;0])
%     .Positions.A  - [x;y] coupler endpoint
%     .Positions.B  - [x;y] input crank endpoint / coupler pivot
%     .Positions.C  - [x;y] right ground pivot
%     .Positions.Q  - [x;y] point of interest on link B-C (NaN if h_q/eta_q not given)
%     .Positions.R  - [x;y] point of interest on link O-A (NaN if h_r/eta_r not given)
%     .Twists.Q     - = [1; E*rOQ] coordinates of the zero-pitch twist at point Q=O,A,B,C
%     .P            - [x;y] coupler point of interest
%     .phi          - coupler angle A→B (rad)
%     .alpha        - output link angle O→A (rad), same as input
%     .theta        - input crank angle C→B (rad)
%     .valid        - 1 if solution exists, 0 otherwise
%
% USAGE EXAMPLE:
%   % Vector form (backward compatible):
%   geo = [81, 88, 92, 151, 10, pi/6, 0];
%   sol = fourbar_inverse_kinematics(geo, deg2rad(10));
%   % Struct form:
%   geo = struct('a',81,'b',88,'c',92,'d',151,'e',10,'epsilon',pi/6,'delta',0);
%   sol = fourbar_inverse_kinematics(geo, deg2rad(10));
%   % Struct form with optional Q and R points:
%   geo.h_q = 30; geo.eta_q = pi/6; geo.h_r = 40; geo.eta_r = pi/4;
%   sol = fourbar_inverse_kinematics(geo, deg2rad(10));
%   % Access results:
%   sol(1).theta          % input crank angle, assembly mode 1
%   sol(1).P              % coupler point P, assembly mode 1
%
% BY:
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025


% --- Parse geometry --------------------------------------------------
if isstruct(geo)
    a = geo.a; b = geo.b; c = geo.c; d = geo.d;
    e = geo.e; epsilon = geo.epsilon; delta = geo.delta;
    [h_q, eta_q, h_r, eta_r] = parse_QR_fields(geo);
else
    a = geo(1); b = geo(2); c = geo(3); d = geo(4);
    e = geo(5); epsilon = geo(6); delta = geo(7);
    h_q = NaN; eta_q = NaN; h_r = NaN; eta_r = NaN;
end

% --- Fixed pivots and A position (column vectors) --------------------
O = [0; 0];
C = [d*cos(delta); d*sin(delta)];
A = a * [cos(alpha); sin(alpha)];

% E matrix for planar cross products
E = [0 -1; 1 0];

rOA = A;
rOC = C;

% --- Optional point R on link O-A (independent of assembly mode) -----
nan2 = [NaN; NaN];
R_pt = nan2;
if ~isnan(h_r) && ~isnan(eta_r)
    ang_OA = atan2(A(2)-O(2), A(1)-O(1));   % direction O→A
    ang_OR = ang_OA + eta_r;                % rotate by eta_r at vertex O
    R_pt = O + h_r * [cos(ang_OR); sin(ang_OR)];
end

% --- Preallocate output ----------------------------------------------
nan3 = [NaN; NaN; NaN];
sol = repmat(struct( ...
    'Positions', struct('O',O,'A',A,'B',nan2,'C',C,'Q',nan2,'R',R_pt), ...
    'P', nan2, 'phi', NaN, 'alpha', alpha, 'theta', NaN, ...
    'valid', 0, ...
    'Twists', struct('xiO',[1;0;0],'xiA',[1;E*rOA],'xiB',nan3,'xiC',[1;E*rOC])), 1, 2);

% --- Circle intersections: B lies on circle(A,b) ∩ circle(C,c) ------
[B1, B2, validB] = circle_intersections(A, b, C, c);
if ~validB
    return;
end

% E matrix for planar cross products
E = [0 -1; 1 0];

% --- Solve for both assembly modes -----------------------------------
for k = 1:2
    B = [B1, B2]; B = B(:,k);

    rOB = B;

    phi   = atan2(B(2)-A(2), B(1)-A(1));
    theta = atan2(B(2)-C(2), B(1)-C(1));
    P     = A + e * [cos(phi+epsilon); sin(phi+epsilon)];

    % --- Optional point Q on link B-C ---------------------------------
    Q = nan2;
    if ~isnan(h_q) && ~isnan(eta_q)
        ang_BC = atan2(C(2)-B(2), C(1)-B(1));   % direction B→C
        ang_BQ = ang_BC + eta_q;                % rotate by eta_q at vertex B
        Q = B + h_q * [cos(ang_BQ); sin(ang_BQ)];
    end

    sol(k).Positions.B = B;
    sol(k).Positions.Q = Q;
    sol(k).Twists.xiO  = [1; 0; 0];
    sol(k).Twists.xiA  = [1; E*rOA];
    sol(k).Twists.xiB  = [1; E*rOB];
    sol(k).Twists.xiC  = [1; E*rOC];
    sol(k).P           = P;
    sol(k).phi         = phi;
    sol(k).theta       = theta;
    sol(k).valid       = 1;
    sol(k).Twists.xiB  = [1; E*rOB];
end
end

% Circle Intersection Helper
function [P1, P2, valid] = circle_intersections(c1, r1, c2, r2)
d = norm(c2 - c1);
if d > (r1 + r2) || d < abs(r1 - r2)
    P1 = [NaN;NaN]; P2 = [NaN;NaN]; valid = false; return;
end
a_val  = (r1^2 - r2^2 + d^2) / (2*d);
h_val  = sqrt(max(r1^2 - a_val^2, 0));
p2     = c1 + a_val * (c2-c1) / d;
perp   = h_val * [0 -1; 1 0] * ((c2-c1)/d);
P1     = p2 + perp;
P2     = p2 - perp;
valid  = true;
end

function [h_q, eta_q, h_r, eta_r] = parse_QR_fields(geo)
% Reads optional Q/R geometry fields; returns NaN for any missing field
% so that the point itself is left as NaN (i.e. simply ignored).
h_q = NaN; eta_q = NaN; h_r = NaN; eta_r = NaN;
if isfield(geo,'h_q'),   h_q   = geo.h_q;   end
if isfield(geo,'eta_q'), eta_q = geo.eta_q; end
if isfield(geo,'h_r'),   h_r   = geo.h_r;   end
if isfield(geo,'eta_r'), eta_r = geo.eta_r; end
end
