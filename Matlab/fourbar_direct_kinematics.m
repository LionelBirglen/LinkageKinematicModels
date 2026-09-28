function sol = fourbar_direct_kinematics(geo, theta)
% FOURBAR_DIRECT_KINEMATICS - Direct kinematics of a planar four-bar linkage
%
% INPUTS:
%   geo   - geometry, as a 1×7 numeric vector [a b c d e epsilon delta]
%           OR a struct with fields .a .b .c .d .e .epsilon .delta
%           a       : length of output link (O→A)
%           b       : length of coupler link (B→A)
%           c       : length of input crank (C→B)
%           d       : length of fixed ground link (O→C)
%           e       : distance A→P along coupler
%           epsilon : angle between coupler (B→A) and A→P (rad)
%           delta   : angle of ground link O→C (rad)
%           Optional fields for additional points of interest:
%           h_q     : distance B→Q along the input crank link B-C
%           eta_q   : angle C-B-Q, measured at vertex B from ray B→C (rad)
%           h_r     : distance O→R along the output link O-A
%           eta_r   : angle A-O-R, measured at vertex O from ray O→A (rad)
%   theta - input crank angle (C→B) relative to ground (rad)
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
%     .alpha        - output link angle O→A (rad)
%     .theta        - input crank angle (rad), same as input
%     .valid        - 1 if configuration closes, 0 otherwise
%
% USAGE EXAMPLE:
%   % Vector form (backward compatible):
%   geo = [.81, .88, .92, 1.51, .8, pi/6, -10*pi/180];
%   sol = fourbar_direct_kinematics(geo, deg2rad(106));
%   % Struct form:
%   geo = struct('a',.81,'b',.88,'c',.92,'d',1.51,'e',.8,'epsilon',pi/6,'delta',-10*pi/180);
%   sol = fourbar_direct_kinematics(geo, deg2rad(106));
%   % Struct form with optional Q and R points:
%   geo.h_q = .3; geo.eta_q = pi/6; geo.h_r = .4; geo.eta_r = pi/4;
%   sol = fourbar_direct_kinematics(geo, deg2rad(106));
%   % Access results:
%   sol(1).Positions.A   % joint A, assembly mode 1
%   sol(1).P             % coupler point P, assembly mode 1
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

% --- Fixed pivots (column vectors) -----------------------------------
O = [0; 0];
C = [d*cos(delta); d*sin(delta)];

% --- Input crank endpoint B ------------------------------------------
B = C + c * [cos(theta); sin(theta)];

% E matrix for planar cross products
E = [0 -1; 1 0];

rOB = B;
rOC = C;

% --- Preallocate output ----------------------------------------------
nan2 = [NaN; NaN];
sol = repmat(struct( ...
    'Positions', struct('O',nan2,'A',nan2,'B',nan2,'C',nan2,'Q',nan2,'R',nan2), ...
    'P', nan2, 'phi', NaN, 'alpha', NaN, 'theta', theta, ...
    'valid', 0, ...
    'Twists', struct('xiO',[1;0;0],'xiA',nan2,'xiB',nan2,'xiC',[1;E*rOC])), 1, 2);

% --- Feasibility check -----------------------------------------------
R = norm(O - B);
if R > (a + b) || R < abs(a - b)
    % Both modes invalid — return with valid=0
    for k = 1:2
        sol(k).Positions.O = O;
        sol(k).Positions.B = B;
        sol(k).Positions.C = C;
    end
    return;
end

% --- Solve for both assembly modes -----------------------------------
angle_BO = atan2(O(2)-B(2), O(1)-B(1));
cos_arg  = min(max((b^2 + R^2 - a^2) / (2*b*R), -1), +1);
epsilon_B = acos(cos_arg);

for k = 1:2
    cfg = 2*k - 3;  % k=1 → cfg=-1, k=2 → cfg=+1

    phi_old = angle_BO + cfg * epsilon_B;
    phi_old = atan2(sin(phi_old), cos(phi_old));

    phi  = phi_old + pi;
    phi  = atan2(sin(phi), cos(phi));

    A     = B + b * [cos(phi_old); sin(phi_old)];
    alpha = atan2(sin(atan2(A(2),A(1))), cos(atan2(A(2),A(1))));
    P     = A + e * [cos(phi+epsilon); sin(phi+epsilon)];

    rOA = A;

    % --- Optional point Q on link B-C ---------------------------------
    Q = nan2;
    if ~isnan(h_q) && ~isnan(eta_q)
        ang_BC = atan2(C(2)-B(2), C(1)-B(1));   % direction B→C
        ang_BQ = ang_BC + eta_q;                % rotate by eta_q at vertex B
        Q = B + h_q * [cos(ang_BQ); sin(ang_BQ)];
    end

    % --- Optional point R on link O-A ---------------------------------
    R_pt = nan2;
    if ~isnan(h_r) && ~isnan(eta_r)
        ang_OA = atan2(A(2)-O(2), A(1)-O(1));   % direction O→A
        ang_OR = ang_OA + eta_r;                % rotate by eta_r at vertex O
        R_pt = O + h_r * [cos(ang_OR); sin(ang_OR)];
    end

    sol(k).Positions.O = O;
    sol(k).Positions.A = A;
    sol(k).Positions.B = B;
    sol(k).Positions.C = C;
    sol(k).Positions.Q = Q;
    sol(k).Positions.R = R_pt;
    sol(k).P           = P;
    sol(k).phi         = phi;
    sol(k).alpha       = alpha;
    sol(k).theta       = theta;
    sol(k).valid       = 1;
    sol(k).Twists.xiO  = [1; 0; 0];
    sol(k).Twists.xiA  = [1; E*rOA];
    sol(k).Twists.xiB  = [1; E*rOB];
    sol(k).Twists.xiC  = [1; E*rOC];
end

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
