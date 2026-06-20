function sols = stephensonIII_direct_kinematics(geo, theta_O)
% stephensonIII_direct_kinematics - Direct kinematics for a planar Stephenson III six-bar linkage
%
% INPUTS:
%   geo     : [OA, Bx, By, OC, CD, DA, BE, EM, DM, MP, eta, delta]
%             where
%                OA   - O to A (along x-axis)
%                Bx   - x coordinate of B
%                By   - y coordinate of B
%                OC   - O to C (input crank)
%                CD   - C to D
%                DA   - D to A
%                BE   - B to E
%                EM   - E to M
%                DM   - D to M (new parameter)
%                MP   - M to P (output point, as before)
%                eta  - angle (deg) from ME to MP (positive CCW)
%                delta - angle (deg) from CD to CM, measured at C (positive CCW)
%   theta_O : Input crank angle (deg), angle of OC w.r.t. x-axis (positive CCW)
%
% OUTPUTS:
%   sols    : Structure array with one entry per valid assembly solution.
%             Each sols(i) contains:
%                .Positions.O  - [x;y] position of O (origin)
%                .Positions.A  - [x;y] position of A (on x-axis)
%                .Positions.B  - [x;y] position of B (ground)
%                .Positions.C  - [x;y] position of C (end of input crank)
%                .Positions.D  - [x;y] position of D
%                .Positions.E  - [x;y] position of E
%                .Positions.M  - [x;y] position of M (end of output link)
%                .Positions.P  - [x;y] position of output point P
%                .Angles.thetaC    - angle of CD w.r.t. CO at C (deg)
%                .Angles.thetaD    - angle of DA w.r.t. DC at D (deg)
%                .Angles.thetaE    - angle of EM w.r.t. EB at E (deg)
%                .Angles.thetaA    - angle of AB w.r.t. AO at A (deg)
%                .Angles.thetaB    - angle of BE w.r.t. BO at B (deg)
%                .Angles.thetaM    - angle of MP w.r.t. ME at M (deg)
%                .Angles.thetaO    - angle of OC w.r.t. OA at O (deg)
%                .Angles.theta_CM  - angle of CM w.r.t. x-axis (deg)
%                .Angles.theta_MP  - angle of MP w.r.t. x-axis (deg)
%                .theta_EM         - angle of EM (E to M) w.r.t. x-axis (deg)
%                .valid            - 1 if solution is valid, 0 otherwise
%
%   If no solution exists, sols is a structure with .valid = 0 and all other fields NaN.
%
%   All angles are in degrees, positive CCW from the reference vector.
%
% geo = [40, 70, 30, 50, 20, 50, 30, 30, -30, 20, 30, 60]; thetaO = 90; sols = stephensonIII_direct_kinematics(geo, thetaO)
% sols = 
%   4×1 struct array with fields:
%     Positions
%     Angles
%     theta_EM
%     valid
%     thetaO
%     thetaB
% sols.thetaB
% ans =
%   141.9562
% ans =
%    69.0020
% ans =
%   214.8062
% ans =
%   136.4307

OA    = geo(1);
Bx    = geo(2);
By    = geo(3);
OC    = geo(4);
CD    = geo(5);
DA    = geo(6);
BE    = geo(7);
EM    = geo(8);
DM    = geo(9);
MP    = geo(10);
eta   = geo(11);
delta = geo(12);

O = [0; 0];
A = [OA; 0];
B = [Bx; By];

sols = [];

theta = deg2rad(theta_O);
C = O + OC * [cos(theta); sin(theta)];

[D1, D2] = circle_intersections(C, CD, A, DA);
D_all = [D1, D2];

for iD = 1:2
    D = D_all(:,iD);
    if any(isnan(D)), continue; end

    % ---- Calculate position of M using DM and delta (angle from CD to CM at C, CCW) ----
    v_CD = D - C;
    v_CD_unit = v_CD / norm(v_CD);
    R_delta = [cosd(delta) -sind(delta); sind(delta) cosd(delta)];
    v_DM_unit = R_delta * (-v_CD_unit);
    M = D + DM * v_DM_unit;


    % Now, E is at intersection of circles (B, BE) and (M, EM)
    [E1, E2] = circle_intersections(B, BE, M, EM);
    E_all = [E1, E2];

    for iE = 1:2
        E = E_all(:,iE);
        if any(isnan(E)), continue; end

        % Output P: at distance MP from M, rotated eta from ME
        v_ME = E - M;
        if norm(v_ME) == 0, continue; end
        v_ME_unit = v_ME / norm(v_ME);
        R_eta = [cosd(eta), -sind(eta); sind(eta), cosd(eta)];
        v_MP = R_eta * v_ME_unit;
        P = M + MP * v_MP;

        % Angles (same as before)
        Angles.thetaC   = rel_angle(D - C, C - O);          % CD w.r.t. CO
        Angles.thetaD   = rel_angle(A - D, D - C);          % DA w.r.t. DC
        Angles.thetaE   = rel_angle(M - E, E - B);          % EM w.r.t. EB
        Angles.thetaA   = rel_angle(B - A, A - O);          % AB w.r.t. AO
        Angles.thetaB   = rel_angle(E - B, B - O);          % BE w.r.t. BO
        Angles.thetaM   = rel_angle(P - M, M - E);          % MP w.r.t. ME
        Angles.thetaO   = rel_angle(C - O, A - O);          % OC w.r.t. OA
        Angles.theta_CM = vec_angle_xaxis(M - C);           % CM w.r.t. x-axis
        Angles.theta_MP = vec_angle_xaxis(P - M);           % MP w.r.t. x-axis

        v_EM_plot = M - E;
        theta_EM = vec_angle_xaxis(v_EM_plot);

        % Store result
        sol.Positions.O = O;
        sol.Positions.A = A;
        sol.Positions.B = B;
        sol.Positions.C = C;
        sol.Positions.D = D;
        sol.Positions.E = E;
        sol.Positions.M = M;
        sol.Positions.P = P;

        sol.Angles = Angles;
        sol.theta_EM = theta_EM;
        sol.valid = 1;
        sol.thetaO=theta_O;
        sol.thetaB=Angles.thetaB;

        sols = [sols; sol];
    end
end

if isempty(sols)
    % Return a single invalid solution
    nan2 = [NaN; NaN];
    sol.Positions.O = nan2;
    sol.Positions.A = nan2;
    sol.Positions.B = nan2;
    sol.Positions.C = nan2;
    sol.Positions.D = nan2;
    sol.Positions.E = nan2;
    sol.Positions.M = nan2;
    sol.Positions.P = nan2;
    flds = {'thetaC','thetaD','thetaE','thetaA','thetaB','thetaM','thetaO','theta_CM','theta_MP'};
    for k = 1:numel(flds), sol.Angles.(flds{k}) = NaN; end
    sol.theta_EM = NaN;
    sol.valid = 0;
    sol.thetaO=theta_0;
    sol.thetaB=NaN;
    sols = sol;
end

end

% --- Relative angle from v1 to v2 (deg, positive CCW, normalized 0..360) ---
function theta = rel_angle(v2, v1)
theta = atan2d(v2(2), v2(1)) - atan2d(v1(2), v1(1));
theta = mod(theta + 360, 360);
end

% --- Angle of vector w.r.t. x-axis (deg, [0,360)) ---
function theta = vec_angle_xaxis(v)
theta = atan2d(v(2), v(1));
theta = mod(theta + 360, 360);
end

% --- Intersection of two circles (returns up to two points, columns) ---
function [p1, p2] = circle_intersections(c1, r1, c2, r2)
d = norm(c2-c1);
if d > r1+r2 || d < abs(r1-r2)
    p1 = [NaN; NaN]; p2 = [NaN; NaN]; return
end
a = (r1^2 - r2^2 + d^2) / (2*d);
h = sqrt(max(0, r1^2 - a^2));
p0 = c1 + a*(c2-c1)/d;
if h < 1e-12
    p1 = p0; p2 = p0;
    return
end
offset = h * [0 -1; 1 0] * (c2-c1)/d;
p1 = p0 + offset;
p2 = p0 - offset;
end