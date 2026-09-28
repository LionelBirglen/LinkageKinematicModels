function sols = stephensonIII_inverse_kinematics(geo, thetaB)
% stephensonIII_inverse_kinematics - Inverse kinematics for Stephenson III six-bar linkage
%
% INPUTS:
%   geo     : [OA, Bx, By, OC, CD, DA, BF, FE, DE, EP, eta, delta]
%             where
%                OA   - O to A (along x-axis)
%                Bx   - x coordinate of B
%                By   - y coordinate of B
%                OC   - O to C (input crank)
%                CD   - C to D
%                DA   - D to A
%                BF   - B to F
%                FE   - F to E
%                DE   - D to E
%                EP   - E to P (output point)
%                eta  - angle (deg) from EF to EP (positive CCW)
%                delta - angle (deg) of body C-D-E, same definition as in
%                        stephensonIII_direct_kinematics
%   thetaB  : angle (deg, CCW) of BF, measured from the direction of the
%             vector O->B (same convention as before)
%
% OUTPUTS:
%   sols    : Structure array with one entry per REAL assembly, i.e. every
%             configuration in which all links close (OC, CD, DA, BF, FE
%             and the bodies C-D-E and F-E-P). Same fields as before:
%                .Positions.O .A .B .C .D .F .E .P  - [x;y]
%                .Angles.thetaC  .thetaD  .thetaF  .thetaA  .thetaB
%                       .thetaE  .thetaO  .theta_CE .theta_EP  (deg)
%                .theta_FE  - angle of FE w.r.t. x-axis (deg)
%                .valid     - 1
%                .thetaO    - input crank angle (deg, in (-180,180])
%                .thetaB    - thetaB, same as input
%             Entries are sorted by thetaO. Each one is also a solution of
%             stephensonIII_direct_kinematics(geo, thetaO).
%
%   If no solution exists, sols is a structure with .valid = 0 and all
%   other fields NaN (except .thetaB).
%
% METHOD:
%   With thetaB given, F is fixed. For each of the two assembly modes of
%   the four-bar O-C-D-A (branches of D, same ordering as the direct
%   kinematics), E traces the coupler curve of body C-D-E as the crank
%   turns. The solutions are the crank angles at which |E - F| = FE.
%   They are bracketed by a dense sweep of thetaO (to which the crank's
%   exact limit angles are added, so roots next to a limit are not
%   missed) and refined with fzero. Every candidate is built with the
%   same formulas as stephensonIII_direct_kinematics, so D always lies at
%   distance DA from A. At most 6 real solutions can exist (circle vs.
%   tricircular sextic coupler curve).
%
% EXAMPLE:
%   geo = [40, 70, 30, 50, 20, 50, 30, 30, -30, 20, 30, 60];
%   sols = stephensonIII_inverse_kinematics(geo, 69);
%   [sols.thetaO]      % -> 47.24  90.00  (the two real assemblies)

OA = geo(1);  Bx = geo(2);  By = geo(3);
OC = geo(4);  CD = geo(5);  DA = geo(6);
BF = geo(7);  FE = geo(8);  DE = geo(9);
EP = geo(10); eta = geo(11); delta = geo(12);

O = [0; 0];
A = [OA; 0];
B = [Bx; By];

% --- F from thetaB (measured from the direction O->B, as before) -------
thetaB_global = atan2(By, Bx) + deg2rad(thetaB);
F = B + BF * [cos(thetaB_global); sin(thetaB_global)];

% --- Dense sweep of the crank angle, plus the exact limit angles --------
% The four-bar O-C-D-A assembles when |CD-DA| <= |AC| <= CD+DA, with
% |AC|^2 = OA^2 + OC^2 - 2*OA*OC*cos(thetaO).
N  = 7200;
th = linspace(-pi, pi, N+1);
th = th(1:end-1);
if OA ~= 0 && OC ~= 0
    for L = [CD + DA, abs(CD - DA)]
        c = (OA^2 + OC^2 - L^2) / (2*OA*OC);
        if abs(c) <= 1
            th = [th, acos(c), -acos(c)]; %#ok<AGROW>
        end
    end
end
th = unique(mod(th + pi, 2*pi) - pi);
th = [th, th(1) + 2*pi];          % close the loop

% --- Roots on each D branch ----------------------------------------------
% The sweep is vectorized (same formulas as config below); fzero then
% refines each bracketed root with the scalar residual.
Fres = sweep_residuals(th);          % 2 x numel(th), NaN where no assembly
roots = zeros(0, 2);              % [thetaO (rad), branch]
for b = 1:2
    f = Fres(b,:);
    for i = find(isfinite(f(1:end-1)) & isfinite(f(2:end)) & ...
                 (f(1:end-1) == 0 | f(1:end-1).*f(2:end) < 0))
        if f(i) == 0
            roots(end+1,:) = [th(i) b]; %#ok<AGROW>
        else
            t = fzero(@(x) residual(x, b), [th(i) th(i+1)]);
            roots(end+1,:) = [t b]; %#ok<AGROW>
        end
    end
end

% --- Build the solutions ------------------------------------------------
sols = [];
R_eta = [cosd(eta), -sind(eta); sind(eta), cosd(eta)];
tol   = 1e-6 * max([1, abs(geo(1:10))]);
for k = 1:size(roots, 1)
    [C, D, E, ok] = config(roots(k,1), roots(k,2));
    if ~ok, continue; end
    % keep only configurations in which every link closes
    err = max(abs([norm(C-O)-abs(OC), norm(D-C)-abs(CD), norm(D-A)-abs(DA), ...
                   norm(E-F)-abs(FE), norm(F-B)-abs(BF)]));
    if err > tol, continue; end

    v_EF = F - E;                          % as in the direct kinematics
    P = E + EP * (R_eta * (v_EF / norm(v_EF)));

    Angles.thetaC   = rel_angle(D - C, C - O);
    Angles.thetaD   = rel_angle(A - D, D - C);
    Angles.thetaF   = rel_angle(E - F, F - B);
    Angles.thetaA   = rel_angle(B - A, A - O);
    Angles.thetaB   = rel_angle(F - B, B - O);
    Angles.thetaE   = rel_angle(P - E, E - F);
    Angles.thetaO   = rel_angle(C - O, A - O);
    Angles.theta_CE = vec_angle_xaxis(E - C);
    Angles.theta_EP = vec_angle_xaxis(P - E);

    sol.Positions.O = O;
    sol.Positions.A = A;
    sol.Positions.B = B;
    sol.Positions.C = C;
    sol.Positions.D = D;
    sol.Positions.F = F;
    sol.Positions.E = E;
    sol.Positions.P = P;
    sol.Angles   = Angles;
    sol.theta_FE = vec_angle_xaxis(E - F);
    sol.valid    = 1;
    sol.thetaO   = rad2deg(atan2(C(2), C(1)));
    sol.thetaB   = thetaB;

    % skip duplicates (same assembly found twice, e.g. at a crank limit)
    isDup = false;
    for q = 1:numel(sols)
        if norm(sols(q).Positions.C - C) < tol && norm(sols(q).Positions.D - D) < tol
            isDup = true; break;
        end
    end
    if ~isDup
        sols = [sols; sol]; %#ok<AGROW>
    end
end

if ~isempty(sols)
    [~, order] = sort([sols.thetaO]);
    sols = sols(order);
else
    nan2 = [NaN; NaN];
    sol = struct();
    sol.Positions.O = nan2;
    sol.Positions.A = nan2;
    sol.Positions.B = nan2;
    sol.Positions.C = nan2;
    sol.Positions.D = nan2;
    sol.Positions.F = nan2;
    sol.Positions.E = nan2;
    sol.Positions.P = nan2;
    flds = {'thetaC','thetaD','thetaF','thetaA','thetaB','thetaE','thetaO','theta_CE','theta_EP'};
    for k = 1:numel(flds), sol.Angles.(flds{k}) = NaN; end
    sol.theta_FE = NaN;
    sol.valid    = 0;
    sol.thetaO   = NaN;
    sol.thetaB   = thetaB;
    sols = sol;
end

    % --- nested helpers (share geometry and F) ---------------------------
    function [C, D, E, ok] = config(t, b)
        % C, D (branch b) and E, with the formulas of the direct kinematics
        C = O + OC * [cos(t); sin(t)];
        [D1, D2] = circle_intersections(C, CD, A, DA);
        if b == 1, D = D1; else, D = D2; end
        ok = all(isfinite(D));
        if ~ok, E = [NaN; NaN]; return; end
        v_CD_unit = (D - C) / norm(D - C);
        R_delta = [cosd(delta) -sind(delta); sind(delta) cosd(delta)];
        E = D + DE * (R_delta * (-v_CD_unit));
    end

    function r = residual(t, b)
        [~, ~, E, ok] = config(t, b);
        if ok
            r = norm(E - F) - FE;
        else
            r = NaN;
        end
    end

    function Fres = sweep_residuals(t)
        % Vectorized version of residual for all angles t and both
        % branches (same formulas and branch ordering as config)
        t  = t(:).';
        Cv = OC * [cos(t); sin(t)];
        dv = A - Cv;                               % C -> A
        d  = sqrt(sum(dv.^2, 1));
        tolc = 1e-9 * max([1, abs(CD), abs(DA)]);
        okv = d > 0 & d <= CD + DA + tolc & d >= abs(CD - DA) - tolc;
        a  = (CD^2 - DA^2 + d.^2) ./ (2*d);
        h  = sqrt(max(0, CD^2 - a.^2));
        u  = dv ./ d;                              % unit C -> A
        p0 = Cv + u .* a;
        off = [-u(2,:); u(1,:)] .* h;              % [0 -1; 1 0] * u * h
        R_delta = [cosd(delta) -sind(delta); sind(delta) cosd(delta)];
        Fres = nan(2, numel(t));
        for bb = 1:2
            Dv = p0 + (3 - 2*bb) * off;            % bb=1: +offset
            ucd = (Dv - Cv) ./ sqrt(sum((Dv - Cv).^2, 1));
            Ev  = Dv + DE * (R_delta * (-ucd));
            r   = sqrt(sum((Ev - F).^2, 1)) - FE;
            r(~okv) = NaN;
            Fres(bb,:) = r;
        end
    end

end


%% ---- HELPERS ----

function [p1, p2] = circle_intersections(c1, r1, c2, r2)
% Same as in stephensonIII_direct_kinematics (same branch ordering), with
% a tiny tolerance so that the exact crank-limit angles (tangent circles)
% still give a point despite rounding
d = norm(c2-c1);
tolc = 1e-9 * max([1, abs(r1), abs(r2)]);
if d > r1+r2+tolc || d < abs(r1-r2)-tolc
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

function theta = rel_angle(v2, v1)
theta = atan2d(v2(2), v2(1)) - atan2d(v1(2), v1(1));
theta = mod(theta + 360, 360);
end

function theta = vec_angle_xaxis(v)
theta = atan2d(v(2), v(1));
theta = mod(theta + 360, 360);
end
