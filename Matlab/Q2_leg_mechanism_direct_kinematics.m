function sol = Q2_leg_mechanism_direct_kinematics(thetaA, rho, parameters)
% Q2_LEG_MECHANISM_DIRECT_KINEMATICS - Direct kinematics of the planar leg mechanism
%
% INPUTS:
%   thetaA     - input crank angle of link A->C relative to ground (rad)
%   rho        - length of the prismatic actuator E->K
%   parameters - struct with exactly these 17 fields, all required and
%                independent (lengths in consistent units, angles in rad):
%     Lengths (12):
%       .AB  : ground link, A = (0,0), B = (AB,0)
%       .AC  : input crank A->C
%       .BD  : B->D   (ternary body B-D-E)
%       .BE  : B->E   (ternary body B-D-E)
%       .CD  : C->D   (quaternary body C-D-G-F)
%       .CF  : C->F   (quaternary body C-D-G-F)
%       .FG  : F->G   (quaternary body C-D-G-F)
%       .GK  : G->K   (ternary body G-K-J)
%       .GJ  : G->J   (ternary body G-K-J)
%       .IF  : link I-F
%       .IJ  : I->J   (ternary body I-J-P)
%       .IP  : I->P   (ternary body I-J-P)
%     Angles (5):
%       .DCF : angle D-C-F at C, clockwise from ray C->D to ray C->F
%       .GFC : angle G-F-C at F, clockwise from ray F->C to ray F->G
%       .DBE : angle D-B-E at B, counter-clockwise from ray B->D to B->E
%       .JGK : angle J-G-K at G, clockwise from ray G->K to ray G->J
%       .JIP : angle J-I-P at I, counter-clockwise from ray I->J to I->P
%              (same definition as epsilon for P in
%              fourbar_direct_kinematics)
%     The quaternary body C-D-G-F is fully defined by CD, CF, FG, DCF and
%     GFC (the side D-G follows from them), so no other field is used.
%
% OUTPUT:
%   sol - 1x8 struct array, one element per assembly mode. The index is
%         k = 4*(iD-1) + 2*(iK-1) + iI with iD, iK, iI in {1,2} the
%         branches of the D, K and I circle intersections, so a given
%         index always refers to the same branch. Each element has:
%     .Positions.A ... .Positions.I - [x;y] joint positions
%                                     (A, B, C, D, E, F, G, K, J, I)
%     .P      - [x;y] point of interest P on body I-J-P
%     .phi    - angle of I->J relative to ground (rad)
%     .thetaA - input crank angle (rad), same as input
%     .rho    - actuator length, same as input
%     .valid  - 1 if the branch assembles, 0 otherwise. Positions that
%               could be computed before closure failed are still filled,
%               the rest (and P, phi) are NaN.
%     .Twists.xiX - [1; E*rOX], coordinates of the zero-pitch twist at
%                   joint X (X = A, B, C, D, E, F, G, K, J, I)
%
% USAGE EXAMPLE:
%   p = struct('AB',150,'AC',90,'BD',120,'CD',100,'FG',80,'CF',90, ...
%              'BE',50,'GK',50,'GJ',80,'IF',80,'IJ',100,'IP',50, ...
%              'DCF',deg2rad(80),'GFC',deg2rad(110),'DBE',deg2rad(45), ...
%              'JGK',deg2rad(30),'JIP',deg2rad(30));
%   sol = Q2_leg_mechanism_direct_kinematics(deg2rad(-80), 220, p);
%   sol(2).Positions.I   % joint I, assembly mode 2
%   sol(2).P             % point P, assembly mode 2
%   find([sol.valid])    % branches that assemble
%
% BY:
% Prof. Lionel Birglen
% Polytechnique Montreal

p = parameters;
check_params(p);

% E matrix for planar cross products / 90 deg rotation
Emat = [0 -1; 1 0];
nan2 = [NaN; NaN];
nan3 = [NaN; NaN; NaN];

% --- Fixed pivots and input crank -------------------------------------
A = [0; 0];
B = [p.AB; 0];
C = A + p.AC * [cos(thetaA); sin(thetaA)];

% --- Preallocate output -----------------------------------------------
pos0 = struct('A',A,'B',B,'C',C,'D',nan2,'E',nan2,'F',nan2, ...
              'G',nan2,'K',nan2,'J',nan2,'I',nan2);
tw0  = struct('xiA',[1;Emat*A],'xiB',[1;Emat*B],'xiC',[1;Emat*C], ...
              'xiD',nan3,'xiE',nan3,'xiF',nan3,'xiG',nan3, ...
              'xiK',nan3,'xiJ',nan3,'xiI',nan3);
sol = repmat(struct('Positions',pos0,'P',nan2,'phi',NaN, ...
    'thetaA',thetaA,'rho',rho,'valid',0,'Twists',tw0), 1, 8);

% --- Solve D (two branches) -------------------------------------------
[Dsol, okD] = circle_circle_intersection(B, p.BD, C, p.CD);
if ~okD
    return;   % all 8 branches invalid
end

for iD = 1:2
    D = Dsol(:,iD);

    % --- Quaternary body C-D-G-F (F and G depend on D) ----------------
    xC_local = (D - C) / norm(D - C);
    yC_local = Emat * xC_local;
    F = C + p.CF * (cos(p.DCF) * xC_local - sin(p.DCF) * yC_local);

    xF_local = (C - F) / norm(C - F);
    yF_local = Emat * xF_local;
    G = F + p.FG * (cos(p.GFC) * xF_local - sin(p.GFC) * yF_local);

    % --- Ternary body B-D-E (E depends on D) --------------------------
    xB_local = (D - B) / norm(D - B);
    yB_local = Emat * xB_local;
    E = B + p.BE * (cos(p.DBE) * xB_local + sin(p.DBE) * yB_local);

    % --- Solve K (two branches) ---------------------------------------
    [Ksol, okK] = circle_circle_intersection(E, rho, G, p.GK);

    for iK = 1:2
        K = nan2;  J = nan2;  okI = false;  Isol = nan(2,2);
        if okK
            K = Ksol(:,iK);
            % --- Ternary body G-K-J -----------------------------------
            u_GK = (K - G) / norm(K - G);
            Rm = [cos(-p.JGK), -sin(-p.JGK); sin(-p.JGK), cos(-p.JGK)];
            J = G + p.GJ * (Rm * u_GK);
            % --- Solve I (two branches) -------------------------------
            [Isol, okI] = circle_circle_intersection(F, p.IF, J, p.IJ);
        end

        for iI = 1:2
            k = 4*(iD-1) + 2*(iK-1) + iI;

            sol(k).Positions.D = D;
            sol(k).Positions.E = E;
            sol(k).Positions.F = F;
            sol(k).Positions.G = G;
            sol(k).Positions.K = K;
            sol(k).Positions.J = J;
            sol(k).Twists.xiD = [1; Emat*D];
            sol(k).Twists.xiE = [1; Emat*E];
            sol(k).Twists.xiF = [1; Emat*F];
            sol(k).Twists.xiG = [1; Emat*G];
            if okK
                sol(k).Twists.xiK = [1; Emat*K];
                sol(k).Twists.xiJ = [1; Emat*J];
            end

            if ~okI
                continue;   % branch does not close: valid stays 0
            end

            I   = Isol(:,iI);
            phi = atan2(J(2)-I(2), J(1)-I(1));   % direction I->J

            % --- Point P on body I-J-P (as P on the fourbar coupler) --
            P = I + p.IP * [cos(phi + p.JIP); sin(phi + p.JIP)];

            sol(k).Positions.I = I;
            sol(k).Twists.xiI  = [1; Emat*I];
            sol(k).P           = P;
            sol(k).phi         = phi;
            sol(k).valid       = 1;
        end
    end
end
end


% ======================================================================
%  local helpers
% ======================================================================
function check_params(p)
% All 17 independent parameters must be present
if ~isstruct(p)
    error('Q2_leg_mechanism_direct_kinematics:BadParams', 'PARAMETERS must be a struct.');
end
must = {'AB','AC','BD','CD','FG','CF','BE','GK','GJ','IF','IJ','IP', ...
        'DCF','GFC','DBE','JGK','JIP'};
for k = 1:numel(must)
    if ~isfield(p, must{k})
        error('Q2_leg_mechanism_direct_kinematics:BadParams', ...
            'Missing field "%s" in parameter struct.', must{k});
    end
end
end


function [Psol, ok] = circle_circle_intersection(O1, r1, O2, r2)
% Intersections of circles (O1,r1) and (O2,r2). Psol = [P1 P2] (2x2),
% same ordering as the original implementation (P0 + offset first).
Psol = nan(2,2);
d = norm(O2 - O1);
ok = ~(d == 0 || d > r1 + r2 || d < abs(r1 - r2));
if ~ok
    return;
end
a  = (r1^2 - r2^2 + d^2) / (2*d);
h  = sqrt(max(r1^2 - a^2, 0));
P0 = O1 + a * (O2 - O1) / d;
offset = h * [0 -1; 1 0] * (O2 - O1) / d;
Psol = [P0 + offset, P0 - offset];
end
