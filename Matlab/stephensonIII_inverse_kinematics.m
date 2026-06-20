function sols = stephensonIII_inverse_kinematics(geo, thetaB)
% stephensonIII_inverse_kinematics - Inverse kinematics for Stephenson III six-bar linkage
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
%   thetaB  : Angle of BE relative to x-axis (deg, CCW, desired output)
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
% geo = [40, 70, 30, 50, 20, 50, 30, 30, -30, 20, 30, 60]; thetaB = 69; sols = stephensonIII_inverse_kinematics(geo, thetaB)
% sols = 
%   4×1 struct array with fields:
%     Positions
%     Angles
%     theta_EM
%     valid
%     thetaO
%     thetaB
% 
% sols.thetaO
% ans =
%    72.7801
% ans =
%    47.2390
% ans =
%    89.9958
% ans =
%    29.0143

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

% Step 1: Compute E from thetaB (relative to BO)
angle_BO = atan2(By, Bx); % global angle from B to O
thetaB_global = angle_BO + deg2rad(thetaB);
E = B + BE * [cos(thetaB_global); sin(thetaB_global)];

% Step 2: Discretize M positions around circle
N = 720; % Number of thetaO samples
theta_samples = linspace(0, 2*pi, N);
M_curve=E+EM*[cos(theta_samples);sin(theta_samples)];

% Step 3: Compute Coupler Curve with M and verify if goes through zero
coupler(1)=fourbar_coupler_curve(delta,OA,DM,CD,OC,DA,M_curve(1,1),M_curve(2,1));
m=0;
for i=2:N
    coupler(i)=fourbar_coupler_curve(delta,OA,DM,CD,OC,DA,M_curve(1,i),M_curve(2,i));
    if coupler(i)*coupler(i-1)<0
        m=m+1;
        sols_index(m)=i;
    end
end
if coupler(1)*coupler(N)<0
    m=m+1;
    sols_index(m)=N;
end

% Step four: Refine solutions
for i=1:m
    sols_theta(i)=fzero(@(x) fourbar_coupler_curve_wrapper(x,delta,OA,DM,CD,OC,DA,E,EM),theta_samples(sols_index(i)));
end

sols=[];

% p=0;
% For all solutions found, compute sols
for i=1:m

    % Compute M for this solution
    PositionM=E+EM*[cos(sols_theta(i));sin(sols_theta(i))];
    M_int = PositionM;

    % Output P: at distance MP from M, rotated eta from ME
    v_ME = E - M_int;
    if norm(v_ME) == 0, continue; end
    v_ME_unit = v_ME / norm(v_ME);
    R_eta = [cosd(eta), -sind(eta); sind(eta), cosd(eta)];
    v_MP = R_eta * v_ME_unit;
    P = M_int + MP * v_MP;

    CM=sqrt(CD^2+DM^2-2*abs(CD)*abs(DM)*cosd(180-delta));
    epsilon=acos((DM^2-CM^2-CD^2)/(-2*CM*CD));
    x=PositionM(1);
    y=PositionM(2);
    a=OC;
    e=CM;
    L=2*a*x;
    M=2*a*y;
    N=e^2-x^2-y^2-a^2;

    if abs(L)>1e-9
        thetaO1=atan(M/L)+acos(-N/sqrt(L^2+M^2));
        thetaO2=atan(M/L)-acos(-N/sqrt(L^2+M^2));
    else
        if abs(M)>1e-9
            thetaO1=asin(-N/M);
            thetaO2=asin(-N/M);
        else
            thetaO1=NaN;
            thetaO2=NaN;
        end
    end

    % if N^2<(L^2+M^2)
    %     if N==L
    %         if c~=0
    %             t=-N/M;
    %             thetaO1=2*atan(t);
    %             thetaO2=thetaO1;
    %         else
    %             thetaO1=pi;
    %             thetaO2=-pi;
    %         end
    %     else
    %         thetaO1=2*atan((-M+sqrt(L^2+M^2-N^2))/(N-L));
    %         thetaO2=2*atan((-M-sqrt(L^2+M^2-N^2))/(N-L));
    %     end
    % else
    %     thetaO1=NaN;
    %     thetaO2=NaN;
    % end



    if not(isnan(thetaO1))&&(isreal(thetaO1))

    C_int1=OC*[cos(thetaO1);sin(thetaO1)];
    R_epsilon = [cos(-epsilon), -sin(-epsilon); sin(-epsilon), cos(-epsilon)];
    D_int1=O+C_int1+CD*R_epsilon*(M_int-C_int1)./CM;
    C_int2=OC*[cos(thetaO2);sin(thetaO2)];
    D_int2=O+C_int2+CD*R_epsilon*(M_int-C_int2)./CM;

    %if abs(norm(D_int1-A)-DA)<0.1
    C_int=C_int1;    
    D_int=D_int1;
        
        % Fill angles
        Angles.thetaC   = rel_angle(D_int - C_int, C_int - O);
        Angles.thetaD   = rel_angle(A - D_int, D_int - C_int);
        Angles.thetaE   = rel_angle(M_int - E, E - B);
        Angles.thetaA   = rel_angle(B - A, A - O);
        Angles.thetaB   = rel_angle(E - B, B - O);
        Angles.thetaM   = rel_angle(P - M_int, M_int - E);
        Angles.thetaO   = rel_angle(C_int - O, A - O);
        Angles.theta_CM = vec_angle_xaxis(M_int - C_int);
        Angles.theta_MP = vec_angle_xaxis(P - M_int);

        v_EM_plot = M_int - E;
        theta_EM = vec_angle_xaxis(v_EM_plot);

        thetaO_int=atan2(C_int(2),C_int(1));

        % Fill structure
        sol.Positions.O = O;
        sol.Positions.A = A;
        sol.Positions.B = B;
        sol.Positions.C = C_int;
        sol.Positions.D = D_int;
        sol.Positions.E = E;
        sol.Positions.M = M_int;
        sol.Positions.P = P;
        sol.Angles = Angles;
        sol.theta_EM = theta_EM;
        sol.valid = 1;
        sol.thetaO = rad2deg(thetaO_int);
        sol.thetaB = thetaB;
        sols = [sols; sol];
    %end

    %if abs(norm(D_int2-A)-DA)<0.1
        C_int=C_int2;
    D_int=D_int2;
        % Fill angles
        Angles.thetaC   = rel_angle(D_int - C_int, C_int - O);
        Angles.thetaD   = rel_angle(A - D_int, D_int - C_int);
        Angles.thetaE   = rel_angle(M_int - E, E - B);
        Angles.thetaA   = rel_angle(B - A, A - O);
        Angles.thetaB   = rel_angle(E - B, B - O);
        Angles.thetaM   = rel_angle(P - M_int, M_int - E);
        Angles.thetaO   = rel_angle(C_int - O, A - O);
        Angles.theta_CM = vec_angle_xaxis(M_int - C_int);
        Angles.theta_MP = vec_angle_xaxis(P - M_int);

        v_EM_plot = M_int - E;
        theta_EM = vec_angle_xaxis(v_EM_plot);

        thetaO_int=atan2(C_int(2),C_int(1));

        % Fill structure
        sol.Positions.O = O;
        sol.Positions.A = A;
        sol.Positions.B = B;
        sol.Positions.C = C_int;
        sol.Positions.D = D_int;
        sol.Positions.E = E;
        sol.Positions.M = M_int;
        sol.Positions.P = P;
        sol.Angles = Angles;
        sol.theta_EM = theta_EM;
        sol.valid = 1;
        sol.thetaO = rad2deg(thetaO_int);
        sol.thetaB = thetaB;
        sols = [sols; sol];
    end



    %thetaA11=-atan2(OC*sin(thetaO1),OA-OC*cos(thetaO1))+acos((CD^2-OC^2-DA^2-OA^2+2*OC*OA*cos(thetaO1))/(2*DA*sqrt(OC^2+OA^2-2*OC*OA*cos(thetaO1))));
    %thetaA12=-atan2(OC*sin(thetaO1),OA-OC*cos(thetaO1))-acos((CD^2-OC^2-DA^2-OA^2+2*OC*OA*cos(thetaO1))/(2*DA*sqrt(OC^2+OA^2-2*OC*OA*cos(thetaO1))));
    %thetaA21=-atan2(OC*sin(thetaO2),OA-OC*cos(thetaO2))+acos((CD^2-OC^2-DA^2-OA^2+2*OC*OA*cos(thetaO2))/(2*DA*sqrt(OC^2+OA^2-2*OC*OA*cos(thetaO2))));
    %thetaA22=-atan2(OC*sin(thetaO2),OA-OC*cos(thetaO2))-acos((CD^2-OC^2-DA^2-OA^2+2*OC*OA*cos(thetaO2))/(2*DA*sqrt(OC^2+OA^2-2*OC*OA*cos(thetaO2))));


%     % Compute potential positions for C and D
%     CM=sqrt(CD^2+DM^2-2*abs(CD)*abs(DM)*cosd(180-delta));
%     [C1,C2]=circle_intersections(M_int, abs(CM), O, abs(OC));
%     [D1,D2]=circle_intersections(M_int, abs(DM), A, abs(DA));
% 
%     % Error with potential length/orientation for CD and real one
%     L(1)=abs(norm(C1-D1)-abs(CD));S(1)=sign(cross([D1-C1;0],[M_int-C1;0])'*[0;0;1]);
%     L(2)=abs(norm(C1-D2)-abs(CD));S(2)=sign(cross([D2-C1;0],[M_int-C1;0])'*[0;0;1]);
%     L(3)=abs(norm(C2-D1)-abs(CD));S(3)=sign(cross([D1-C2;0],[M_int-C2;0])'*[0;0;1]);
%     L(4)=abs(norm(C2-D2)-abs(CD));S(4)=sign(cross([D2-C2;0],[M_int-C2;0])'*[0;0;1]);
%     tol=1e-6;
% 
%     L,S
% 
%     for j=1:4
% 
%         if (L(j)<tol)&&(S(j)>0)
% 
%             switch j
%                 case 1
%                     C_int=C1;D_int=D1;
%                 case 2
%                     C_int=C1;D_int=D2;
%                 case 3
%                     C_int=C2;D_int=D1;
%                 case 4
%                     C_int=C2;D_int=D2;
%             end
% 
%             % Fill angles
%             Angles.thetaC   = rel_angle(D_int - C_int, C_int - O);
%             Angles.thetaD   = rel_angle(A - D_int, D_int - C_int);
%             Angles.thetaE   = rel_angle(M_int - E, E - B);
%             Angles.thetaA   = rel_angle(B - A, A - O);
%             Angles.thetaB   = rel_angle(E - B, B - O);
%             Angles.thetaM   = rel_angle(P - M_int, M_int - E);
%             Angles.thetaO   = rel_angle(C_int - O, A - O);
%             Angles.theta_CM = vec_angle_xaxis(M_int - C_int);
%             Angles.theta_MP = vec_angle_xaxis(P - M_int);
% 
%             v_EM_plot = M_int - E;
%             theta_EM = vec_angle_xaxis(v_EM_plot);
% 
%             thetaO_int=atan2(C_int(2),C_int(1));
% 
%             % Fill structure
%             sol.Positions.O = O;
%             sol.Positions.A = A;
%             sol.Positions.B = B;
%             sol.Positions.C = C_int;
%             sol.Positions.D = D_int;
%             sol.Positions.E = E;
%             sol.Positions.M = M_int;
%             sol.Positions.P = P;
%             sol.Angles = Angles;
%             sol.theta_EM = theta_EM;
%             sol.valid = 1;
%             sol.thetaO = rad2deg(thetaO_int);
%             sols = [sols; sol];
% 
%             p=p+1;
%         end
% 
%     end
% 
end

if isempty(sols)
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
            sol.thetaO = NaN;
            sol.thetaB = thetaB;
            sols = sol;
end

%if p~=m
    % On a rate une solution ! Le probleme est dans le calcul des points
    % C1..D2 :
    % [C1,C2]=circle_intersections(M_int, abs(CM), O, abs(OC));
    % [D1,D2]=circle_intersections(M_int, abs(DM), A, abs(DA));
    % qui ne trouve pas une solution
    % par exemple : stephensonIII_inverse_kinematics(geo,143)
    % vs stephensonIII_inverse_kinematics(geo,142)
%end

end


%% ---- COUPLER CURVE
function f = fourbar_coupler_curve(delta,OA,DM,CD,OC,DA,x,y);

CM=sqrt(CD^2+DM^2-2*abs(CD)*abs(DM)*cosd(180-delta));
epsilon=acos((DM^2-CM^2-CD^2)/(-2*abs(CD*CM)));
d=abs(OA);
e=abs(CM);
b=abs(CD);
a=abs(OC);
c=abs(DA);

aa = sqrt(e^2+b^2-2*e*b*cos(epsilon));
gamma = asin(b*sin(epsilon)/aa);
beta = asin(e*sin(epsilon)/aa);
f = (sin(epsilon).*((x-d).*sin(gamma)-y.*cos(gamma)).*(x.*x+y.*y+e.*e-a.*a)+...
    y.*sin(beta).*((x-d).^2+y.*y+aa.*aa-c.*c)).^2+...
    (sin(epsilon).*((x-d).*cos(gamma)+y.*sin(gamma)).*(x.*x+y.*y+e.*e-a.*a)-...
    x.*sin(beta).*((x-d).*(x-d)+y.*y+aa.*aa-c.*c)).^2-...
    4*e.^2.*sin(epsilon).^2.*sin(gamma).^2.*(x.*(x-d)+y.*y-d.*y.*cot(gamma)).^2;
end

%% ---- COUPLER CURVE
function f = fourbar_coupler_curve_wrapper(theta,delta,OA,DM,CD,OC,DA,E,EM);
p=E+EM*[cos(theta);sin(theta)];
x=p(1);
y=p(2);

f = fourbar_coupler_curve(delta,OA,DM,CD,OC,DA,x,y);
end


%% ---- HELPERS ----

function [p1, p2] = circle_intersections(c1, r1, c2, r2)
d = norm(c2-c1);
if d > r1+r2 || d - abs(r1-r2) < 0
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

function [D1, D2] = possible_D_from_M(M, DM)
% Returns the two points at distance DM from M (unit circle)
D1 = M + DM*[cos(0); sin(0)]; % arbitrary direction, but needs proper calculation
D2 = M + DM*[cos(pi); sin(pi)];
% Actually, for general case, you might have an extra constraint—delta, or relative to C, etc.
% If you know direction to E or C, or use the vector from M to E/C and rotate by delta...
end

function theta = rel_angle(v2, v1)
theta = atan2d(v2(2), v2(1)) - atan2d(v1(2), v1(1));
theta = mod(theta + 360, 360);
end

function theta = vec_angle_xaxis(v)
theta = atan2d(v(2), v(1));
theta = mod(theta + 360, 360);
end