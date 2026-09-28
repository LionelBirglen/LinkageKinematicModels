function [ax, sol, kin, constr] = Q2_leg_mechanism_plot(params, mode, inputs, opts)
%Q2_LEG_MECHANISM_PLOT  Plot the planar leg mechanism
%
%   [AX,SOL,KIN] = Q2_LEG_MECHANISM_PLOT(PARAMS, MODE, INPUTS)
%   [AX,SOL,KIN] = Q2_LEG_MECHANISM_PLOT(PARAMS, MODE, INPUTS, OPTS)
%   [AX,SOL,KIN,CONSTR] = Q2_LEG_MECHANISM_PLOT(...)
%
%   PARAMS : struct with the mechanism geometry (lengths in mm, angles in
%            rad), as used by Q2_leg_mechanism_direct_kinematics:
%              .AB .AC .BD .CD .FG .CF .BE .GK .GJ .IF .IJ .IP
%              .DCF .GFC .DBE .JGK .JIP
%            (17 independent parameters, see Q2_leg_mechanism_direct_kinematics;
%            P is the point of body I-J-P defined by IP and JIP)
%            Ground pivots are A = (0,0) and B = (AB,0).
%
%   MODE   : 'direct'  -> INPUTS = [thetaA, rho]   (rad, mm)
%            'inverse' -> INPUTS = [xI, yI]        (mm, target for point I)
%
%   OPTS   : optional struct, fields:
%       .ax         - axes to plot in (GUI passes its axes handle)
%       .solutions  - scalar or vector of solution indices to draw
%                     (default: draw all valid ones; indices that match
%                     no solution, e.g. 0, draw no linkage)
%       .showLabels - true/false (default: true)
%       .clearAxes  - true/false (default: true)
%       .limits     - [xmin xmax ymin ymax] to enforce fixed limits
%       .colors     - Nx3 colormap, one row per solution index
%                     (default: red, blue, then six more distinct colors)
%       .showConstruction - true/false (default: false). Draws the line
%                     intersections M = (AC) x (BD) and N = (IF) x (GJ)
%                     as black "x" marks, with fine lines through
%                     A-C-M, B-D-M, I-F-N and G-J-N, drawn on top of
%                     the bodies in .constructionColor. If a pair of
%                     lines is parallel, that point and its lines are
%                     not drawn.
%       .constructionColor - RGB of the construction lines, the same for
%                     all solutions (default: [0 0 0], black)
%       .workspace  - workspace struct from Q2_leg_mechanism_workspace. If
%                     given (and not empty), the reachable region of P is
%                     drawn under the mechanism, one semi-transparent
%                     area per branch in that branch's color.
%       .workspaceSolutions - branches whose workspace is drawn
%                     (default: .solutions if given, else all 8)
%       .workspaceStyle - 'fill' (default), 'points' or 'both':
%                     'fill'   : filled grid cells only,
%                     'points' : all sampled positions as dots,
%                     'both'   : filled cells, plus dots only for the
%                                isolated positions not covered by any
%                                filled cell (thin regions)
%       .workspaceAlpha - lightness of the areas, 0..1 (default 0.15):
%                     the branch color blended with the axes background
%                     by this factor. Areas are drawn OPAQUE in that
%                     light color, so overlapping cells (and folds of
%                     the workspace) keep one uniform color instead of
%                     getting darker. The axes grid is kept on top.
%
%   AX     : axes handle used for plotting
%   SOL    : 1x8 solution struct array returned by Q2_leg_mechanism_direct_kinematics
%            (fields .Positions.A..I, .P, .phi, .valid, ...)
%   KIN    : struct with the actuated inputs actually used:
%              .thetaA, .rho      (from INPUTS in direct mode, from the
%                                  numerical IK in inverse mode)
%              .ik_error          (NaN in direct mode)
%              .target            ([NaN;NaN] in direct mode)
%   CONSTR : struct array, one element per solution of SOL, with the
%            line intersections of each DRAWN valid solution (NaN for
%            the others, and whenever the lines are parallel):
%              .M   - M = (AC) x (BD)
%              .N   - N = (IF) x (GJ)
%              .dMN - distance |MN|
%            Computed whether or not .showConstruction is set.
%
%   Drawing conventions (identical to fourbar_plot):
%     - one uniform color per solution branch,
%     - rigid bodies with 3+ points (B-D-E, C-D-G-F, G-K-J, I-J-P) as
%       semi-transparent patches, binary links as lines,
%     - point P as a black "x",
%     - revolute joints as white-filled black circles,
%     - ground symbols at the fixed pivots A and B,
%     - joint letters offset up-right by a fixed fraction of AC.
%   The prismatic actuator E-K is drawn as a dash-dot line.
%   In inverse mode, the target for I is marked with a black "+".
%   Label strings of M and N are set in section 0 below.
%
%   Example (direct):
%       p = struct('AB',150,'AC',90,'BD',120,'CD',100,'FG',80,'CF',90, ...
%                  'BE',50,'GK',50,'GJ',80,'IF',80,'IJ',100,'IP',50, ...
%                  'DCF',deg2rad(80),'GFC',deg2rad(110),'DBE',deg2rad(45), ...
%                  'JGK',deg2rad(30),'JIP',deg2rad(30));
%       Q2_leg_mechanism_plot(p,'direct',[deg2rad(-80) 220]);
%
%   Example (inverse, plot only sol 2):
%       Q2_leg_mechanism_plot(p,'inverse',[0 -250],struct('solutions',2));
%

if nargin < 4, opts = struct(); end
if nargin < 3
    error('Q2_leg_mechanism_plot:NotEnoughInputs', ...
        'Need at least: params, mode, inputs.');
end

% --- 0) Octave vs MATLAB rendering scale factors -----------------
% Same values as fourbar_plot so both tools look identical.
isOctave = exist('OCTAVE_VERSION','builtin') ~= 0;
if isOctave
    lw_link  = 0.6;   % link and x-marker line width
    lw_joint = 0.4;   % joint circle edge line width
    lw_ground_hatch = 0.4;  % ground symbol hatch lines
    lw_ground_base  = 0.3;  % ground symbol baseline
    ms_joint = 6;     % scatter marker size for joints
    ms_P     = 2;     % MarkerSize for the x/+ markers
    fs_label = 14;    % joint label font size
    lw_fine  = 0.25;  % construction lines
else
    lw_link  = 1.5;
    lw_joint = 1.0;
    lw_ground_hatch = 1.5;
    lw_ground_base  = 1.0;
    ms_joint = 40;
    ms_P     = 8;
    fs_label = 10;
    lw_fine  = 0.5;
end

% Labels of the constructed intersection points
lbl_ACBD = 'M';     % intersection of lines AC and BD
lbl_IFGJ = 'N';     % intersection of lines IF and GJ
showConstr = isfield(opts,'showConstruction') && ~isempty(opts.showConstruction) ...
             && opts.showConstruction;
% Color of the construction lines (same for all solutions, distinct
% from the per-solution linkage colors)
if isfield(opts,'constructionColor') && ~isempty(opts.constructionColor)
    constrColor = opts.constructionColor;
else
    constrColor = [0 0 0];   % black
end

% --- 1) geometry --------------------------------------------------
local_check_params(params);

% Ground symbol length: fixed fraction of the longest link of the base
% four-bar A-C-D-B (same rule as fourbar_plot), never changes with pose
gs_len = max([params.AB params.AC params.BD params.CD]) * 0.1125;

% Fixed pivots
A = [0; 0];
B = [params.AB; 0];

% --- 2) normalize mode & inputs -----------------------------------
mode = lower(char(mode));
if ~isnumeric(inputs) || numel(inputs) ~= 2
    error('Q2_leg_mechanism_plot:BadInputs', ...
        'INPUTS must be a 2-element numeric vector.');
end
inputs = inputs(:);

% --- 3) prepare axes ----------------------------------------------
if isfield(opts,'ax') && ~isempty(opts.ax) && ishghandle(opts.ax)
    ax = opts.ax;
else
    fig = figure('Name','Q2_leg_mechanism_plot','NumberTitle','off');
    ax  = axes('Parent',fig, 'Units','normalized', ...
        'Position',[0.12 0.10 0.78 0.80]);
    axis(ax,'equal');
    grid(ax,'on');
    xlabel(ax,'X [mm]');
    ylabel(ax,'Y [mm]');
    title(ax,'Q2_Leg Mechanism');
end

if ~isfield(opts,'clearAxes') || opts.clearAxes
    cla(ax);
end
hold(ax,'on');   % before any drawing, so axes properties are kept

% --- 4) colors ----------------------------------------------------
if isfield(opts,'colors') && ~isempty(opts.colors)
    colors = opts.colors;
else
    colors = Q2_leg_mechanism_colors();
end

% --- 4b) workspace of P (drawn first, under everything) -----------
if isfield(opts,'workspace') && ~isempty(opts.workspace) && ...
        isstruct(opts.workspace) && isfield(opts.workspace,'patch')
    W = opts.workspace;
    if isfield(opts,'workspaceSolutions') && ~isempty(opts.workspaceSolutions)
        wsIdx = opts.workspaceSolutions(:).';
    elseif isfield(opts,'solutions') && ~isempty(opts.solutions)
        wsIdx = opts.solutions(:).';
    else
        wsIdx = 1:numel(W.patch);
    end
    wsIdx = wsIdx(wsIdx >= 1 & wsIdx <= numel(W.patch));
    if isfield(opts,'workspaceStyle') && ~isempty(opts.workspaceStyle)
        wsStyle = lower(char(opts.workspaceStyle));
    else
        wsStyle = 'fill';
    end
    if isfield(opts,'workspaceAlpha') && ~isempty(opts.workspaceAlpha)
        wsAlpha = opts.workspaceAlpha;
    else
        wsAlpha = 0.15;
    end
    % Background the light colors are blended with
    bg = get(ax,'Color');
    if ~isnumeric(bg) || numel(bg) ~= 3, bg = [1 1 1]; end
    % Opaque areas would hide the grid: draw axes lines on top
    set(ax, 'Layer', 'top');

    for ii = wsIdx
        colW = colors( mod(ii-1, size(colors,1)) + 1 , : );
        colL = (1 - wsAlpha) * bg(:).' + wsAlpha * colW;   % light, opaque
        pd = W.patch(ii);
        doFill = any(strcmp(wsStyle, {'fill','both'})) && ~isempty(pd.Faces);
        if doFill
            % Opaque faces with edges in the same color: no darkening
            % where cells overlap and no hairline seams between cells
            patch('Vertices', pd.Vertices, 'Faces', pd.Faces, ...
                'FaceColor', colL, 'EdgeColor', colL, ...
                'FaceAlpha', 1, 'Parent', ax);
        end
        if any(strcmp(wsStyle, {'points','both'}))
            v = W.valid(:,:,ii);
            if strcmp(wsStyle, 'both') && doFill
                % keep only nodes that are not a corner of a filled cell
                v(pd.Faces(:)) = false;
            end
            if any(v(:))
                Xw = W.X(:,:,ii);  Yw = W.Y(:,:,ii);
                plot(ax, Xw(v), Yw(v), '.', 'Color', colL, 'MarkerSize', 6);
            end
        end
    end
end

% --- 5) kinematics ------------------------------------------------
kin = struct('thetaA',NaN,'rho',NaN,'ik_error',NaN,'target',[NaN;NaN]);
switch mode
    case {'direct','d'}
        kin.thetaA = inputs(1);
        kin.rho    = inputs(2);

    case {'inverse','i'}
        if ~exist('Q2_leg_inverse_kinematics_numeric','file')
            error('Q2_leg_mechanism_plot:MissingFile', ...
                'Q2_leg_inverse_kinematics_numeric.m is not on the path.');
        end
        kin.target = inputs;
        [kin.thetaA, kin.rho, kin.ik_error] = ...
            Q2_leg_inverse_kinematics_numeric(inputs, params);

    otherwise
        error('Q2_leg_mechanism_plot:BadMode', ...
            'MODE must either be ''direct'' or ''inverse''.');
end

if ~exist('Q2_leg_mechanism_direct_kinematics','file')
    error('Q2_leg_mechanism_plot:MissingFile', ...
        'Q2_leg_mechanism_direct_kinematics.m is not on the path.');
end
sol = Q2_leg_mechanism_direct_kinematics(kin.thetaA, kin.rho, params);

% Line intersections M and N per solution (filled in the drawing loop)
constr = repmat(struct('M',[NaN;NaN],'N',[NaN;NaN],'dMN',NaN), 1, numel(sol));

% If no solution, stop but keep style
if isempty(sol) || (isfield(sol,'valid') && ~any([sol.valid]))
    axis(ax,'equal'); grid(ax,'on');
    local_apply_limits(ax, opts);
    return;
end

% --- 6) what to draw? ---------------------------------------------
nSol = numel(sol);
if isfield(opts,'solutions') && ~isempty(opts.solutions)
    idxToPlot = opts.solutions(:).';
else
    idxToPlot = 1:nSol;
end
idxToPlot = idxToPlot(idxToPlot>=1 & idxToPlot<=nSol);

% --- 7) ground symbols (fixed pivots A and B) ---------------------
drawGroundSymbol(ax, A, lw_ground_hatch, lw_ground_base, gs_len);
drawGroundSymbol(ax, B, lw_ground_hatch, lw_ground_base, gs_len);

% --- 8) draw each selected solution -------------------------------
for k = 1:numel(idxToPlot)
    ii = idxToPlot(k);
    s  = sol(ii);

    if isfield(s,'valid') && ~s.valid
        continue;
    end
    [pts, ok] = local_get_points(s);
    if ~ok
        continue;   % unassembled / NaN branch
    end
    C = pts.C;  D = pts.D;  E = pts.E;  F = pts.F;
    G = pts.G;  K = pts.K;  J = pts.J;  I = pts.I;
    hasP = isfield(s,'P') && numel(s.P) >= 2 && all(isfinite(s.P(1:2)));
    if hasP, P = s.P(:); end

    % one color per solution index (stable across redraws)
    col = colors( mod(ii-1, size(colors,1)) + 1 , : );

    % --- quaternary body C-D-G-F, semi-transparent ----------------
    patch('XData', [C(1) D(1) G(1) F(1)], ...
        'YData', [C(2) D(2) G(2) F(2)], ...
        'FaceColor', col, 'EdgeColor', col, 'FaceAlpha', 0.5, ...
        'Parent', ax);

    % --- ternary body B-D-E (pivoted on ground at B) --------------
    patch('XData', [B(1) D(1) E(1)], ...
        'YData', [B(2) D(2) E(2)], ...
        'FaceColor', col, 'EdgeColor', col, 'FaceAlpha', 0.5, ...
        'Parent', ax);

    % --- ternary body G-K-J ---------------------------------------
    patch('XData', [G(1) K(1) J(1)], ...
        'YData', [G(2) K(2) J(2)], ...
        'FaceColor', col, 'EdgeColor', col, 'FaceAlpha', 0.5, ...
        'Parent', ax);

    % --- body I-J: ternary I-J-P if P is defined, else binary -----
    if hasP
        patch('XData', [I(1) J(1) P(1)], ...
            'YData', [I(2) J(2) P(2)], ...
            'FaceColor', col, 'EdgeColor', col, 'FaceAlpha', 0.5, ...
            'Parent', ax);
    else
        plot(ax, [I(1) J(1)], [I(2) J(2)], ...
            '-', 'Color', col, 'LineWidth', lw_link);
    end

    % --- binary links: input crank A-C, and F-I ----------------------
    plot(ax, [A(1) C(1)], [A(2) C(2)], ...
        '-', 'Color', col, 'LineWidth', lw_link);
    plot(ax, [F(1) I(1)], [F(2) I(2)], ...
        '-', 'Color', col, 'LineWidth', lw_link);

    % --- prismatic actuator E-K -----------------------------------
    plot(ax, [E(1) K(1)], [E(2) K(2)], ...
        '-.', 'Color', col, 'LineWidth', lw_link);

    % --- point P (black "x") ---------------------------------------
    if hasP
        plot(ax, P(1), P(2), 'kx', 'MarkerSize', ms_P, 'LineWidth', lw_link);
    end

    % --- construction: fine lines through A-C-M, B-D-M, I-F-N and
    %     G-J-N, in a color distinct from the linkage and drawn on top
    %     of the bodies so they stay visible -------------------------
    Mc = local_line_intersection(A, C, B, D);
    Nc = local_line_intersection(I, F, G, J);
    constr(ii).M   = Mc;
    constr(ii).N   = Nc;
    constr(ii).dMN = norm(Nc - Mc);   % NaN if either is at infinity
    if showConstr
        if all(isfinite(Mc))
            local_draw_collinear(ax, [A C Mc], constrColor, lw_fine);
            local_draw_collinear(ax, [B D Mc], constrColor, lw_fine);
        end
        if all(isfinite(Nc))
            local_draw_collinear(ax, [I F Nc], constrColor, lw_fine);
            local_draw_collinear(ax, [G J Nc], constrColor, lw_fine);
        end
    end

    % --- constructed points M and N (black "x") -------------------
    if showConstr
        if all(isfinite(Mc))
            plot(ax, Mc(1), Mc(2), 'kx', 'MarkerSize', ms_P, 'LineWidth', lw_link);
        end
        if all(isfinite(Nc))
            plot(ax, Nc(1), Nc(2), 'kx', 'MarkerSize', ms_P, 'LineWidth', lw_link);
        end
    end

    % --- joints as filled white circles ---------------------------
    Jx = [A(1) B(1) C(1) D(1) E(1) F(1) G(1) K(1) J(1) I(1)];
    Jy = [A(2) B(2) C(2) D(2) E(2) F(2) G(2) K(2) J(2) I(2)];
    scatter(ax, Jx, Jy, ms_joint, ...
        'MarkerFaceColor','w', 'MarkerEdgeColor','k', ...
        'LineWidth', lw_joint);  % Octave-compatible

    % --- optional labels ------------------------------------------
    if ~isfield(opts,'showLabels') || opts.showLabels
        local_label_joints(ax, A, B, pts, params.AC/20, fs_label);
        if hasP
            text(ax, P(1)+params.AC/20, P(2)+params.AC/20, 'P', ...
                'FontSize',fs_label, 'Color','k', ...
                'HorizontalAlignment','left', ...
                'VerticalAlignment','bottom');
        end
        if showConstr
            dxl = params.AC/20;
            if all(isfinite(Mc))
                text(ax, Mc(1)+dxl, Mc(2)+dxl, lbl_ACBD, ...
                    'FontSize',fs_label, 'Color','k', ...
                    'HorizontalAlignment','left', ...
                    'VerticalAlignment','bottom');
            end
            if all(isfinite(Nc))
                text(ax, Nc(1)+dxl, Nc(2)+dxl, lbl_IFGJ, ...
                    'FontSize',fs_label, 'Color','k', ...
                    'HorizontalAlignment','left', ...
                    'VerticalAlignment','bottom');
            end
        end
    end
end

% --- 9) inverse mode: mark the target for I -----------------------
if all(isfinite(kin.target))
    plot(ax, kin.target(1), kin.target(2), 'k+', ...
        'MarkerSize', ms_P, 'LineWidth', lw_link);
end

% --- 10) axis housekeeping ----------------------------------------
axis(ax,'equal');
grid(ax,'on');
local_apply_limits(ax, opts);
end


% ======================================================================
%  local helpers
% ======================================================================
function colors = Q2_leg_mechanism_colors()
% Default per-solution colors: first two match fourbar_plot (red, blue)
colors = [1    0    0   ;   % red
          0    0    1   ;   % blue
          0    0.6  0   ;   % green
          0.9  0.5  0   ;   % orange
          0.55 0    0.75;   % purple
          0    0.6  0.6 ;   % teal
          0.45 0.45 0.45;   % grey
          0.8  0    0.45];  % crimson
end


function local_check_params(p)
if ~isstruct(p)
    error('Q2_leg_mechanism_plot:BadParams', 'PARAMS must be a struct.');
end
must = {'AB','AC','BD','CD','FG','CF','BE','GK','GJ','IF','IJ','IP', ...
        'DCF','GFC','DBE','JGK','JIP'};
for k = 1:numel(must)
    if ~isfield(p, must{k})
        error('Q2_leg_mechanism_plot:BadParams', ...
            'Missing field "%s" in parameter struct.', must{k});
    end
end
end


function [pts, ok] = local_get_points(s)
% Joint positions from s.Positions (fourbar-style output). Falls back to
% top-level fields for the legacy leg_direct_kinematics output.
names = {'C','D','E','F','G','K','J','I'};
pts = struct();
ok = true;
if isfield(s,'Positions'), src = s.Positions; else, src = s; end
for k = 1:numel(names)
    if ~isfield(src, names{k})
        ok = false; return;
    end
    v = src.(names{k});
    v = v(:);
    if numel(v) < 2 || any(~isfinite(v(1:2)))
        ok = false; return;
    end
    pts.(names{k}) = v(1:2);
end
end


function X = local_line_intersection(P1, P2, P3, P4)
% Intersection of line (P1,P2) with line (P3,P4); [NaN;NaN] if the
% lines are parallel (or a line is degenerate).
d1 = P2 - P1;  d2 = P4 - P3;
den = d1(1)*d2(2) - d1(2)*d2(1);
if abs(den) <= 1e-12 * norm(d1) * norm(d2) || norm(d1) == 0 || norm(d2) == 0
    X = [NaN; NaN];
    return;
end
w = P3 - P1;
t = (w(1)*d2(2) - w(2)*d2(1)) / den;
X = P1 + t * d1;
end


function local_draw_collinear(ax, pts, col, lw)
% Fine line through collinear points (columns of pts), drawn between the
% two extreme points so that it passes through all of them.
d = pts(:,2) - pts(:,1);
if norm(d) == 0, d = pts(:,3) - pts(:,1); end
if norm(d) == 0, return; end
d = d / norm(d);
t = d.' * (pts - pts(:,1) * ones(1, size(pts,2)));
p0 = pts(:,1) + min(t) * d;
p1 = pts(:,1) + max(t) * d;
plot(ax, [p0(1) p1(1)], [p0(2) p1(2)], '-', 'Color', col, 'LineWidth', lw);
end


function local_apply_limits(ax, opts)
if isfield(opts,'limits') && numel(opts.limits)==4
    xlim(ax, opts.limits(1:2));
    ylim(ax, opts.limits(3:4));
end
end


function local_label_joints(ax, A, B, pts, dx, fs_label)
% Same offset and text style as fourbar_plot's local_label_joint
dy = dx;
P   = {A, B, pts.C, pts.D, pts.E, pts.F, pts.G, pts.K, pts.J, pts.I};
lab = {'A','B','C','D','E','F','G','K','J','I'};
for k = 1:numel(P)
    text(ax, P{k}(1)+dx, P{k}(2)+dy, lab{k}, ...
        'FontSize',fs_label, 'Color','k', ...
        'HorizontalAlignment','left', ...
        'VerticalAlignment','bottom');
end
end


function drawGroundSymbol(ax, jointCenter, lw_ground_hatch, lw_ground_base, lineLength)
% Identical to the one in fourbar_plot
jointCenter = jointCenter(:);
numLines = 3;
lineSpacing = lineLength/2;
angle = +pi/4;
R = [cos(angle+pi) -sin(angle+pi); sin(angle+pi) cos(angle+pi)];
x = [lineLength; 0];
Rx = R*x;
for i = 0:numLines-1
    v1 = jointCenter + [i*lineSpacing-(numLines-1)/2*lineSpacing; 0];
    v2 = v1 + Rx;
    plot(ax, [v1(1) v2(1)], [v1(2) v2(2)], 'k', 'LineWidth', lw_ground_hatch);
end
plot(ax, jointCenter(1)+[-(numLines-1)/2*lineSpacing +(numLines-1)/2*lineSpacing], ...
    [jointCenter(2) jointCenter(2)], 'k', 'LineWidth', lw_ground_base);
end
