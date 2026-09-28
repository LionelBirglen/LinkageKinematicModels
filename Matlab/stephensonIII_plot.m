function [ax, sol] = stephensonIII_plot(geo, mode, inputs, opts)
%STEPHENSONIII_PLOT  Plot a Stephenson III six-bar linkage
%
%   [AX,SOL] = STEPHENSONIII_PLOT(GEO, MODE, INPUTS)
%   [AX,SOL] = STEPHENSONIII_PLOT(GEO, MODE, INPUTS, OPTS)
%
%   GEO   : 1x12 numeric vector, as used by the kinematics functions:
%           [OA, Bx, By, OC, CD, DA, BF, FE, DE, EP, eta, delta]
%           (lengths in consistent units, eta and delta in DEGREES)
%           Ground pivots: O = (0,0), A = (OA,0), B = (Bx,By).
%
%   MODE  : 'direct'  -> INPUTS = thetaO (deg), angle of the crank O->C
%           'inverse' -> INPUTS = thetaB (deg), angle of B->F w.r.t. B->O
%
%   INPUTS: numeric scalar (deg)
%
%   OPTS  : optional struct, fields (same as fourbar_plot):
%       .ax         - axes to plot in (the GUI passes its axes handle)
%       .solutions  - scalar or vector of solution indices to draw
%                     (default: draw all valid ones)
%       .showLabels - true/false (default: true)
%       .clearAxes  - true/false (default: true)
%       .limits     - [xmin xmax ymin ymax] to enforce fixed limits
%       .colors     - Nx3 colormap, one row per solution index
%                     (default: red, blue, then four more distinct colors)
%       .track      - solution tracking. If this field is present, SOL is
%                     returned in a fixed number of SLOTS (.nSlots) and
%                     the solutions are reordered so that each one keeps
%                     the slot of the configuration it continues from in
%                     .track (the SOL returned by the previous call):
%                     the assignment minimizes the total displacement of
%                     the joints C, D, F, E and P between the two calls.
%                     A slot whose solution has vanished is kept with
%                     .valid = 0; a new solution takes the first free
%                     slot. Pass [] on the first call (or to restart
%                     numbering), then the previous SOL on each call.
%                     Without .track, SOL is returned as computed.
%       .nSlots     - number of slots with .track (default 6: at most 4
%                     solutions in direct mode, 6 in inverse mode)
%
%   AX    : axes handle used for plotting
%   SOL   : solution struct array returned by the kinematics call
%           (1 x nSlots, reordered, when .track is given)
%
%   Drawing conventions (identical to fourbar_plot):
%     - one uniform color per solution index,
%     - ternary bodies C-D-E and F-E-P as semi-transparent patches,
%       binary links O-C, A-D and B-F as lines,
%     - revolute joints (O, A, B, C, D, F, E) as white-filled black
%       circles, the output point P as a black "x",
%     - ground symbols at the fixed pivots O, A and B,
%     - joint letters offset up-right by a fixed fraction of OC.
%
%   Example (direct):
%       g = [40, 70, 30, 50, 20, 50, 30, 30, -30, 20, 30, 60];
%       stephensonIII_plot(g,'direct',90);
%
%   Example (inverse, plot only solution 2):
%       stephensonIII_plot(g,'inverse',69,struct('solutions',2));
%

if nargin < 4, opts = struct(); end
if nargin < 3
    error('stephensonIII_plot:NotEnoughInputs', ...
        'Need at least: geo, mode, inputs.');
end

% --- 0) Octave vs MATLAB rendering scale factors -----------------
% Same values as fourbar_plot so both tools look identical.
isOctave = exist('OCTAVE_VERSION','builtin') ~= 0;
if isOctave
    lw_link  = 0.6;   % link and P-marker line width
    lw_joint = 0.4;   % joint circle edge line width
    lw_ground_hatch = 0.4;  % ground symbol hatch lines
    lw_ground_base  = 0.3;  % ground symbol baseline
    ms_joint = 6;     % scatter marker size for joints
    ms_P     = 2;     % MarkerSize for the X at P
    fs_label = 14;    % joint label font size
else
    lw_link  = 1.5;
    lw_joint = 1.0;
    lw_ground_hatch = 1.5;
    lw_ground_base  = 1.0;
    ms_joint = 40;
    ms_P     = 8;
    fs_label = 10;
end

% --- 1) geometry --------------------------------------------------
if ~isnumeric(geo) || numel(geo) ~= 12
    error('stephensonIII_plot:BadGeo', ...
        ['GEO must have 12 elements: ' ...
         '[OA Bx By OC CD DA BF FE DE EP eta delta].']);
end
g = geo(:).';

% Fixed pivots (taken from the geometry, so they are drawn even when
% no configuration assembles)
O = [0; 0];
A = [g(1); 0];
B = [g(2); g(3)];

% Ground symbol length: fixed fraction of the longest link (same rule
% as fourbar_plot), never changes with pose
gs_len = max(abs([g(1) norm(B) g(4:10)])) * 0.1125;

% Label offset: fixed fraction of the input crank OC (fourbar_plot
% uses a/20)
dxl = abs(g(4)) / 20;

% --- 2) normalize mode & inputs -----------------------------------
mode = lower(char(mode));
if ~isnumeric(inputs) || numel(inputs) ~= 1
    error('stephensonIII_plot:BadInputs', ...
        'INPUT must be a numeric scalar (deg).');
end

% --- 3) prepare axes ----------------------------------------------
if isfield(opts,'ax') && ~isempty(opts.ax) && ishghandle(opts.ax)
    ax = opts.ax;
else
    fig = figure('Name','stephensonIII_plot','NumberTitle','off');
    ax  = axes('Parent',fig, 'Units','normalized', ...
        'Position',[0.12 0.10 0.78 0.80]);
    axis(ax,'equal');
    grid(ax,'on');
    xlabel(ax,'X');
    ylabel(ax,'Y');
    title(ax,'Stephenson III Linkage');
end

if ~isfield(opts,'clearAxes') || opts.clearAxes
    cla(ax);
end
hold(ax,'on');   % before any drawing, so axes properties are kept

% --- 4) colors ----------------------------------------------------
if isfield(opts,'colors') && ~isempty(opts.colors)
    colors = opts.colors;
else
    colors = stephensonIII_colors();
end

% --- 5) call the existing kinematics functions --------------------
switch mode
    case {'direct','d'}
        if ~exist('stephensonIII_direct_kinematics','file')
            error('stephensonIII_plot:MissingFile', ...
                'stephensonIII_direct_kinematics.m is not on the path.');
        end
        sol = stephensonIII_direct_kinematics(g, inputs);

    case {'inverse','i'}
        if ~exist('stephensonIII_inverse_kinematics','file')
            error('stephensonIII_plot:MissingFile', ...
                'stephensonIII_inverse_kinematics.m is not on the path.');
        end
        sol = stephensonIII_inverse_kinematics(g, inputs);

    otherwise
        error('stephensonIII_plot:BadMode', ...
            'MODE must either be ''direct'' or ''inverse''.');
end

% --- 5b) solution tracking (keeps each branch in its slot) --------
if isfield(opts,'track')
    if isfield(opts,'nSlots') && ~isempty(opts.nSlots)
        nSlots = opts.nSlots;
    else
        nSlots = 6;
    end
    sol = local_track(sol, opts.track, nSlots);
end

% --- 6) ground symbols (always drawn) -----------------------------
drawGroundSymbol(ax, O, lw_ground_hatch, lw_ground_base, gs_len);
drawGroundSymbol(ax, A, lw_ground_hatch, lw_ground_base, gs_len);
drawGroundSymbol(ax, B, lw_ground_hatch, lw_ground_base, gs_len);

% If no solution, stop but keep style
if isempty(sol) || (isfield(sol,'valid') && ~any([sol.valid]))
    local_axis_housekeeping(ax, opts);
    return;
end

% --- 7) what to draw? ---------------------------------------------
nSol = numel(sol);
if isfield(opts,'solutions') && ~isempty(opts.solutions)
    idxToPlot = opts.solutions(:).';
else
    idxToPlot = 1:nSol;
end
idxToPlot = idxToPlot(idxToPlot>=1 & idxToPlot<=nSol);

% --- 8) draw each selected solution -------------------------------
for k = 1:numel(idxToPlot)
    ii = idxToPlot(k);
    s  = sol(ii);

    if isfield(s,'valid') && ~s.valid
        continue;
    end
    pos = s.Positions;
    C = pos.C;  D = pos.D;  F = pos.F;  E = pos.E;  P = pos.P;
    if any(~isfinite([C; D; F; E; P]))
        continue;
    end

    % one color per solution index
    col = colors( mod(ii-1, size(colors,1)) + 1 , : );

    % --- ternary body C-D-E, semi-transparent ---------------------
    patch('XData', [C(1) D(1) E(1)], ...
        'YData', [C(2) D(2) E(2)], ...
        'FaceColor', col, 'EdgeColor', col, 'FaceAlpha', 0.5, ...
        'Parent', ax);

    % --- ternary body F-E-P, semi-transparent ---------------------
    patch('XData', [F(1) E(1) P(1)], ...
        'YData', [F(2) E(2) P(2)], ...
        'FaceColor', col, 'EdgeColor', col, 'FaceAlpha', 0.5, ...
        'Parent', ax);

    % --- binary links: input crank O-C, then A-D and B-F ----------
    plot(ax, [O(1) C(1)], [O(2) C(2)], '-', 'Color', col, 'LineWidth', lw_link);
    plot(ax, [A(1) D(1)], [A(2) D(2)], '-', 'Color', col, 'LineWidth', lw_link);
    plot(ax, [B(1) F(1)], [B(2) F(2)], '-', 'Color', col, 'LineWidth', lw_link);

    % --- point P (black "x") ---------------------------------------
    plot(ax, P(1), P(2), 'kx', 'MarkerSize', ms_P, 'LineWidth', lw_link);

    % --- joints as filled white circles ----------------------------
    Jx = [O(1) A(1) B(1) C(1) D(1) F(1) E(1)];
    Jy = [O(2) A(2) B(2) C(2) D(2) F(2) E(2)];
    scatter(ax, Jx, Jy, ms_joint, ...
        'MarkerFaceColor','w', 'MarkerEdgeColor','k', ...
        'LineWidth', lw_joint);  % Octave-compatible

    % --- optional labels -------------------------------------------
    if ~isfield(opts,'showLabels') || opts.showLabels
        pts = {O, A, B, C, D, F, E, P};
        lab = {'O','A','B','C','D','F','E','P'};
        for q = 1:numel(pts)
            text(ax, pts{q}(1)+dxl, pts{q}(2)+dxl, lab{q}, ...
                'FontSize',fs_label, 'Color','k', ...
                'HorizontalAlignment','left', ...
                'VerticalAlignment','bottom');
        end
    end
end

% --- 9) axis housekeeping ------------------------------------------
local_axis_housekeeping(ax, opts);
end


% ======================================================================
%  local helpers
% ======================================================================
function colors = stephensonIII_colors()
% Default per-solution colors: first two match fourbar_plot (red, blue)
colors = [1    0    0   ;   % red
          0    0    1   ;   % blue
          0    0.6  0   ;   % green
          0.9  0.5  0   ;   % orange
          0.55 0    0.75;   % purple
          0    0.6  0.6 ];  % teal
end


function out = local_track(sol, prev, nSlots)
% Reorder the valid solutions of SOL into nSlots slots so that each one
% keeps the slot of the closest configuration in PREV (previous frame).
% Optimal assignment by exhaustive search (at most 6! = 720 cases).
tmpl = local_nanify(sol(1));
tmpl.valid = 0;
out = repmat(tmpl, 1, nSlots);

cur = find(arrayfun(@(q) isfield(q,'valid') && q.valid, sol(:).'));
cur = cur(1:min(end, nSlots));

pv = [];
if ~isempty(prev) && isstruct(prev) && isfield(prev,'valid')
    pv = find(arrayfun(@(q) q.valid == 1, prev(:).'));
    pv = pv(pv <= nSlots);
end

slot = zeros(1, numel(cur));          % slot assigned to each current sol
if ~isempty(pv) && ~isempty(cur)
    n = max(numel(cur), numel(pv));
    cost = zeros(n);                  % rows: current, cols: previous;
    for i = 1:numel(cur)              % dummy rows/cols cost 0
        for j = 1:numel(pv)
            cost(i,j) = local_confdist(sol(cur(i)), prev(pv(j)));
        end
    end
    P = perms(1:n);
    idx = sub2ind([n n], repmat(1:n, size(P,1), 1), P);
    [~, best] = min(sum(cost(idx), 2));
    p = P(best,:);
    for i = 1:numel(cur)
        if p(i) <= numel(pv)
            slot(i) = pv(p(i));       % continues previous solution
        end
    end
end
free = setdiff(1:nSlots, slot(slot > 0));
for i = 1:numel(cur)                  % new solutions: first free slots
    if slot(i) == 0
        slot(i) = free(1);
        free(1) = [];
    end
end
for i = 1:numel(cur)
    out(slot(i)) = sol(cur(i));
end
end


function d = local_confdist(s1, s2)
% Total displacement of the moving joints between two configurations
d = 0;
for f = {'C','D','F','E','P'}
    d = d + norm(s1.Positions.(f{1}) - s2.Positions.(f{1}));
end
end


function s = local_nanify(s)
% Same struct with every numeric field set to NaN (recursive)
fn = fieldnames(s);
for k = 1:numel(fn)
    v = s.(fn{k});
    if isstruct(v)
        s.(fn{k}) = local_nanify(v);
    elseif isnumeric(v) || islogical(v)
        s.(fn{k}) = nan(size(v));
    end
end
end


function local_axis_housekeeping(ax, opts)
axis(ax,'equal');
grid(ax,'on');
if isfield(opts,'limits') && numel(opts.limits)==4
    xlim(ax, opts.limits(1:2));
    ylim(ax, opts.limits(3:4));
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
