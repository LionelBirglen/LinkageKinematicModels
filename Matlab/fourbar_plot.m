function [ax, sol] = fourbar_plot(geo, mode, inputs, opts)
%FOURBAR_PLOT  Plot a four-bar linkage
%
%   [AX,SOL] = FOURBAR_PLOT(geo, MODE, INPUTS)
%   [AX,SOL] = FOURBAR_PLOT(geo, MODE, INPUTS, OPTS)
%
%   GEO   : 1x8 numeric vector  [a, b, c, d, e, epsilon, delta] (all in consistent length units, epsilon, delta in rad)
%           OR struct with fields:
%              .a, .b, .c, .d, .e, .epsilon, .delta,
%
%   MODE  : 'direct'  -> INPUT = theta
%           'inverse' -> INPUT = alpha
%
%   INPUTS: numeric scalar
%
%   OPTS  : optional struct, fields:
%       .ax         - axes to plot in (GUI will pass data.ax)
%       .solutions  - scalar or vector of solution indices to draw
%                     (default: draw all valid ones, like GUI checkboxes)
%       .showLabels - true/false (default: true)
%       .clearAxes  - true/false (default: true)
%       .limits     - [xmin xmax ymin ymax] to enforce GUI’s fixed limits
%       .colors     - colormap to use (default: lines(4))
%
%   AX    : axes handle used for plotting
%   SOL   : solution structure returned by the kinematics call
%
%   This is meant to be drop-in compatible with fivebar_gui.m.
%
%   Example (direct):
%
%       g = [81, 88, 92, 151, 10, pi/6, 0];
%       fourbar_plot(g,'direct',deg2rad(106));
%
%   Example (inverse, plot only sol 2):
%       fourbar_plot(g,'inverse',pi/3,struct('solutions',2));
%

if nargin < 4, opts = struct(); end
if nargin < 3
    error('fourbar_plot:NotEnoughInputs', ...
        'Need at least: geo, mode, inputs.');
end

% --- 0) Octave vs MATLAB rendering scale factors -----------------
% Octave renders LineWidth and MarkerSize larger than MATLAB at equal
% numeric values, so we scale them down when running under Octave.
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
    fs_label = 10;    % joint label font size
end

% --- 1) geoetry --------------------------------------------------
g = local_parse_geo(geo);

% Ground symbol length: fixed fraction of the longest link, never changes with pose
gs_len = max(g(1:4)) * 0.1125;

% --- 2) normalize mode & inputs -----------------------------------
mode = lower(char(mode));  % char() works in both MATLAB and Octave
if ~isnumeric(inputs) || numel(inputs) ~= 1
    error('fourbar_plot:BadInputs', ...
        'INPUT must be a numeric scalar.');
end

% --- 3) prepare axes ----------------------------------------------
if isfield(opts,'ax') && ishghandle(opts.ax)
    ax = opts.ax;
else
    fig = figure('Name','fivebar_plot','NumberTitle','off');
    ax  = axes('Parent',fig, 'Units','normalized', ...
        'Position',[0.12 0.10 0.78 0.80]);
    axis(ax,'equal');
    grid(ax,'on');
    xlabel(ax,'X');
    ylabel(ax,'Y');
    title(ax,'Four-Bar Linkage');
    %xlim(ax,[-1 1.4]*1.2);
    %ylim(ax,[-1.2 1.2]*1.2);
    hold(ax,'on');
end

if ~isfield(opts,'clearAxes') || opts.clearAxes
    cla(ax);
end

% --- 4) colors ----------------------------------------------------
if isfield(opts,'colors') && ~isempty(opts.colors)
    colors = opts.colors;
else
    colors = [1 0 0;0 0 1];
end

% --- 5) call the existing kinematics functions --------------------
switch mode
    case {'direct','d'}
        if ~exist('fourbar_direct_kinematics','file')
            error('fourbar_plot:MissingFile', ...
                'fourbar_direct_kinematics.m is not on the path.');
        end
        th  = inputs;
        sol = fourbar_direct_kinematics(g, th);

    case {'inverse','i'}
        if ~exist('fourbar_inverse_kinematics','file')
            error('fourbar_plot:MissingFile', ...
                'fourbar_inverse_kinematics.m is not on the path.');
        end
        alph = inputs;
        sol  = fourbar_inverse_kinematics(g, alph);

    otherwise
        error('fourbar_plot:BadMode', ...
            'MODE must either be ''direct'' or ''inverse''.');
end

% If no solution, stop but keep style
if isempty(sol)
    axis(ax,'equal'); grid(ax,'on');
    if isfield(opts,'limits') && numel(opts.limits)==4
        xlim(ax, opts.limits(1:2));
        ylim(ax, opts.limits(3:4));
    end
    return;
end

% --- 6) what to draw? ---------------------------------------------
nSol = numel(sol);
if isfield(opts,'solutions') && ~isempty(opts.solutions)
    idxToPlot = opts.solutions(:).';
else
    idxToPlot = 1:nSol;
end
% keep only valid indices
idxToPlot = idxToPlot(idxToPlot>=1 & idxToPlot<=nSol);
if isempty(idxToPlot)
    idxToPlot = 1:nSol;
end

% --- 7) draw each selected solution -------------------------------
for k = 1:numel(idxToPlot)
    ii = idxToPlot(k);
    s  = sol(ii);

    % GUI checks a "valid" flag
    if isfield(s,'valid') && ~s.valid
        continue;
    end

    % Same fields fivebar_gui uses:
    % s.Positions.O, s.Positions.A, s.Positions.B,
    % s.Positions.C, s.Positions.D, and s.P
    O = s.Positions.O;
    A = s.Positions.A;
    B = s.Positions.B;
    C = s.Positions.C;
    P = s.P;

    % color cycling as in GUI (it uses lines(4) and loops)
    col = colors( mod(ii-1, size(colors,1)) + 1 , : );

    % --- ground symbol --------------------------------------
    drawGroundSymbol(ax,O,lw_ground_hatch,lw_ground_base,gs_len);
    drawGroundSymbol(ax,C,lw_ground_hatch,lw_ground_base,gs_len);

    % --- linkage polygon: O-A-B-C-O ------------------------------
    plot(ax, [O(1) A(1) B(1) C(1)], ...
        [O(2) A(2) B(2) C(2)], ...
        '-', 'Color', col, 'LineWidth', lw_link);
    hold(ax,'on');
    grid(ax,'on');

    % --- point P (black "x") ---------------------------------------
    plot(ax, P(1), P(2), 'kx', 'MarkerSize', ms_P, 'LineWidth', lw_link);

    % --- triangle A-B-P, semi-transparent --------------------------
    patch('XData', [A(1) B(1) P(1)], ...
        'YData', [A(2) B(2) P(2)], ...
        'FaceColor', col, ...
        'EdgeColor', col, ...
        'FaceAlpha', 0.5, ...
        'Parent', ax);

    % --- joints as filled white circles ----------------------------
    scatter(ax, [O(1) A(1) B(1) C(1)], ...
        [O(2) A(2) B(2) C(2)], ...
        ms_joint, 'MarkerFaceColor','w', 'MarkerEdgeColor','k', 'LineWidth', lw_joint);  % Octave-compatible

    % --- optional labels -------------------------------------------
    if ~isfield(opts,'showLabels') || opts.showLabels
        local_label_joint(ax, s, fs_label);
    end
end

% --- 8) axis housekeeping ------------------------------------------
axis(ax,'equal');

if isfield(opts,'limits') && numel(opts.limits)==4
    xlim(ax, opts.limits(1:2));
    ylim(ax, opts.limits(3:4));
end
end


% ======================================================================
%  local helpers
% ======================================================================
function g = local_parse_geo(geo)
if isnumeric(geo)
    if numel(geo) ~= 7
        error('fourbar_plot:Badgeo', ...
            'Numeric geo must have 7 elements: [a b c d e epsilon delta].');
    end
    g = geo(:).';
elseif isstruct(geo)
    must = {'a','b','c','d','e','epsilon','delta'};
    g = zeros(1,7);
    for k = 1:7
        if ~isfield(geo, must{k})
            error('fourbar_plot:BadgeoStruct', ...
                'Missing field "%s" in geometry struct.', must{k});
        end
        g(k) = geo.(must{k});
    end
else
    error('fourbar_plot:BadgeoType', ...
        'geo must be a 1x7 numeric vector or a struct with fields a..delta.');
end
end


function local_label_joint(ax, s, fs_label)
% dx = 0.02;
% dy = 0.02;
O = s.Positions.O;
A = s.Positions.A;
B = s.Positions.B;
C = s.Positions.C;
P = s.P;

dx=norm(A-O)/20;
dy=dx;

text(ax, O(1)+dx, O(2)+dy, 'O', ...
    'FontSize',fs_label, 'Color','k', ...
    'HorizontalAlignment','left', ...
    'VerticalAlignment','bottom');
text(ax, A(1)+dx, A(2)+dy, 'A', ...
    'FontSize',fs_label, 'Color','k', ...
    'HorizontalAlignment','left', ...
    'VerticalAlignment','bottom');
text(ax, B(1)+dx, B(2)+dy, 'B', ...
    'FontSize',fs_label, 'Color','k', ...
    'HorizontalAlignment','left', ...
    'VerticalAlignment','bottom');
text(ax, C(1)+dx, C(2)+dy, 'C', ...
    'FontSize',fs_label, 'Color','k', ...
    'HorizontalAlignment','left', ...
    'VerticalAlignment','bottom');
text(ax, P(1)+dx, P(2)+dy, 'P', ...
    'FontSize',fs_label, 'Color','k', ...
    'HorizontalAlignment','left', ...
    'VerticalAlignment','bottom');
end

% function drawGroundSymbol(axh, Q_pt)
% % Draw small "L" shapes at point Q
% Ls = max(get(axh,'XLim')) * 0.02;
% plot(axh, [Q_pt(1)-Ls, Q_pt(1)+Ls], [Q_pt(2), Q_pt(2)], 'k-', 'LineWidth',2);
% plot(axh, [Q_pt(1), Q_pt(1)], [Q_pt(2)-Ls, Q_pt(2)+Ls], 'k-', 'LineWidth',2);
% end

function drawGroundSymbol(ax, jointCenter, lw_ground_hatch, lw_ground_base, lineLength)
jointCenter=jointCenter';
numLines = 3;
% lineLength is passed in as argument (fixed to geometry, not axis limits)
lineSpacing = lineLength/2;
angle = +pi/4;
R = [cos(angle+pi) -sin(angle+pi); sin(angle+pi) cos(angle+pi)];
x=[lineLength;0];
Rx=R*x;
for i = 0:numLines-1
    v1 = jointCenter + [i*lineSpacing-(numLines-1)/2*lineSpacing;0];
    v2 = v1+Rx;
    plot(ax, [v1(1) v2(1)], [v1(2) v2(2)], 'k', 'LineWidth', lw_ground_hatch);
end
plot(ax, jointCenter(1)+[-(numLines-1)/2*lineSpacing +(numLines-1)/2*lineSpacing], [jointCenter(2) jointCenter(2)], 'k', 'LineWidth', lw_ground_base);

end
