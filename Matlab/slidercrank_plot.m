function ax = slidercrank_plot(geo, mode, inputs, opts)
%SLIDERCRANK_PLOT  Plot a planar slider-crank mechanism
%
%   AX = SLIDERCRANK_PLOT(a, b, MODE, INPUTS)
%   AX = SLIDERCRANK_PLOT(a, b, MODE, INPUTS, OPTS)
%
%   a, b   : crank and coupler lengths (consistent units)
%
%   MODE   : 'direct'  -> INPUTS = [phi, slider_angle, config]  (phi in rad)
%            'inverse' -> INPUTS = [x_slider, slider_angle, config]
%
%   OPTS   : optional struct:
%       .ax          - axes handle
%       .clearAxes   - true/false (default true)
%       .showLabels  - true/false (default true)
%       .limits      - [xmin xmax ymin ymax]
%       .showBoth    - true/false: draw alternate config too
%       .lineStyle   - '-' or '--' (default '-')
%       .labelSubset - indices into {'O','B','P'} to label (default 1:3)
%
%   AX     : axes handle used
%
%   Example (direct):
%       geo = struct('a',50,'b',120,'c',30,'slider_angle',0);
%       slidercrank_plot(geo,'direct',[deg2rad(30),1]);  % [phi, config]
%
% BY:
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Contact: lionel.birglen@polymtl.ca
% Code provided under GNU Affero General Public License v3.0

if nargin < 4, opts = struct(); end

% --- Octave vs MATLAB scale factors -----------------------------------
isOctave = exist('OCTAVE_VERSION','builtin') ~= 0;
if isOctave
    lw_link         = 0.6;
    lw_joint        = 0.4;
    lw_ground_hatch = 0.4;
    lw_ground_base  = 0.3;
    lw_rail         = 0.4;
    ms_joint        = 6;
    ms_ee           = 2;
    fs_label        = 14;
else
    lw_link         = 1.5;
    lw_joint        = 1.0;
    lw_ground_hatch = 1.5;
    lw_ground_base  = 1.0;
    lw_rail         = 0.8;
    ms_joint        = 40;
    ms_ee           = 8;
    fs_label        = 10;
end

gs_len = max(a,b) * 0.1125;   % ground symbol size

% --- Prepare axes -----------------------------------------------------
if isfield(opts,'ax') && ishghandle(opts.ax)
    ax = opts.ax;
else
    fig = figure('Name','Slider-Crank','NumberTitle','off');
    ax  = axes('Parent',fig,'Units','normalized','Position',[0.12 0.10 0.78 0.80]);
    axis(ax,'equal'); grid(ax,'on');
    xlabel(ax,'X'); ylabel(ax,'Y');
    title(ax,'Slider-Crank Linkage');
    hold(ax,'on');
end

if ~isfield(opts,'clearAxes') || opts.clearAxes
    cla(ax);
end

% --- Parse inputs -----------------------------------------------------
mode = lower(char(mode));
if numel(inputs) < 2
    error('slidercrank_plot:BadInputs','INPUTS needs [value, config].');
end
config       = inputs(2);
[a, b, c, slider_angle] = sc_parse_geo_plot(geo);
ls = '-';
if isfield(opts,'lineStyle'), ls = opts.lineStyle; end

% --- Kinematics -------------------------------------------------------
switch mode
    case {'direct','d'}
        phi = inputs(1);
        sol = slidercrank_direct_kinematics(a, b, c, phi, slider_angle, config);
    case {'inverse','i'}
        x_s = inputs(1);
        sol  = slidercrank_inverse_kinematics(a, b, c, x_s, config, slider_angle);
    otherwise
        error('slidercrank_plot:BadMode','MODE must be ''direct'' or ''inverse''.');
end

slider_dir = [cos(slider_angle); sin(slider_angle)];

% --- Draw rail (fixed in space) ----------------------------------------
R_lim = a + b;
rail_s = -R_lim * slider_dir;
rail_e =  R_lim * slider_dir;
plot(ax, [rail_s(1) rail_e(1)], [rail_s(2) rail_e(2)], 'k-.', 'LineWidth', lw_rail);
hold(ax,'on');

% --- Draw fixed slider block (grounded prismatic joint) ----------------
% The block is fixed to the ground at the midstroke position.
% x_fixed = a places it at the crank-length distance along the slider axis.
x_fixed   = a;
blk_centre = x_fixed * slider_dir;
local_drawSlider(ax, blk_centre, slider_dir, slider_angle, lw_ground_hatch, gs_len);

% --- Ground symbol at O -----------------------------------------------
local_drawGroundSymbol(ax, [0;0], lw_ground_hatch, lw_ground_base, gs_len);
% Label O once here — it is a fixed joint and must not move
if ~isfield(opts,'showLabels') || opts.showLabels
    text(ax, gs_len*0.6, gs_len*0.6, 'O', 'FontSize', fs_label, 'Color', 'k', ...
        'HorizontalAlignment','left','VerticalAlignment','bottom');
end

% --- Draw primary solution (sol 1) ------------------------------------
showFirst = ~isfield(opts,'showFirst') || opts.showFirst;
if showFirst
    if sol.valid
        local_draw(ax, sol, [1 0 0; 0 0.7 0], ls, lw_link, lw_joint, ...
            ms_joint, ms_ee, fs_label, gs_len, slider_angle, opts, false);
    else
        text(0, 0, 'Sol 1: Unreachable', 'Parent', ax, 'Color', 'r', ...
            'FontSize', 14, 'HorizontalAlignment', 'center');
    end
end

% --- Draw alternate config if requested --------------------------------
if isfield(opts,'showBoth') && opts.showBoth
    switch mode
        case {'direct','d'}
            sol2 = slidercrank_direct_kinematics(a, b, c, phi, slider_angle, -config);
        case {'inverse','i'}
            sol2 = slidercrank_inverse_kinematics(a, b, c, x_s, -config, slider_angle);
    end
    if sol2.valid
        opts2            = opts;
        opts2.clearAxes  = false;
        opts2.lineStyle  = '--';
        opts2.labelSubset = [3 4];  % label B and P for alternate (O and A already drawn)
        local_draw(ax, sol2, [1 0 0; 0 0.7 0], '--', lw_link, lw_joint, ...
            ms_joint, ms_ee, fs_label, gs_len, slider_angle, opts2, true);
    end
end

% --- Axis housekeeping ------------------------------------------------
axis(ax,'equal');
if isfield(opts,'limits') && numel(opts.limits)==4
    xlim(ax, opts.limits(1:2));
    ylim(ax, opts.limits(3:4));
end
end


% ======================================================================
%  Local helpers
% ======================================================================
function local_draw(ax, sol, colors, ls, lw_link, lw_joint, ...
    ms_joint, ms_ee, fs_label, gs_len, slider_angle, opts, isAlt)

O = sol.Positions.O;
A = sol.Positions.A;
B = sol.Positions.B;
P = sol.Positions.P;
slider_dir = sol.slider_dir;

% Crank link O->A (red)
plot(ax, [O(1) A(1)], [O(2) A(2)], '-', 'Color', colors(1,:), ...
    'LineStyle', ls, 'LineWidth', lw_link);
% Coupler link A->B (green); B is the pin that rides in the slider block
plot(ax, [A(1) B(1)], [A(2) B(2)], '-', 'Color', colors(2,:), ...
    'LineStyle', ls, 'LineWidth', lw_link);
% Extension link B->P (blue)
plot(ax, [B(1) P(1)], [B(2) P(2)], '-', 'Color', [0 0 1], ...
    'LineStyle', ls, 'LineWidth', lw_link);

% P marker as cross (like fourbar/rrr end-effectors)
plot(ax, P(1), P(2), 'kx', 'MarkerSize', ms_ee, 'LineWidth', lw_link);

% Revolute joints O, A, B as white circles (P is a cross, not a circle)
jx = [O(1) A(1) B(1)];
jy = [O(2) A(2) B(2)];
scatter(ax, jx, jy, ms_joint, 'MarkerFaceColor','w','MarkerEdgeColor','k','LineWidth',lw_joint);

% Labels
lbls = {'O','A','B','P'};
pts  = [O A B P];
if isfield(opts,'labelSubset')
    toLabel = opts.labelSubset;
else
    toLabel = 2:4;  % O is labelled once in the main body (fixed joint)
end
if ~isfield(opts,'showLabels') || opts.showLabels
    dx = gs_len * 0.6;  % fixed offset based on geometry, not pose
    for k = toLabel
        text(ax, pts(1,k)+dx, pts(2,k)+dx, lbls{k}, ...
            'FontSize', fs_label, 'Color', 'k', ...
            'HorizontalAlignment','left','VerticalAlignment','bottom');
    end
end
end


function local_drawSlider(ax, centre, slider_dir, slider_angle, lw, gs_len)
% Draw the fixed slider block (grounded prismatic joint) as a rectangle.
% centre is a fixed point — does not change with crank angle.
perp_dir = [-sin(slider_angle); cos(slider_angle)];
sl = gs_len * 3.0;   % block length along rail
sw = gs_len * 1.8;   % block width across rail
c = [centre + sl/2*slider_dir + sw/2*perp_dir, ...
     centre + sl/2*slider_dir - sw/2*perp_dir, ...
     centre - sl/2*slider_dir - sw/2*perp_dir, ...
     centre - sl/2*slider_dir + sw/2*perp_dir];
fill(ax, c(1,:), c(2,:), [0.75 0.85 1.0], 'FaceAlpha', 0.6, 'EdgeColor','k','LineWidth',lw);
end


function local_drawGroundSymbol(ax, jc, lw_hatch, lw_base, lineLength)
jc          = jc(:);
numLines    = 3;
lineSpacing = lineLength / 2;
angle       = pi + pi/4;
R           = [cos(angle) -sin(angle); sin(angle) cos(angle)];
Rx          = R * [lineLength; 0];
for i = 0:numLines-1
    v1 = jc + [i*lineSpacing - (numLines-1)/2*lineSpacing; 0];
    v2 = v1 + Rx;
    plot(ax, [v1(1) v2(1)], [v1(2) v2(2)], 'k', 'LineWidth', lw_hatch);
end
plot(ax, jc(1)+[-(numLines-1)/2*lineSpacing, +(numLines-1)/2*lineSpacing], ...
    [jc(2) jc(2)], 'k', 'LineWidth', lw_base);
end

function [a, b, c, slider_angle] = sc_parse_geo_plot(geo)
if isnumeric(geo)
    g=geo(:).'; a=g(1); b=g(2); c=g(3); slider_angle=g(4);
elseif isstruct(geo)
    a=geo.a; b=geo.b; c=geo.c; slider_angle=geo.slider_angle;
else
    error('slidercrank_plot:BadGeo','geo must be [a b c slider_angle] or struct.');
end
end
