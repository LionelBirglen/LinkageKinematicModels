function ax = rrr_plot(geo, mode, inputs, opts)
%RRR_PLOT  Plot a planar RRR serial linkage
%
%   AX = RRR_PLOT(L1, L2, L3, MODE, INPUTS)
%   AX = RRR_PLOT(L1, L2, L3, MODE, INPUTS, OPTS)
%
%   L1, L2, L3 : link lengths (consistent units)
%
%   MODE   : 'direct'  -> INPUTS = [theta1, theta2, theta3] (rad)
%            'inverse' -> INPUTS = [Px, Py, phi, elbow_config]
%                         elbow_config: +1 elbow-up, -1 elbow-down
%
%   INPUTS : numeric vector (see MODE above)
%
%   OPTS   : optional struct, fields:
%       .ax          - axes handle to plot in
%       .showLabels  - true/false (default: true)
%       .clearAxes   - true/false (default: true)
%       .limits      - [xmin xmax ymin ymax]
%       .showBoth    - true/false: also draw alternate elbow config (inverse only)
%       .colors      - Nx3 color matrix (default: red/green/blue per link)
%       .lineStyle   - line style string (default: '-')
%
%   AX     : axes handle used for plotting
%
%   Example (direct):
%       geo = struct('L1',57,'L2',46,'L3',51);
%       rrr_plot(geo, 'direct', deg2rad([39 37 40]));
%
%   Example (inverse, elbow-up, show both configs):
%       rrr_plot(geo,'inverse',[35 125 deg2rad(116) 1],struct('showBoth',true));
%
% BY:
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Contact: lionel.birglen@polymtl.ca
% Code provided under GNU Affero General Public License v3.0

if nargin < 4, opts = struct(); end
if nargin < 3
    error('rrr_plot:NotEnoughInputs','Need at least: geo, mode, inputs.');
end
[L1, L2, L3] = rrr_parse_geo(geo);

% --- 0) Octave vs MATLAB rendering scale factors ---------------------
isOctave = exist('OCTAVE_VERSION','builtin') ~= 0;
if isOctave
    lw_link         = 0.6;
    lw_joint        = 0.4;
    lw_ground_hatch = 0.4;
    lw_ground_base  = 0.3;
    ms_joint        = 6;
    ms_ee           = 2;
    fs_label        = 14;
else
    lw_link         = 1.5;
    lw_joint        = 1.0;
    lw_ground_hatch = 1.5;
    lw_ground_base  = 1.0;
    ms_joint        = 40;
    ms_ee           = 8;
    fs_label        = 10;
end

% Ground symbol size: fixed fraction of longest link
gs_len = max([L1 L2 L3]) * 0.1125;

% --- 1) Prepare axes -------------------------------------------------
if isfield(opts,'ax') && ishghandle(opts.ax)
    ax = opts.ax;
else
    fig = figure('Name','RRR Linkage','NumberTitle','off');
    ax  = axes('Parent',fig,'Units','normalized','Position',[0.12 0.10 0.78 0.80]);
    axis(ax,'equal');
    grid(ax,'on');
    xlabel(ax,'X');
    ylabel(ax,'Y');
    title(ax,'Planar RRR Linkage');
    hold(ax,'on');
end

if ~isfield(opts,'clearAxes') || opts.clearAxes
    cla(ax);
end

% --- 2) Colors -------------------------------------------------------
if isfield(opts,'colors') && ~isempty(opts.colors)
    colors = opts.colors;
else
    colors = [1 0 0; 0 0.7 0; 0 0 1];   % red / green / blue per link
end

ls = '-';
if isfield(opts,'lineStyle'), ls = opts.lineStyle; end

% --- 3) Kinematics and drawing ---------------------------------------
mode = lower(char(mode));
switch mode
    case {'direct','d'}
        if numel(inputs) < 3
            error('rrr_plot:BadInputs','Direct mode needs [theta1 theta2 theta3] (rad).');
        end
        sol = rrr_direct_kinematics(geo,inputs(1),inputs(2),inputs(3));
        if sol.valid
            local_draw(ax, sol, colors, ls, lw_link, lw_joint, ...
                lw_ground_hatch, lw_ground_base, ms_joint, ms_ee, fs_label, gs_len, opts);
        end

    case {'inverse','i'}
        if numel(inputs) < 4
            error('rrr_plot:BadInputs','Inverse mode needs [Px Py phi elbow_config].');
        end
        Px=inputs(1); Py=inputs(2); phi=inputs(3); cfg=inputs(4);
        sol = rrr_inverse_kinematics(geo,Px,Py,phi,cfg);
        if sol.valid
            local_draw(ax, sol, colors, ls, lw_link, lw_joint, ...
                lw_ground_hatch, lw_ground_base, ms_joint, ms_ee, fs_label, gs_len, opts);
        else
            text(0, 0, 'Unreachable', 'Parent', ax, 'Color', 'r', ...
                'FontSize', 14, 'HorizontalAlignment', 'center');
        end
        % Alternate elbow config
        if isfield(opts,'showBoth') && opts.showBoth
            sol2 = rrr_inverse_kinematics(geo,Px,Py,phi,-cfg);
            if sol2.valid
                opts2             = opts;
                opts2.clearAxes   = false;
                opts2.lineStyle   = '--';
                opts2.showLabels   = true;
                opts2.labelSubset  = 2;  % only label A; O, B, P already labelled by sol 1
                local_draw(ax, sol2, colors, '--', lw_link, lw_joint, ...
                    lw_ground_hatch, lw_ground_base, ms_joint, ms_ee, fs_label, gs_len, opts2);
            end
        end

    otherwise
        error('rrr_plot:BadMode','MODE must be ''direct'' or ''inverse''.');
end

% --- 4) Axis housekeeping --------------------------------------------
axis(ax,'equal');
if isfield(opts,'limits') && numel(opts.limits)==4
    xlim(ax, opts.limits(1:2));
    ylim(ax, opts.limits(3:4));
end
end


% =====================================================================
%  Local helpers
% =====================================================================
function local_draw(ax, sol, colors, ls, lw_link, lw_joint, ...
    lw_ground_hatch, lw_ground_base, ms_joint, ms_ee, fs_label, gs_len, opts)

O  = sol.Positions.O;
A = sol.Positions.A;
B = sol.Positions.B;
P = sol.Positions.P;

% Ground symbol at base
drawGroundSymbol(ax, O, lw_ground_hatch, lw_ground_base, gs_len);

% Draw three links in three colors
segs = {[O A], [A B], [B P]};
for k = 1:3
    col = colors(mod(k-1, size(colors,1))+1, :);
    plot(ax, segs{k}(1,:), segs{k}(2,:), ...
        'Color', col, 'LineStyle', ls, 'LineWidth', lw_link);
    hold(ax,'on');
end

% End-effector marker (X)
plot(ax, P(1), P(2), 'kx', 'MarkerSize', ms_ee, 'LineWidth', lw_link);

% Joints as white circles with black border (base + A + B)
jx = [O(1) A(1) B(1)];
jy = [O(2) A(2) B(2)];
scatter(ax, jx, jy, ms_joint, ...
    'MarkerFaceColor','w', 'MarkerEdgeColor','k', 'LineWidth', lw_joint);

% Labels
if ~isfield(opts,'showLabels') || opts.showLabels
    dx = max(abs([A(1)-O(1), B(1)-A(1), P(1)-B(1), ...
                  A(2)-O(2), B(2)-A(2), P(2)-B(2)])) / 10;
    if dx == 0, dx = 1; end
    dy = dx;
    lbls = {'O','A','B','P'};
    pts  = [O A B P];   % 2x4
    if isfield(opts,'labelSubset')
        toLabel = opts.labelSubset;
    else
        toLabel = 1:4;
    end
    for k = toLabel
        text(ax, pts(1,k)+dx, pts(2,k)+dy, lbls{k}, ...
            'FontSize', fs_label, 'Color', 'k', ...
            'HorizontalAlignment','left','VerticalAlignment','bottom');
    end
end
end


function drawGroundSymbol(ax, jointCenter, lw_ground_hatch, lw_ground_base, lineLength)
jc          = jointCenter(:);
numLines    = 3;
lineSpacing = lineLength / 2;
angle       = pi + pi/4;
R           = [cos(angle) -sin(angle); sin(angle) cos(angle)];
Rx          = R * [lineLength; 0];
for i = 0:numLines-1
    v1 = jc + [i*lineSpacing - (numLines-1)/2*lineSpacing; 0];
    v2 = v1 + Rx;
    plot(ax, [v1(1) v2(1)], [v1(2) v2(2)], 'k', 'LineWidth', lw_ground_hatch);
end
plot(ax, ...
    jc(1) + [-(numLines-1)/2*lineSpacing, +(numLines-1)/2*lineSpacing], ...
    [jc(2) jc(2)], 'k', 'LineWidth', lw_ground_base);
end

function [L1, L2, L3] = rrr_parse_geo(geo)
if isnumeric(geo)
    g = geo(:).'; L1=g(1); L2=g(2); L3=g(3);
elseif isstruct(geo)
    L1=geo.L1; L2=geo.L2; L3=geo.L3;
else
    error('rrr_plot:BadGeo','geo must be [L1 L2 L3] or struct with L1,L2,L3.');
end
end
