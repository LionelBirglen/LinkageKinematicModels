function [ax, sol] = fivebar_plot(geo, mode, inputs, opts)
%FIVEBAR_PLOT  Plot a five-bar linkage 
%
%   [AX,SOL] = FIVEBAR_PLOT(geo, MODE, INPUTS)
%   [AX,SOL] = FIVEBAR_PLOT(geo, MODE, INPUTS, OPTS)
%
%   GEO   : 1x8 numeric vector  [a b c d e alpha h eta]
%           OR struct with fields:
%              .a, .b, .c, .d, .e, .alpha, .h, .eta
%
%   MODE  : 'direct'  -> INPUTS = [theta1_deg theta2_deg]
%           'inverse' -> INPUTS = [Px Py]
%
%   INPUTS: 1x2 numeric vector
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
%       g = [0.6 0.7 0.9 0.6 1.0 0 0.5 45];
%       fivebar_plot(g,'direct',[30 60]);
%
%   Example (inverse, plot only sol 2):
%       fivebar_plot(g,'inverse',[0.4 0.2],struct('solutions',2));
%

    if nargin < 4, opts = struct(); end
    if nargin < 3
        error('fivebar_plot:NotEnoughInputs', ...
              'Need at least: geo, mode, inputs.');
    end

    % --- 0) Octave vs MATLAB scale factors ----------------------------
    isOctave = exist('OCTAVE_VERSION','builtin') ~= 0;
    if isOctave
        lw_link  = 0.6;
        ms_joint = 6;
        ms_ee    = 3;
        lw_joint = 0.5;
    else
        lw_link  = 2.0;
        ms_joint = 40;
        ms_ee    = 8;
        lw_joint = 1.0;
    end

    % --- 1) geoetry --------------------------------------------------
    g = local_parse_geo(geo);

    % --- 2) normalize mode & inputs -----------------------------------
    mode = lower(char(mode));
    if ~isnumeric(inputs) || numel(inputs) ~= 2
        error('fivebar_plot:BadInputs', ...
              'INPUTS must be a 1x2 numeric vector.');
    end
    inputs = inputs(:).';  % make it row

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
        title(ax,'Five-Bar Linkage');
        xlim(ax,[-1 1.4]*1.2);
        ylim(ax,[-1.2 1.2]*1.2);
        hold(ax,'on');
    end

    if ~isfield(opts,'clearAxes') || opts.clearAxes
        cla(ax);
    end

    % --- 4) colors ----------------------------------------------------
    if isfield(opts,'colors') && ~isempty(opts.colors)
        colors = opts.colors;
    else
        colors = lines(4);
    end

    % --- 5) call the existing kinematics functions --------------------
    switch mode
        case {'direct','d'}
            if ~exist('fivebar_direct_kinematics','file')
                error('fivebar_plot:MissingFile', ...
                    'fivebar_direct_kinematics.m is not on the path.');
            end
            th  = inputs;                      % [theta1 theta2] deg
            sol = fivebar_direct_kinematics(g, th(:));

        case {'inverse','i'}
            if ~exist('fivebar_inverse_kinematics','file')
                error('fivebar_plot:MissingFile', ...
                    'fivebar_inverse_kinematics.m is not on the path.');
            end
            Pdes = inputs;                     % [Px Py]
            sol  = fivebar_inverse_kinematics(g, Pdes(:));

        otherwise
            error('fivebar_plot:BadMode', ...
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
        D = s.Positions.D;
        P = s.P;

        % color cycling as in GUI (it uses lines(4) and loops)
        col = colors( mod(ii-1, size(colors,1)) + 1 , : );

        % --- linkage polygon: O-A-B-C-D-O ------------------------------
        plot(ax, [O(1) A(1) B(1) C(1) D(1) O(1)], ...
                 [O(2) A(2) B(2) C(2) D(2) O(2)], ...
                 '-', 'Color', col, 'LineWidth', lw_link);
        hold(ax,'on');
        grid(ax,'on');

        % --- point P (black "x") ---------------------------------------
        plot(ax, P(1), P(2), 'kx', 'MarkerSize', ms_ee, 'LineWidth', lw_link);

        % --- triangle A-B-P, semi-transparent --------------------------
        patch('XData', [A(1) B(1) P(1)], ...
              'YData', [A(2) B(2) P(2)], ...
              'FaceColor', col, ...
              'EdgeColor', col, ...
              'FaceAlpha', 0.5, ...
              'Parent', ax);

        % --- joints as filled black circles ----------------------------
        scatter(ax, [O(1) A(1) B(1) C(1) D(1)], ...
                   [O(2) A(2) B(2) C(2) D(2)], ...
                   ms_joint, 'MarkerFaceColor','w','MarkerEdgeColor','k','LineWidth', lw_joint);

        % --- optional labels -------------------------------------------
        if ~isfield(opts,'showLabels') || opts.showLabels
            local_label_joint(ax, s);
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
        if numel(geo) ~= 8
            error('fivebar_plot:Badgeo', ...
                  'Numeric geo must have 8 elements: [a b c d e alpha h eta].');
        end
        g = geo(:).';
    elseif isstruct(geo)
        must = {'a','b','c','d','e','alpha','h','eta'};
        g = zeros(1,8);
        for k = 1:8
            if ~isfield(geo, must{k})
                error('fivebar_plot:BadgeoStruct', ...
                      'Missing field "%s" in geoetry struct.', must{k});
            end
            g(k) = geo.(must{k});
        end
    else
        error('fivebar_plot:BadgeoType', ...
              'geo must be a 1x8 numeric vector or a struct with fields a..eta.');
    end
end


function local_label_joint(ax, s)
    % dx = 0.02;
    % dy = 0.02;
    O = s.Positions.O;
    A = s.Positions.A;
    B = s.Positions.B;
    C = s.Positions.C;
    D = s.Positions.D;
    P = s.P;

    dx=norm(A-O)/20;
    dy=dx;

    text(ax, O(1)+dx, O(2)+dy, 'O', ...
        'FontSize',10, 'Color','k', ...
        'HorizontalAlignment','left', ...
        'VerticalAlignment','bottom');
    text(ax, A(1)+dx, A(2)+dy, 'A', ...
        'FontSize',10, 'Color','k', ...
        'HorizontalAlignment','left', ...
        'VerticalAlignment','bottom');
    text(ax, B(1)+dx, B(2)+dy, 'B', ...
        'FontSize',10, 'Color','k', ...
        'HorizontalAlignment','left', ...
        'VerticalAlignment','bottom');
    text(ax, C(1)+dx, C(2)+dy, 'C', ...
        'FontSize',10, 'Color','k', ...
        'HorizontalAlignment','left', ...
        'VerticalAlignment','bottom');
    text(ax, D(1)+dx, D(2)+dy, 'D', ...
        'FontSize',10, 'Color','k', ...
        'HorizontalAlignment','left', ...
        'VerticalAlignment','bottom');
    text(ax, P(1)+dx, P(2)+dy, 'P', ...
        'FontSize',10, 'Color','k', ...
        'HorizontalAlignment','left', ...
        'VerticalAlignment','bottom');
end
