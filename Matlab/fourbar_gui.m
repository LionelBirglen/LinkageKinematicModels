function fourbar_gui()
% FOURBAR_GUI - Interactive GUI for a Planar Four-Bar Linkage
%
% OBJECTIVE:
%   Launch a MATLAB GUI to simulate and visualize the kinematics of a
%   planar four-bar mechanism. Supports direct/inverse kinematics,
%   configuration toggling (open/crossed), alternate solution display,
%   and animation.
%
% INPUTS:
%   None. User interacts through GUI controls for link lengths, angles,
%   modes, and configurations.
%
% OUTPUTS:
%   No return arguments. Visualization is rendered in the GUI figure.
%
% USAGE:
%   >> fourbar_gui
%   Opens the GUI with default link lengths and angle.
%
% SHOW D TRAJECTORY:
%   Draws, for each displayed solution, the point D where the lines O-A
%   and C-B intersect (instantaneous center of rotation of the coupler
%   A-B), with fine black lines through O, A, D and through C, B, D, as
%   well as the path of D over a full rotation of the crank (dotted for
%   solution 1, dashed for solution 2, as the P trajectory). Where O-A
%   and C-B are parallel, D is at infinity and is not drawn.
%
% BY:
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025-...

% ----------------------------------------------------------------
% 0) Environment detection
% ----------------------------------------------------------------
isOctave = exist('OCTAVE_VERSION','builtin') ~= 0;

% ----------------------------------------------------------------
% 1) Figure
% ----------------------------------------------------------------
if isOctave
    hFig = figure('Name','Four-Bar Linkage GUI','NumberTitle','off', ...
        'MenuBar','figure','ToolBar','figure','Position',[300 100 900 600]);
    set(hFig,'Color',[1 1 1]);
    set(0,'DefaultUicontrolBackgroundColor',[1 1 1]);
else
    hFig = figure('Name','Four-Bar Linkage GUI','NumberTitle','off', ...
        'MenuBar','none','ToolBar','figure','Position',[300 100 900 600]);
end

% ----------------------------------------------------------------
% 2) Menu bar
% ----------------------------------------------------------------
hMenuFig = gcf;
if isOctave
    % Remove Octave's built-in menus so only our custom ones remain
    delete(findall(hMenuFig,'Type','uimenu'));
end

% Define callback selection helpers for uifigure vs. classic figure
isUIFigure = isa(hMenuFig, 'matlab.ui.Figure');
if isUIFigure
    parentProp = 'Text';
    cbProp = 'MenuSelectedFcn';
else
    parentProp = 'Label';
    cbProp = 'Callback';
end

% Create top-level menus only if none exist yet
existingMenus = findall(hMenuFig, 'Type','uimenu','-depth',1);
if isempty(existingMenus)
    % ---- Top level ----
    mFile    = uimenu(hMenuFig, parentProp,'File');
    mEdit    = uimenu(hMenuFig, parentProp,'Edit');
    mView    = uimenu(hMenuFig, parentProp,'View');
    mOptions = uimenu(hMenuFig, parentProp,'Options');
    mHelp    = uimenu(hMenuFig, parentProp,'Help');

    % ---- File submenus ----
    uimenu(mFile, parentProp,'Open',            cbProp,@(~,~) cbOpen(hMenuFig));
    uimenu(mFile, parentProp,'Save',            cbProp,@(~,~) cbSave(hMenuFig));
    uimenu(mFile, parentProp,'Export PNG',      cbProp,@(~,~) cbExportPNG(hMenuFig));
    uimenu(mFile, parentProp,'Export EPS+PDF',  cbProp,@(~,~) cbExportEPSPDF(hMenuFig));
    uimenu(mFile, parentProp,'Print',           cbProp,@(~,~) cbPrint(hMenuFig));
    if isOctave
        epsMenu = findall(hMenuFig,'Label','Export EPS+PDF');
        prtMenu = findall(hMenuFig,'Label','Print');
        if ~isempty(epsMenu), set(epsMenu,'Enable','off'); end
        if ~isempty(prtMenu), set(prtMenu,'Enable','off'); end
    end
    uimenu(mFile, parentProp,'Exit',            cbProp,@(~,~) cbExit(hMenuFig));

    % ---- View submenus ----
    uimenu(mView, parentProp,'Reset View',      cbProp,@(~,~) cbResetView(hMenuFig));

    % ---- Options submenus ----
    uimenu(mOptions, parentProp,'Preferences',  cbProp,@(~,~) cbPreferences(hMenuFig));
end

% ----------------------------------------------------------------
% 3) Defaults
% ----------------------------------------------------------------
def_a       = 0.81;
def_b       = 0.88;
def_c       = 0.92;
def_d       = 1.51;
def_e       = 0.80;
def_epsilon = 30;
def_delta   = -10;

% ----------------------------------------------------------------
% 4) Geometry panel  
% ----------------------------------------------------------------
geoPanel = uipanel('Title','Geometry', ...
    'FontSize',10, ...
    'Position',[0.02 0.77 0.33 0.21]);
txtX = 0.021;  wX = 0.07;  hX = 0.03;  dy = 0.04;  sy = 0.91;
geoNames = {'a (O→A)','b (A→B)','c (B→C)','d (O→C)','e (A→P)', 'ϵ (BAP, deg)', 'δ (xOC, deg)'};
geoDefs  = {def_a, def_b, def_c, def_d, def_e, def_epsilon, def_delta};
for k = 1:4
    uicontrol('Style','text', ...
        'Units','normalized', ...
        'Position',[txtX sy-(k-1)*dy wX hX], ...
        'String',geoNames{k}, ...
        'HorizontalAlignment','right');
    geoEd(k) = uicontrol('Style','edit', ...
        'Units','normalized', ...
        'Position',[txtX+wX+0.01 sy-(k-1)*dy wX hX], ...
        'String',num2str(geoDefs{k}),'Callback',@updatePlot);
end
for k = 5:7
    uicontrol('Style','text', ...
        'Units','normalized', ...
        'Position',[0.17+txtX sy-(k-5)*dy wX hX], ...
        'String',geoNames{k}, ...
        'HorizontalAlignment','right');
    geoEd(k) = uicontrol('Style','edit', ...
        'Units','normalized', ...
        'Position',[0.17+txtX+wX+0.01 sy-(k-5)*dy wX hX], ...
        'String',num2str(geoDefs{k}),'Callback',@updatePlot);
end
if isOctave
    set(geoPanel,'BackgroundColor',[1 1 1]);
    for k = 1:7, set(geoEd(k),'FontSize',8); end
end

% ----------------------------------------------------------------
% 5) Mode Selection (Direct / Inverse)
% ----------------------------------------------------------------
modePanel = uipanel('Title','Mode','FontSize',10, ...
    'Position',[0.02 0.70 0.33 0.07]);
rad1 = uicontrol('Style','radiobutton','String','Direct', ...
    'Units','normalized','Position',[0.1 0.71 0.08 0.035], ...
    'Value',1,'Callback',@cbRad1);
rad2 = uicontrol('Style','radiobutton','String','Inverse', ...
    'Units','normalized','Position',[0.20 0.71 0.08 0.035], ...
    'Value',0,'Callback',@cbRad2);
if isOctave, set(modePanel,'BackgroundColor',[1 1 1]); end

% ----------------------------------------------------------------
% 6) Direct mode sliders (theta)
% ----------------------------------------------------------------
directPanel = uipanel('Title','Direct Mode Slider','FontSize',10, ...
    'Position',[0.02 0.61 0.33 0.08], ...
    'Visible','on');
if isOctave
    dirLabels = {'th'};
else
    dirLabels = {[char(952)]}; %θ
end
uicontrol('Parent',directPanel,'Style','text','Units','normalized', ...
    'Position',[0.04 0.25 0.1 0.6],'String',dirLabels{1},'HorizontalAlignment','left');
thetaSlider = uicontrol('Parent',directPanel,'Style','slider', ...
    'Units','normalized','Position',[0.10 0.25 0.75 0.7], ...
    'Min',-180,'Max',180,'Value',106,'SliderStep', [0.01/7.2, 0.1/7.2],...
    'Callback',@updatePlot);
thetaValTxt = uicontrol('Parent',directPanel,'Style','edit', ...
    'Units','normalized','Position',[0.88 0.25 0.1 0.6], ...
    'BackgroundColor',[1 1 1], ...
    'String',num2str(get(thetaSlider,'Value')),'HorizontalAlignment','left', ...
    'Callback',@cbThetaEdit);
if isOctave
    set(directPanel,'BackgroundColor',[1 1 1]);
    set(thetaValTxt,'FontSize',9);
end

% ----------------------------------------------------------------
% 7) Inverse mode sliders (alpha)
% ----------------------------------------------------------------
inversePanel = uipanel('Title','Inverse Mode Slider','FontSize',10, ...
    'Position',[0.02 0.52 0.33 0.08], ...
    'Visible','on');
uicontrol('Parent',inversePanel,'Style','text','Units','normalized', ...
    'Position',[0.04 0.25 0.1 0.6],'String','α','HorizontalAlignment','left');
alphaSlider = uicontrol('Parent',inversePanel,'Style','slider','Units','normalized', ...
    'Position',[0.10 0.25 0.75 0.7],'Min',-180,'Max',180,'Value',120,'SliderStep', [0.01/7.2, 0.1/7.2],...
    'Callback',@updatePlot);
alphaValTxt = uicontrol('Parent',inversePanel,'Style','edit', ...
    'Units','normalized','Position',[0.88 0.25 0.1 0.6], ...
    'BackgroundColor',[1 1 1], ...
    'String',num2str(get(alphaSlider,'Value')),'HorizontalAlignment','left', ...
    'Callback',@cbAlphaEdit);

% Start in Direct mode: disable the Inverse controls
set(alphaSlider, 'Enable','off');
set(alphaValTxt,'Enable','off');
if isOctave
    set(inversePanel,'BackgroundColor',[1 1 1]);
    set(thetaValTxt,'FontSize', 10 - 1*isOctave);
    set(alphaValTxt,'FontSize', 10 - 1*isOctave);
    set(thetaValTxt,'Position',[0.82 0.15 0.16 0.75]);
    set(alphaValTxt,'Position',[0.82 0.15 0.16 0.75]);
end

% ----------------------------------------------------------------
% 8) Display solutions panel (2 checkboxes)
% ----------------------------------------------------------------
solsPanel = uipanel('Title','Display solutions:','FontSize',10, ...
    'Position',[0.02 0.43 0.33 0.08], ...
    'Visible','on');
for i=1:2
    sols_checkbox(i) = uicontrol('Parent',solsPanel,'Units','normalized','Style','checkbox','Position',[0.2+0.2*i 0.3 0.7 0.6], ...
        'String',num2str(i),'Value',1,'Callback',@updatePlot);
end

% ----------------------------------------------------------------
% 9) Trajectory, animate button and info text
% ----------------------------------------------------------------
% Checkbox: show D = (OA) x (CB), its construction lines and its path
dtraj_checkbox = uicontrol('Style','checkbox','Position',[20 235 270 20], ...
    'String','Show D trajectory','Value',0, ...
    'Callback',@updatePlot);

% Checkbox: show P trajectory (for full crank rotation)
traj_checkbox = uicontrol('Style','checkbox','Position',[20 214 270 20], ...
    'String','Show P trajectory','Value',0, ...
    'Callback',@updatePlot);

animate_btn = uicontrol('Style','pushbutton','String','Animate', ...
    'Position',[20 180 295 30], 'Callback',@toggleAnimation);

info_text = uicontrol('Style','text','Position',[20 75 295 100], ...
    'FontSize', 10 - 1*isOctave, 'HorizontalAlignment','left');

% ----------------------------------------------------------------
% 10) Axes
% ----------------------------------------------------------------
ax = axes('Units','pixels','Position',[310 60 560 500]);
axis equal;grid on;
if isOctave
    xlabel(ax,'X','FontSize',14); ylabel(ax,'Y','FontSize',14);
    title(ax,'Four-Bar Linkage','FontSize',16);
else
    xlabel(ax,'X'); ylabel(ax,'Y');
    title(ax,'Four-Bar Linkage');
end
xlim([-1 1.4]*1.2);     %TO DO: add computeLimits subfunction and adjust dynamically
ylim([-1.2 1.2]*1.2);
hold(ax,'on');
lims=[get(ax,'XLim'), get(ax,'YLim')];

% ----------------------------------------------------------------
% 11) Store state
% ----------------------------------------------------------------
data.name          = 'Fourbar Linkage';
data.type          = 2.01;
data.info          = 'Lorem ipsum';
data.author        = 'Lionel Birglen';
data.date          = '20260628';
data.version       = 0.1;
data.geoEd         = geoEd;
data.modeStr       = 'Direct';  
data.rad1          = rad1;
data.rad2          = rad2;
data.directPanel   = directPanel;
data.inversePanel  = inversePanel;
data.thetaSl       = thetaSlider;
data.thetaTxt      = thetaValTxt;
data.alphaSl       = alphaSlider;
data.alphaTxt      = alphaValTxt;
data.sols_checkbox  = sols_checkbox;
data.traj_checkbox  = traj_checkbox;
data.dtraj_checkbox = dtraj_checkbox;
data.animateFlag   = false;
data.timerObj      = [];
data.animateBtn    = animate_btn;
data.th_offset     = get(thetaSlider,'Value');
data.alph_offset   = get(alphaSlider,'Value');
data.info_text     = info_text;
data.ax            = ax;
data.limits       = lims;
data.userZoomed   = false;  % true once the user has zoomed/panned
data.firstPlot    = true;   % next redraw applies the geometry-based limits
data.lastLimits   = [];     % limits in effect after the last redraw
guidata(hFig,data);

% In Octave, uicontrols and uipanels inherit the system grey background.
% Set them all to white in one sweep after all widgets are created.
if isOctave
    objs = findobj(hFig,'Type','uicontrol');
    for oi = 1:numel(objs)
        style = get(objs(oi),'Style');
        if ~strcmp(style,'pushbutton')
            set(objs(oi),'BackgroundColor',[1 1 1]);
        end
    end
    objs = findobj(hFig,'Type','uipanel');
    for oi = 1:numel(objs)
        set(objs(oi),'BackgroundColor',[1 1 1]);
    end
end

updatePlot([],[]);


% ================================================================
%  Callbacks
% ================================================================
 
    function cbRad1(~,~)
        set(rad1,'Value',1);  set(rad2,'Value',0);
        data = guidata(hFig);
        data.modeStr = 'Direct';
        set(data.thetaSl,  'Enable','on');
        set(data.thetaTxt, 'Enable','on');
        set(data.alphaSl,  'Enable','off');
        set(data.alphaTxt, 'Enable','off');
        g = readGeo(data);
        alpha_current = get(data.alphaSl,'Value');
        invSol = fourbar_inverse_kinematics(g, deg2rad(alpha_current));
        if ~isempty(invSol)
            set(data.thetaSl,  'Value', rad2deg(invSol(1).theta));
            set(data.thetaTxt, 'String', sprintf('%.1f', rad2deg(invSol(1).theta)));
        end
        guidata(hFig, data);
        updatePlot([], []);
    end

    function cbRad2(~,~)
        set(rad1,'Value',0);  set(rad2,'Value',1);
        data = guidata(hFig);
        data.modeStr = 'Inverse';
        set(data.thetaSl,  'Enable','off');
        set(data.thetaTxt, 'Enable','off');
        set(data.alphaSl,  'Enable','on');
        set(data.alphaTxt, 'Enable','on');
        g = readGeo(data);
        theta_current = get(data.thetaSl,'Value');
        solDir = fourbar_direct_kinematics(g, deg2rad(theta_current));
        if ~isempty(solDir)
            set(data.alphaSl,  'Value', rad2deg(solDir(1).alpha));
            set(data.alphaTxt, 'String', sprintf('%.2f', rad2deg(solDir(1).alpha)));
        end
        guidata(hFig, data);
        updatePlot([], []);
    end

    function cbThetaEdit(~,~)
        % Pull GUI data
        data = guidata(hFig);

        % Pull modified theta value
        theta = str2num(get(data.thetaTxt,'String'));

        % Push to slider
        set(data.thetaSl,'Value',theta);

        % Redraw
        updatePlot([], []);
    end

    function cbAlphaEdit(~,~)
        % Pull GUI data
        data = guidata(hFig);

        % Pull modified theta value
        alpha = str2num(get(data.alphaTxt,'String'));

        % Push to slider
        set(data.alphaSl,'Value',alpha);

        % Redraw
        updatePlot([], []);
    end

    function g = readGeo(data)
        % Helper: read geometry edit boxes, convert deg->rad for fields 6-7
        g = zeros(1,7);
        for ii = 1:7
            g(ii) = str2double(get(data.geoEd(ii),'String'));
        end
        g(6) = deg2rad(g(6));
        g(7) = deg2rad(g(7));
    end

    function updatePlot(~,~)
        data = guidata(hFig);
        try
            % 1) Read geometry from the 8 edit boxes (same order as before)
            g = zeros(1,7);
            for ii = 1:7
                g(ii) = str2double(get(data.geoEd(ii),'String'));
            end
            % Conversion to radians from the input values in degrees
            g(6)=deg2rad(g(6));
            g(7)=deg2rad(g(7));

            % Compute axis limits
            a=g(1); b=g(2); c=g(3); d=g(4);
            R = max([a+b, c+d, a+c, b+d]);
            margin = 0.01 * R;
            data.limits=[-R - margin, R + margin,-R - margin, R + margin];

            % 2) Find which solutions the user wants to see (1..4)
            selSol = [];
            if isfield(data,'sols_checkbox')
                for k = 1:numel(data.sols_checkbox)
                    if get(data.sols_checkbox(k),'Value')
                        selSol(end+1) = k;
                    end
                end
            end

            % 3) Common plotting options for fourbar_plot
            opts.ax         = data.ax;
            % Save current axes limits BEFORE clearing (user may have zoomed/panned)
            prevXLim = xlim(data.ax);
            prevYLim = ylim(data.ax);
            % A zoom/pan is detected when the current limits differ from
            % the ones in effect after the previous redraw. It is then
            % remembered (data.userZoomed) until View > Reset View or a
            % new session, so the user's view is kept while the linkage
            % moves. Comparing with the previous redraw (not with the
            % geometry-based limits) also lets a geometry change update
            % the view when the user has not zoomed.
            if ~isfield(data,'userZoomed'), data.userZoomed = false; end
            if ~isfield(data,'firstPlot'),  data.firstPlot  = true;  end
            if ~isfield(data,'lastLimits'), data.lastLimits = [];    end
            if data.firstPlot
                data.userZoomed = false;
            elseif ~data.userZoomed && numel(data.lastLimits) == 4
                tol = max(abs(data.lastLimits(2)-data.lastLimits(1)), eps) * 1e-3;
                data.userZoomed = any(abs([prevXLim prevYLim] - data.lastLimits) > tol);
            end
            userZoomed = data.userZoomed;
            opts.clearAxes  = true;
            opts.limits     = [];  % limits applied after plot, not inside
            opts.showLabels = true;
            if ~isempty(selSol)
                opts.solutions = selSol;
            end
            modeStr = data.modeStr;
            if strcmp(modeStr,'Direct')
                theta = get(data.thetaSl,'Value');
                set(data.thetaTxt,'String',sprintf('%.1f',theta));
                if exist('fourbar_plot','file') ~= 2
                    error('fourbar_plot.m is not on the path.');
                end
                [~, sol] = fourbar_plot(g, 'direct', deg2rad(theta), opts);
                for i = 1:2
                    if i <= numel(sol) && (~isfield(sol(i),'valid') || sol(i).valid)
                        set(data.sols_checkbox(i),'Enable','on');
                    else
                        set(data.sols_checkbox(i),'Enable','off');
                    end
                end
                info_str = sprintf('Direct mode:\nθ = %.2f°\n',theta);
                for i = 1:numel(sol)
                    if ~isfield(sol(i),'valid') || sol(i).valid
                        Pp = sol(i).P;
                        al=rad2deg(sol(i).alpha);
                        ph=rad2deg(sol(i).phi);
                        info_str = sprintf('%sSol %d: P=[%.1f;%.1f] α=%.1f° Φ=%.1f°\n', ...
                            info_str, i, Pp(1), Pp(2),al,ph);
                    else
                        info_str = sprintf('%sSol %d: invalid\n', info_str, i);
                    end
                end
                set(data.info_text,'String',info_str);

            else
                alpha = get(data.alphaSl,'Value');
                set(data.alphaTxt,'String',sprintf('%.2f',alpha));
                if exist('fourbar_plot','file') ~= 2
                    error('fourbar_plot.m is not on the path.');
                end
                [~, sol] = fourbar_plot(g, 'inverse', deg2rad(alpha), opts);
                for i = 1:2
                    if i <= numel(sol) && (~isfield(sol(i),'valid') || sol(i).valid)
                        set(data.sols_checkbox(i),'Enable','on');
                    else
                        set(data.sols_checkbox(i),'Enable','off');
                    end
                end
                info_str = sprintf('Inverse mode:\nα = %.2f\n',alpha);
                for i = 1:numel(sol)
                    if ~isfield(sol(i),'valid') || sol(i).valid
                        Pp = sol(i).P;
                        th=rad2deg(sol(i).theta);
                        ph=rad2deg(sol(i).phi);
                        info_str = sprintf('%sSol %d: P=[%.1f;%.1f] θ=%.1f° Φ=%.1f°\n', ...
                            info_str, i, Pp(1), Pp(2),th,ph);
                    else
                        info_str = sprintf('%sSol %d: invalid\n', info_str, i);
                    end
                end
                set(data.info_text,'String',info_str);
            end
            if get(traj_checkbox,'Value')
                thetas = linspace(0, 2*pi, 360);
                P_traj = nan(2, numel(thetas));
                P_traj_alt = nan(2, numel(thetas));
                for ii = 1:numel(thetas)
                    try
                        solDir = fourbar_direct_kinematics(g, thetas(ii));
                        P_traj(:,ii) = solDir(1).P;
                        P_traj_alt(:,ii) = solDir(2).P;
                    end
                end
                if any(selSol==1)
                    lw_traj = 1.0; if isOctave, lw_traj = 0.4; end
                    plot(ax, P_traj(1,:), P_traj(2,:), 'k:', 'LineWidth',lw_traj);
                end
                if any(selSol==2)
                    lw_traj = 1.0; if isOctave, lw_traj = 0.4; end
                    plot(ax, P_traj_alt(1,:), P_traj_alt(2,:), 'k--', 'LineWidth',lw_traj);
                end
            end
            % --- Point D = (OA) x (CB): construction lines and path -----
            if get(data.dtraj_checkbox,'Value')
                lw_fine = 0.5; if isOctave, lw_fine = 0.25; end
                lw_traj = 1.0; if isOctave, lw_traj = 0.4; end
                ms_D    = 8;   if isOctave, ms_D    = 2;    end
                fs_D    = 10;  if isOctave, fs_D    = 14;   end
                styles  = {'k:','k--'};
                dxl = g(1)/20;     % label offset, as in fourbar_plot (a/20)
                for i = selSol
                    if i > numel(sol) || (isfield(sol(i),'valid') && ~sol(i).valid)
                        continue;
                    end
                    Pos = sol(i).Positions;
                    D = lineIntersect(Pos.O, Pos.A, Pos.C, Pos.B);
                    if all(isfinite(D))
                        drawThroughPoints(data.ax, [Pos.O Pos.A D], lw_fine);
                        drawThroughPoints(data.ax, [Pos.C Pos.B D], lw_fine);
                        plot(data.ax, D(1), D(2), 'kx', 'MarkerSize', ms_D, ...
                            'LineWidth', lw_traj);
                        text(data.ax, D(1)+dxl, D(2)+dxl, 'D', 'FontSize', fs_D, ...
                            'Color','k', 'HorizontalAlignment','left', ...
                            'VerticalAlignment','bottom');
                    end
                    % Path of D over a full crank rotation (branch i),
                    % computed once per geometry (cached)
                    Dpaths = dPaths(g);
                    D_traj = Dpaths(:,:,i);
                    % Break the curve where D goes to infinity (O-A and C-B
                    % parallel), so no segment is drawn across the view
                    Rv = max(abs(data.limits));
                    far  = any(abs(D_traj) > 50*Rv, 1);
                    D_traj(:, far) = NaN;
                    jump = [false, sqrt(sum(diff(D_traj,1,2).^2,1)) > Rv];
                    D_traj(:, jump) = NaN;
                    plot(data.ax, D_traj(1,:), D_traj(2,:), styles{min(i,2)}, ...
                        'LineWidth', lw_traj);
                end
            end

            if userZoomed
                xlim(data.ax, prevXLim);
                ylim(data.ax, prevYLim);
            else
                axis(data.ax,'equal');
                xlim(data.ax, data.limits(1:2));
                ylim(data.ax, data.limits(3:4));
            end
            % Store the view state. The GUI data is re-read first so that
            % fields changed meanwhile by other callbacks (e.g. the
            % animation flag set by the Stop button) are not overwritten.
            dView = guidata(hFig);
            dView.limits     = data.limits;
            dView.userZoomed = userZoomed;
            dView.firstPlot  = false;
            dView.lastLimits = [xlim(data.ax) ylim(data.ax)];
            guidata(hFig, dView);
            drawnow();

        catch ME
            cla(ax);
            text(0,0,'Unreachable','Parent',ax,'Color','r','FontSize',14,'HorizontalAlignment','center');
            set(info_text,'String','Unreachable configuration');
        end
    end

    % ---- Animation ---------------------------------------------------
    function toggleAnimation(~,~)
        data = guidata(hFig);
        if data.animateFlag
            % --- Stop ---
            if ~isOctave
                if ~isempty(data.timerObj) && isvalid(data.timerObj)
                    stop(data.timerObj);
                    delete(data.timerObj);
                end
                data.timerObj = [];
            end
            data.animateFlag = false;
            set(data.animateBtn,'String','Animate');
            guidata(hFig,data);
        else
            % --- Start ---
            data.th_offset   = get(data.thetaSl,'Value');
            data.al_offset   = get(data.alphaSl,'Value');
            data.time_offset = now*24*3600;
            data.animateFlag = true;
            set(data.animateBtn,'String','Stop');
            if ~isOctave
                % MATLAB: timer-based, non-blocking
                if ~isempty(data.timerObj) && isvalid(data.timerObj)
                    stop(data.timerObj); delete(data.timerObj);
                end
                t = timer('ExecutionMode','fixedRate', ...
                    'Period',0.033, ...
                    'TimerFcn',@animateStep);
                data.timerObj = t;
                guidata(hFig,data);
                start(t);
            else
                guidata(hFig,data);
                while true
                    drawnow();           % process GUI events (Stop button)
                    data = guidata(hFig);
                    if ~data.animateFlag || ~ishandle(hFig)
                        break;
                    end
                    animateStep([],[]);
                    pause(0.033);        % ~30 fps
                end
            end
        end
    end

    function animateStep(~,~)
        data = guidata(hFig);
        mode = data.modeStr;
        tnow = now*24*3600;
        speed = 10;  % degrees per second
        switch mode
            case 'Direct'
                th = mod(data.th_offset + speed*(tnow-data.time_offset) + 180, 360) - 180;
                set(data.thetaSl,'Value',th);
            case 'Inverse'
                al = mod(data.al_offset + speed*(tnow-data.time_offset) + 180, 360) - 180;
                set(data.alphaSl,'Value',al);
        end
        updatePlot();
    end

    % ---- File callbacks ----------------------------------------------
    function cbOpen(hFig)
        warning('off','all');
        [f,p] = uigetfile({'*.mat','MAT-file (*.mat)'}, 'Open Session File');
        warning('on','all');
        if isequal(f,0), return; end
        full = fullfile(char(p), char(f));
        warning('off','all');
        s = load(full, '-mat');
        warning('on','all');
        if isfield(s,'session')
            session = s.session;
        elseif isfield(s,'data') && isstruct(s.data) && isfield(s.data,'geo')
            d = s.data;
            session.geo     = d.geo;
            session.theta   = 0;
            session.alpha   = 0;
            session.modeStr = 'Direct';
            session.solsVisible = ones(1,2);
        else
            errordlg('Unrecognised session file format.','Open Error');
            return;
        end
        data = guidata(hFig);
        geo = session.geo;
        geoStrs = {'a','b','c','d','e','ε (°)','δ (°)'};
        for ii = 1:min(7, numel(geo))
            v = geo(ii);
            set(data.geoEd(ii), 'String', num2str(v));
        end
        set(data.thetaSl,  'Value', session.theta);
        set(data.thetaTxt, 'String', sprintf('%.1f', session.theta));
        set(data.alphaSl,  'Value', session.alpha);
        set(data.alphaTxt, 'String', sprintf('%.2f', session.alpha));
        data.modeStr = session.modeStr;
        if strcmp(session.modeStr,'Direct')
            set(data.rad1,'Value',1); set(data.rad2,'Value',0);
            set(data.thetaSl,'Enable','on');  set(data.thetaTxt,'Enable','on');
            set(data.alphaSl,'Enable','off'); set(data.alphaTxt,'Enable','off');
        else
            set(data.rad1,'Value',0); set(data.rad2,'Value',1);
            set(data.thetaSl,'Enable','off'); set(data.thetaTxt,'Enable','off');
            set(data.alphaSl,'Enable','on');  set(data.alphaTxt,'Enable','on');
        end
        for ii = 1:min(numel(data.sols_checkbox), numel(session.solsVisible))
            set(data.sols_checkbox(ii),'Value', session.solsVisible(ii));
        end
        if isfield(session,'showTraj')
            set(data.traj_checkbox,'Value', session.showTraj);
        end
        if isfield(session,'showDTraj')
            set(data.dtraj_checkbox,'Value', session.showDTraj);
        end
        % View: first draw the new session with its geometry-based
        % limits; then restore the saved view, and keep it as a user
        % view, only if it differs from those limits (i.e. the session
        % was saved zoomed or panned)
        data.userZoomed = false;
        data.firstPlot  = true;
        guidata(hFig, data);
        updatePlot([],[]);
        if isfield(session,'axesXLim') && isfield(session,'axesYLim') && ...
                numel(session.axesXLim) == 2 && numel(session.axesYLim) == 2
            data = guidata(hFig);
            cur = [xlim(data.ax) ylim(data.ax)];
            sav = [session.axesXLim(:).' session.axesYLim(:).'];
            tol = max(abs(cur(2)-cur(1)), eps) * 1e-3;
            if any(abs(sav - cur) > tol)
                xlim(data.ax, session.axesXLim);
                ylim(data.ax, session.axesYLim);
                data.userZoomed = true;
                data.lastLimits = [xlim(data.ax) ylim(data.ax)];
                guidata(hFig, data);
            end
        end
    end

    function cbSave(hFig)
        warning('off','all');
        [f,p] = uiputfile({'*.mat','MAT-file (*.mat)'}, 'Save Session As', 'fourbar_session.mat');
        warning('on','all');
        if isequal(f,0), return; end
        target = fullfile(char(p),char(f));
        data = guidata(hFig);
        session.geo      = zeros(1,7);
        for ii = 1:7
            session.geo(ii) = str2double(get(data.geoEd(ii),'String'));
        end
        session.theta    = get(data.thetaSl, 'Value');
        session.alpha    = get(data.alphaSl, 'Value');
        session.modeStr  = data.modeStr;
        session.solsVisible = zeros(1, numel(data.sols_checkbox));
        for ii = 1:numel(data.sols_checkbox)
            session.solsVisible(ii) = get(data.sols_checkbox(ii),'Value');
        end
        session.showTraj    = get(data.traj_checkbox,'Value');
        session.showDTraj   = get(data.dtraj_checkbox,'Value');
        session.axesXLim    = xlim(data.ax);
        session.axesYLim    = ylim(data.ax);
        save(target, 'session', '-mat', '-v6');
    end

    function cbExportPNG(hFig)
        warning('off','all');
        [f,p] = uiputfile({'*.png','PNG Image (*.png)'}, 'Export As');
        warning('on','all');
        if isequal(f,0), return; end
        target = fullfile(char(p),char(f));
        if isOctave
            origPU = get(hFig,'PaperUnits');
            origPP = get(hFig,'PaperPosition');
            origPPM = get(hFig,'PaperPositionMode');
            pos = get(hFig,'Position');  % pixels
            wIn = pos(3)/96;
            hIn = pos(4)/96;
            set(hFig,'PaperUnits','inches', ...
                     'PaperPosition',[0 0 wIn hIn], ...
                     'PaperPositionMode','manual');
            data = guidata(hFig);
            axH = data.ax;
            scatH  = findall(axH, 'Type','scatter');
            crossH = findall(axH, 'Type','line', 'Marker','x');
            for kk = 1:numel(scatH)
                set(scatH(kk), 'SizeData',  get(scatH(kk),'SizeData')  * 16);
            end
            for kk = 1:numel(crossH)
                set(crossH(kk),'MarkerSize',get(crossH(kk),'MarkerSize') * 4);
                set(crossH(kk),'LineWidth', get(crossH(kk),'LineWidth')  * 4);
            end
            print(hFig, '-dpng', '-r96', target);
            for kk = 1:numel(scatH)
                set(scatH(kk), 'SizeData',  get(scatH(kk),'SizeData')  / 16);
            end
            for kk = 1:numel(crossH)
                set(crossH(kk),'MarkerSize',get(crossH(kk),'MarkerSize') / 4);
                set(crossH(kk),'LineWidth', get(crossH(kk),'LineWidth')  / 4);
            end
            set(hFig,'PaperUnits',origPU, ...
                     'PaperPosition',origPP, ...
                     'PaperPositionMode',origPPM);
        elseif exist('exportgraphics','file')
            exportgraphics(hFig, target);
        else
            print(hFig, '-dpng', '-r300', target);
        end
    end

    function cbExportEPSPDF(hFig)
        warning('off','all');
        [f,p] = uiputfile({'*.eps','EPS Vector (*.eps)'}, 'Export As');
        warning('on','all');
        if isequal(f,0), return; end
        target = fullfile(char(p),char(f));
        if exist('exportgraphics','file')
            exportgraphics(hFig, target); % export full figure
        else
            print(hFig, '-depsc', '-r300', target);
        end
        unix(strcat(['epstopdf ',target]));
    end

    function cbPrint(hFig)
        printdlg(hFig);
    end

    function cbExit(hFig)
        choice = questdlg('Are you sure you want to exit?', 'Exit', ...
            'Yes','No','No');
        if strcmp(choice,'Yes')
            close(hFig);
        end
    end

    function cbResetView(hFig)
        % Drop any user zoom/pan and redraw with the geometry-based limits
        data = guidata(hFig);
        data.userZoomed = false;
        data.firstPlot  = true;
        guidata(hFig, data);
        try, zoom(data.ax,'reset'); catch, end   % forget zoom history
        updatePlot([],[]);
    end

    function cbPreferences(hFig)
        ax = findall(hFig,'Type','axes');
        if isempty(ax)
            return;
        end
        states = get(ax,'XGrid'); % char for single handle, or cellstr for multiple
        if ischar(states)
            states = {states};
        end
        oncount = sum(strcmp(states,'on'));
        if oncount >= numel(ax)/2
            newState = 'off';
        else
            newState = 'on';
        end

        for k = 1:numel(ax)
            try
                grid(ax(k), newState);
            catch
                if strcmpi(newState,'on')
                    grid(ax(k),'on');
                else
                    grid(ax(k),'off');
                end
            end
        end
    end

end


% ================================================================
%  Local functions (outside the nested scope)
% ================================================================
function X = lineIntersect(P1, P2, P3, P4)
% Intersection of line (P1,P2) with line (P3,P4); [NaN;NaN] if the lines
% are parallel (or a line is degenerate)
d1 = P2(:) - P1(:);  d2 = P4(:) - P3(:);
den = d1(1)*d2(2) - d1(2)*d2(1);
if ~all(isfinite([d1; d2])) || abs(den) <= 1e-12 * norm(d1) * norm(d2) ...
        || norm(d1) == 0 || norm(d2) == 0
    X = [NaN; NaN];
    return;
end
w = P3(:) - P1(:);
X = P1(:) + (w(1)*d2(2) - w(2)*d2(1)) / den * d1;
end

function P = dPaths(g)
% Path of D = (OA) x (CB) over a full crank rotation, for both assembly
% modes: 2 x 721 x 2 array (NaN where the mode does not assemble or D is
% at infinity). Cached: recomputed only when the geometry g changes, so
% moving the slider or animating stays fast.
persistent gLast PLast
if ~isempty(gLast) && isequal(gLast, g)
    P = PLast;
    return;
end
thetas = linspace(0, 2*pi, 721);
P = nan(2, numel(thetas), 2);
for ii = 1:numel(thetas)
    solDir = fourbar_direct_kinematics(g, thetas(ii));
    for b = 1:min(2, numel(solDir))
        if solDir(b).valid
            Q = solDir(b).Positions;
            P(:,ii,b) = lineIntersect(Q.O, Q.A, Q.C, Q.B);
        end
    end
end
gLast = g;
PLast = P;
end

function drawThroughPoints(ax, pts, lw)
% Fine black line through collinear points (columns of pts), drawn
% between the two extreme ones so that it passes through all of them
d = pts(:,2) - pts(:,1);
if norm(d) == 0, d = pts(:,3) - pts(:,1); end
if norm(d) == 0, return; end
d = d / norm(d);
t = d.' * (pts - pts(:,1) * ones(1, size(pts,2)));
p0 = pts(:,1) + min(t) * d;
p1 = pts(:,1) + max(t) * d;
plot(ax, [p0(1) p1(1)], [p0(2) p1(2)], 'k-', 'LineWidth', lw);
end
