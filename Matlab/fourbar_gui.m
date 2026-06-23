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
% BY:
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Last Update: 2025/06/03
% Contact: lionel.birglen@polymtl.ca
%
% Code provided under GNU Affero General Public License v3.0


% ---------------------------------------------------------------------
% GUI COMPONENTS
% ---------------------------------------------------------------------

%— Create figure & controls —
% Detect Octave (timer is not available there; animation uses a drawnow loop)
isOctave = exist('OCTAVE_VERSION','builtin') ~= 0;

% In Octave, 'MenuBar','none' hides custom uimenu items too; use 'figure' there.
if isOctave
    hFig = figure('Name','Four-Bar Linkage GUI','NumberTitle','off', ...
        'MenuBar','figure','ToolBar','figure','Position',[300 100 900 600]);
else
    hFig = figure('Name','Four-Bar Linkage GUI','NumberTitle','off', ...
        'MenuBar','none','ToolBar','figure','Position',[300 100 900 600]);
end
if isOctave
    set(hFig,'Color',[1 1 1]);
    set(0,'DefaultUicontrolBackgroundColor',[1 1 1]);
end

% Custom menu bar
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


% ---- Callback implementations ----
    function cbOpen(hFig)
        % Load a session .mat file saved by cbSave and restore the GUI.
        % Compatible with files saved by both MATLAB and Octave.
        warning('off','all');
        [f,p] = uigetfile({'*.mat','MAT-file (*.mat)'}, 'Open Session File');
        warning('on','all');
        if isequal(f,0), return; end
        full = fullfile(char(p), char(f));
        warning('off','all');
        s = load(full, '-mat');
        warning('on','all');

        % Accept both new format ('session') and legacy format ('data' with .geo)
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

        % Restore into the live GUI
        data = guidata(hFig);
        geo = session.geo;
        geoStrs = {'a','b','c','d','e','ε (°)','δ (°)'};
        for ii = 1:min(7, numel(geo))
            v = geo(ii);
            % fields 6-7 were stored in degrees already (session format)
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
        guidata(hFig, data);
        updatePlot([],[]);
    end

    function cbSave(hFig)
        % Save session to a .mat file.
        % Only pure numeric/string data is saved (no graphics handles),
        % so the file is compatible with both MATLAB and Octave.
        warning('off','all');
        [f,p] = uiputfile({'*.mat','MAT-file (*.mat)'}, 'Save Session As', 'fourbar_session.mat');
        warning('on','all');
        if isequal(f,0), return; end
        target = fullfile(char(p),char(f));
        data = guidata(hFig);
        % Build a clean session struct with only portable data
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
        session.showTraj = get(data.traj_checkbox,'Value');
        save(target, 'session', '-mat', '-v6');
    end

    function cbExportPNG(hFig)
        % Export current figure to PNG
        warning('off','all');
        [f,p] = uiputfile({'*.png','PNG Image (*.png)'}, 'Export As');
        warning('on','all');
        if isequal(f,0), return; end
        target = fullfile(char(p),char(f));
        if isOctave
            % In Octave, print font scaling is controlled by PaperPosition.
            % Save and override paper settings so the output matches the
            % screen at a fixed pixel size, then restore.
            origPU = get(hFig,'PaperUnits');
            origPP = get(hFig,'PaperPosition');
            origPPM = get(hFig,'PaperPositionMode');
            % Match figure's screen size in inches at 96 dpi
            pos = get(hFig,'Position');  % pixels
            wIn = pos(3)/96;
            hIn = pos(4)/96;
            set(hFig,'PaperUnits','inches', ...
                     'PaperPosition',[0 0 wIn hIn], ...
                     'PaperPositionMode','manual');
            % Scale up markers before print (they render too small at screen dpi)
            data = guidata(hFig);
            axH = data.ax;
            scatH  = findall(axH, 'Type','scatter');
            crossH = findall(axH, 'Type','line', 'Marker','x');
            % Scale up markers element-by-element (get() returns cell when >1 object)
            for kk = 1:numel(scatH)
                set(scatH(kk), 'SizeData',  get(scatH(kk),'SizeData')  * 16);
            end
            for kk = 1:numel(crossH)
                set(crossH(kk),'MarkerSize',get(crossH(kk),'MarkerSize') * 4);
                set(crossH(kk),'LineWidth', get(crossH(kk),'LineWidth')  * 4);
            end
            print(hFig, '-dpng', '-r96', target);
            % Restore markers
            for kk = 1:numel(scatH)
                set(scatH(kk), 'SizeData',  get(scatH(kk),'SizeData')  / 16);
            end
            for kk = 1:numel(crossH)
                set(crossH(kk),'MarkerSize',get(crossH(kk),'MarkerSize') / 4);
                set(crossH(kk),'LineWidth', get(crossH(kk),'LineWidth')  / 4);
            end
            % Restore paper settings
            set(hFig,'PaperUnits',origPU, ...
                     'PaperPosition',origPP, ...
                     'PaperPositionMode',origPPM);
        elseif exist('exportgraphics','file')
            exportgraphics(hFig, target);
        else
            print(hFig, '-dpng', '-r300', target);
        end
        %localInfoAlert(hFig, sprintf('Exported to: %s', target), 'Export');
    end

    function cbExportEPSPDF(hFig)
        % Export current figure to EPS and PDF
        warning('off','all');
        [f,p] = uiputfile({'*.eps','EPS Vector (*.eps)'}, 'Export As');
        warning('on','all');
        if isequal(f,0), return; end
        target = fullfile(char(p),char(f));
        if exist('exportgraphics','file')
            exportgraphics(hFig, target); % export full figure
        else
            % Fallback for older MATLAB: use print
            print(hFig, '-depsc', '-r300', target);
        end
        %localInfoAlert(hFig, sprintf('Exported to: %s', target), 'Export');
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
        ax = findall(hFig,'Type','axes');
        for k = 1:numel(ax)
            try
                axis(ax(k),'auto');
            catch, end

            % Optional: reset zoom/pan
            try, zoom(ax(k),'out'); catch, end
        end
        drawnow();
        %localInfoAlert(hFig, 'View has been reset.', 'Reset View');
    end

    function cbPreferences(hFig)
        % Simple Preferences placeholder: toggle grid on all axes
        ax = findall(hFig,'Type','axes');
        if isempty(ax)
            %localInfoAlert(hFig,'No axes found to configure.','Preferences');
            return;
        end

        % Determine majority grid state across axes
        states = get(ax,'XGrid'); % char for single handle, or cellstr for multiple
        if ischar(states)
            states = {states};
        end
        oncount = sum(strcmp(states,'on'));
        if oncount >= numel(ax)/2
            % Majority ON -> switch OFF
            newState = 'off';
        else
            % Majority OFF -> switch ON
            newState = 'on';
        end

        for k = 1:numel(ax)
            try
                grid(ax(k), newState);
            catch
                % Fallback for older MATLAB versions
                if strcmpi(newState,'on')
                    grid(ax(k),'on');
                else
                    grid(ax(k),'off');
                end
            end
        end

        %localInfoAlert(hFig, sprintf('Grid turned %s for %d axes.', newState, numel(ax)), 'Preferences');
    end


%------------------------------------------------------
% Geometry Panel
%------------------------------------------------------
uipanel('Title','Geometry', ...
    'FontSize',10, ...
    'Position',[0.02 0.77 0.33 0.21]);

uipanel('Title','Geometry', ...
    'FontSize',10, ...
    'Position',[0.02 0.77 0.33 0.21]);

% a,b,c,d,e,h,eta labels and edit boxes
txtX = 0.021;  wX = 0.07;  hX = 0.03;  dy = 0.04;  sy = 0.91;
names = {'a (O→A)','b (A→B)','c (B→C)','d (O→C)','e (A→P)', 'ϵ (BAP, deg)', 'δ (xOC, deg)'};
for k = 1:4
    uicontrol('Style','text', ...
        'Units','normalized', ...
        'Position',[txtX sy-(k-1)*dy wX hX], ...
        'String',names{k}, ...
        'HorizontalAlignment','right');
    geoEd(k) = uicontrol('Style','edit', ...
        'Units','normalized', ...
        'Position',[txtX+wX+0.01 sy-(k-1)*dy wX hX], ...
        'String','0','Callback',@updatePlot);
end
for k = 5:7
    uicontrol('Style','text', ...
        'Units','normalized', ...
        'Position',[0.17+txtX sy-(k-5)*dy wX hX], ...
        'String',names{k}, ...
        'HorizontalAlignment','right');
    geoEd(k) = uicontrol('Style','edit', ...
        'Units','normalized', ...
        'Position',[0.17+txtX+wX+0.01 sy-(k-5)*dy wX hX], ...
        'String','0','Callback',@updatePlot);
end

% Preload some default geometry
set(geoEd(1),'String','0.81');  % a
set(geoEd(2),'String','0.88');  % b
set(geoEd(3),'String','0.92');  % c
set(geoEd(4),'String','1.51');  % d
set(geoEd(5),'String','0.80');  % e
set(geoEd(6),'String','30');  % epsilon = pi/6
set(geoEd(7),'String','-10');  % delta = -10*pi/180
%set(geoEd(6),'String','0.5236');  % epsilon = pi/6
%set(geoEd(7),'String','-0.1745');  % delta = -10*pi/180


%------------------------------------------------------
% Mode Selection (Direct / Inverse)
%------------------------------------------------------
uipanel('Title','Mode','FontSize',10, ...
    'Position',[0.02 0.70 0.33 0.07]);
% Note: uibuttongroup SelectedObject is not reliable in Octave.
% We use two plain radiobuttons with individual callbacks instead,
% and track the active mode via data.modeStr ('Direct' or 'Inverse').
rad1 = uicontrol('Style','radiobutton','String','Direct', ...
    'Units','normalized','Position',[0.1 0.71 0.08 0.035], ...
    'Value',1,'Callback',@cbRad1);
rad2 = uicontrol('Style','radiobutton','String','Inverse', ...
    'Units','normalized','Position',[0.20 0.71 0.08 0.035], ...
    'Value',0,'Callback',@cbRad2);


%------------------------------------------------------
% Direct Mode Controls
%------------------------------------------------------
directPanel = uipanel('Title','Direct Mode Slider','FontSize',10, ...
    'Position',[0.02 0.61 0.33 0.08], ...
    'Visible','on');
uicontrol('Parent',directPanel,'Style','text','Units','normalized', ...
    'Position',[0.04 0.25 0.1 0.6],'String','θ','HorizontalAlignment','left');
thetaSlider = uicontrol('Parent',directPanel,'Style','slider', ...
    'Units','normalized','Position',[0.10 0.25 0.75 0.7], ...
    'Min',-180,'Max',180,'Value',106,'SliderStep', [0.01/7.2, 0.1/7.2],...
    'Callback',@updatePlot);
thetaValTxt = uicontrol('Parent',directPanel,'Style','edit', ...
    'Units','normalized','Position',[0.88 0.25 0.1 0.6], ...
    'BackgroundColor',[1 1 1], ...
    'String',num2str(get(thetaSlider,'Value')),'HorizontalAlignment','left', ...
    'Callback',@cbThetaEdit);

%------------------------------------------------------
% Inverse Mode Controls
%------------------------------------------------------
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

if isOctave
    set(thetaValTxt,'FontSize', 10 - 1*isOctave);
    set(alphaValTxt,'FontSize', 10 - 1*isOctave);
    set(thetaValTxt,'Position',[0.82 0.15 0.16 0.75]);
    set(alphaValTxt,'Position',[0.82 0.15 0.16 0.75]);
end

% Start in Direct mode: disable the Inverse controls
set(alphaSlider, 'Enable','off');
set(alphaValTxt,'Enable','off');

% Checkbox: display solutions
solsPanel = uipanel('Title','Display solutions:','FontSize',10, ...
    'Position',[0.02 0.43 0.33 0.08], ...
    'Visible','on');
for i=1:2
    sols_checkbox(i) = uicontrol('Parent',solsPanel,'Units','normalized','Style','checkbox','Position',[0.2+0.2*i 0.3 0.7 0.6], ...
        'String',num2str(i),'Value',1,'Callback',@updatePlot);
end

% Define color schemes
colors = [1 0 0;0 0 1];

% Checkbox: show P trajectory (for full crank rotation)
traj_checkbox = uicontrol('Style','checkbox','Position',[20 235 270 20], ...
    'String','Show P trajectory','Value',0, ...
    'Callback',@updatePlot);

% Animate button
animate_btn = uicontrol('Style','pushbutton','String','Animate', ...
    'Position',[20 200 295 30], 'Callback',@toggleAnimation);

% Text area for φ/theta/α & P coords
info_text = uicontrol('Style','text','Position',[20 95 295 100], ...
    'FontSize', 10 - 1*isOctave, 'HorizontalAlignment','left');


% Axes for plotting
%------------------------------------------------------
ax = axes('Units','pixels','Position',[320 100 550 450]);
axis equal;
grid on;
if isOctave
    xlabel(ax,'X','FontSize',14); ylabel(ax,'Y','FontSize',14);
    title(ax,'Four-Bar Linkage','FontSize',16);
else
    xlabel(ax,'X'); ylabel(ax,'Y');
    title(ax,'Four-Bar Linkage');
end
xlim([-1 1.4]*1.2);
ylim([-1.2 1.2]*1.2);
hold(ax,'on');

%------------------------------------------------------
% Data storage (use guidata)
%------------------------------------------------------
data.name          = 'Fourbar Linkage';
data.version       = 0.9;
data.geoEd         = geoEd;
data.modeStr       = 'Direct';  % active mode: 'Direct' or 'Inverse'
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
data.colors        = colors;
data.animateFlag   = false;
data.timerObj      = [];
data.animateBtn    = animate_btn;
data.th_offset     = get(thetaSlider,'Value');
data.alph_offset   = get(alphaSlider,'Value');
data.info_text     = info_text;
data.ax            = ax;
guidata(hFig,data);

% Store fixed axis limits in data
data.limits = [get(ax,'XLim'), get(ax,'YLim')];
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

% Initial draw
updatePlot([],[]);

% ---------------------------------------------------------------------
% Nested callback functions and helpers
% ---------------------------------------------------------------------

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



    % cbRad1 / cbRad2: called when user clicks 'Direct' or 'Inverse'.
    % Replaces modeChanged (event.NewValue not supported in Octave).
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
        % Pull GUI data
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
            opts.clearAxes  = true;
            opts.limits     = data.limits;
            opts.showLabels = true;
            if ~isempty(selSol)
                opts.solutions = selSol;
            end

            % 4) Which mode?
            modeStr = data.modeStr;

            if strcmp(modeStr,'Direct')
                %----------------------------------------------------------
                % DIRECT MODE
                %----------------------------------------------------------
                theta = get(data.thetaSl,'Value');

                % keep the text boxes in sync
                set(data.thetaTxt,'String',sprintf('%.1f',theta));

                % call the new unified plotter
                if exist('fourbar_plot','file') ~= 2
                    error('fourbar_plot.m is not on the path.');
                end
                [~, sol] = fourbar_plot(g, 'direct', deg2rad(theta), opts);

                % enable/disable the 2 checkboxes exactly as before
                for i = 1:2
                    if i <= numel(sol) && (~isfield(sol(i),'valid') || sol(i).valid)
                        set(data.sols_checkbox(i),'Enable','on');
                    else
                        set(data.sols_checkbox(i),'Enable','off');
                    end
                end

                % build the info text
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
                %----------------------------------------------------------
                % INVERSE MODE
                %----------------------------------------------------------
                alpha = get(data.alphaSl,'Value');

                % keep the edit boxes in sync
                set(data.alphaTxt,'String',sprintf('%.2f',alpha));

                % call the new unified plotter
                if exist('fourbar_plot','file') ~= 2
                    error('fourbar_plot.m is not on the path.');
                end
                [~, sol] = fourbar_plot(g, 'inverse', deg2rad(alpha), opts);

                % enable/disable checkboxes according to valid solutions
                for i = 1:2
                    if i <= numel(sol) && (~isfield(sol(i),'valid') || sol(i).valid)
                        set(data.sols_checkbox(i),'Enable','on');
                    else
                        set(data.sols_checkbox(i),'Enable','off');
                    end
                end

                % build info text
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

            %— Plot trajectory of P if requested —
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
                % Check which branch to display
                if any(selSol==1)
                    lw_traj = 1.0; if isOctave, lw_traj = 0.4; end
                    plot(ax, P_traj(1,:), P_traj(2,:), 'k:', 'LineWidth',lw_traj);
                end
                if any(selSol==2)
                    lw_traj = 1.0; if isOctave, lw_traj = 0.4; end
                    plot(ax, P_traj_alt(1,:), P_traj_alt(2,:), 'k--', 'LineWidth',lw_traj);
                end
            end

            % 5) enforce GUI limits and look
            axis(data.ax,'equal');
            xlim(data.ax, data.limits(1:2));
            ylim(data.ax, data.limits(3:4));
            drawnow();

        catch ME
            % Invalid/unreachable → show error
            cla(ax);
            text(0,0,'Unreachable','Parent',ax,'Color','r','FontSize',14,'HorizontalAlignment','center');
            set(info_text,'String','Unreachable configuration');
        end
    end


%-------------------------------------------------------------------------
    function toggleAnimation(~,~)
        % Start/stop animation.
        % MATLAB: uses timer object for non-blocking animation.
        % Octave:  timer is not implemented; uses a drawnow-based while-loop
        %          that processes GUI events via drawnow, allowing the Stop
        %          button to interrupt the loop through the animateFlag.
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
                % Octave: timer not available; run a drawnow loop.
                % guidata must be saved before entering the loop so
                % the Stop button callback can clear animateFlag.
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

%-------------------------------------------------------------------------
    function animateStep(~,~)
        % Advance the slider by one time-step and redraw.
        % Called by the MATLAB timer tick, or directly by the Octave loop.
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


end