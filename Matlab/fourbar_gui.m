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
hFig = figure('Name','Four-Bar Linkage GUI','NumberTitle','off', ...
    'MenuBar','none','ToolBar','figure','Position',[300 100 900 600]);

% Custom menu bar
hMenuFig = gcf;

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
    uimenu(mFile, parentProp,'Exit',            cbProp,@(~,~) cbExit(hMenuFig));

    % ---- View submenus ----
    uimenu(mView, parentProp,'Reset View',      cbProp,@(~,~) cbResetView(hMenuFig));

    % ---- Options submenus ----
    uimenu(mOptions, parentProp,'Preferences',  cbProp,@(~,~) cbPreferences(hMenuFig));
end


% ---- Callback implementations ----
    function cbOpen(hFig)
        % Open a MATLAB .mat file created by cbSave and update the GUI.
        [f,p] = uigetfile({'*.mat','MAT-file (*.mat)'}, 'Open Data File');
        if isequal(f,0), return; end
        full = fullfile(p,f);

        s = load(full, '-mat');
        if ~isfield(s,'data')
            error('Selected file does not contain a variable named ''data''.');
        end
        data = s.data;

        % ----- Store 'data' back into the GUI state (guidata/appdata) -----
        didStore = false;

        gd = guidata(hFig);
        if isstruct(gd) && isfield(gd,'data')
            gd.data = data;
            guidata(hFig, gd);
            didStore = true;
        elseif isstruct(gd)
            gd.data = data;
            guidata(hFig, gd);
            didStore = true;
        end
        if ~didStore
            setappdata(hFig,'data', data);
            didStore = true;
        end
        if ~didStore
            set(hFig,'UserData', data);
        end

        localApplyDataToGUI(hFig, data);

    end

    function localApplyDataToGUI(hFig, data)
        % Recursively apply fields of 'data' to UI components whose Tag matches fields.
        if ~isstruct(data)
            return;
        end
        fns = fieldnames(data);
        for k = 0:numel(fns)-1
            fn = fns{k+1};
            val = data.(fn);

            % Recurse into sub-structs
            if isstruct(val)
                localApplyDataToGUI(hFig, val);
                continue;
            end

            % Try to find an object with Tag = field name
            obj = findobj(hFig, '-depth', 10, 'Tag', fn); % supports classic figure/uifigure

            if isempty(obj)
                continue;
            end

            % Try to set Value/String/Checked/Items sensibly
            for h = transpose(obj(:))
                didSet = false;
                if isnumeric(val) || islogical(val)
                    set(h, 'Value', val);  % preferred for modern controls
                    didSet = true;
                end
                if ~didSet
                    if isstring(val) || ischar(val)
                        set(h, 'Value', char(val));  % uifigure edit fields, etc.
                        didSet = true;
                    end
                end

                if ~didSet
                    if isstring(val) || ischar(val) || isnumeric(val) || islogical(val)
                        if ~ischar(val) && ~isstring(val)
                            sval = mat2str(val);
                        else
                            sval = char(val);
                        end
                        set(h, 'String', sval);      % classic uicontrols
                        didSet = true;
                    end
                end

                if ~didSet && (islogical(val) || (isnumeric(val) && isscalar(val)))
                    set(h, 'Checked', ternary(val,'on','off')); %#ok<*UNRCH>
                    didSet = true;
                end

                if ~didSet && (isstring(val) || ischar(val))
                    items = get(h,'Items');
                    if iscell(items) && any(strcmp(items, char(val)))
                        set(h,'Value', char(val));   % dropdown / listbox items
                        didSet = true;
                    end
                end
            end
        end
    end

    function cbSave(hFig)
        % Save a MATLAB .mat file containing the 'data' structure.
        [f,p] = uiputfile({'*.mat','MAT-file (*.mat)'}, 'Save Data As', 'fourbar_session.mat');
        if isequal(f,0), return; end
        target = fullfile(p,f);

        % --- Try to obtain the 'data' structure from common stores ---
        data = [];
        gd = guidata(hFig);
        if isstruct(gd) && isfield(gd,'data')
            data = gd.data;            % case: guidata has a field 'data'
        elseif isstruct(gd)
            data = gd;                 % case: guidata itself is the data struct
        end

        % Saving geometry as values
        for ii = 1:numel(data.geoEd)
            g(ii) = str2double(get(data.geoEd(ii),'String'));
        end
        data.geo=g;

        warning('off', 'MATLAB:save:SavingFigure');

        % Write MAT file compatible with MATLAB
        save(target, 'data', '-mat');

        %localInfoAlert(hFig, sprintf('Saved MATLAB data to:\n%s', target), 'Save');
    end

    function cbExportPNG(hFig)
        % Export current figure to PNG (vector formats may vary across versions)
        [f,p] = uiputfile({'*.png','PNG Image (*.png)'}, 'Export As');
        if isequal(f,0), return; end
        target = fullfile(p,f);
        if exist('exportgraphics','file')
            exportgraphics(hFig, target); % export full figure

        else
            % Fallback for older MATLAB: use print
            print(hFig, '-dpng', '-r300', target);
        end

        %localInfoAlert(hFig, sprintf('Exported to: %s', target), 'Export');
    end

    function cbExportEPSPDF(hFig)
        % Export current figure to EPS and PDF
        [f,p] = uiputfile({'*.eps','EPS Vector (*.eps)'}, 'Export As');
        if isequal(f,0), return; end
        target = fullfile(p,f);
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
        drawnow;
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
    'Position',[0.02 0.67 0.33 0.10]);
modeBtn = uibuttongroup('Units','normalized', ...
    'Position',[0.02 0.68 0.33 0.06], ...
    'SelectionChangedFcn',@modeChanged);
rad1 = uicontrol(modeBtn,'Style','radiobutton','String','Direct', ...
    'Units','normalized','Position',[0.05 0.10 0.4 0.8]);
rad2 = uicontrol(modeBtn,'Style','radiobutton','String','Inverse', ...
    'Units','normalized','Position',[0.50 0.10 0.4 0.8]);
modeBtn.SelectedObject = rad1;  % default: Direct


%------------------------------------------------------
% Direct Mode Controls
%------------------------------------------------------
directPanel = uipanel('Title','Direct Mode Slider','FontSize',10, ...
    'Position',[0.02 0.59 0.33 0.08], ...
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
    'Position',[0.02 0.50 0.33 0.08], ...
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

% Checkbox: display solutions
solsPanel = uipanel('Title','Display solutions:','FontSize',10, ...
    'Position',[0.02 0.42 0.33 0.08], ...
    'Visible','on');
for i=1:2
    sols_checkbox(i) = uicontrol('Parent',solsPanel,'Units','normalized','Style','checkbox','Position',[0.2+0.2*i 0.3 0.7 0.6], ...
        'String',num2str(i),'Value',1,'Callback',@updatePlot);
end

% Define color schemes
colors = [1 0 0;0 0 1];

% Checkbox: show P trajectory (for full crank rotation)
traj_checkbox = uicontrol('Style','checkbox','Position',[20 230 270 20], ...
    'String','Show P trajectory','Value',0, ...
    'Callback',@updatePlot);

% Animate button
animate_btn = uicontrol('Style','pushbutton','String','Animate', ...
    'Position',[20 200 295 30], 'Callback',@toggleAnimation);

% Text area for φ/theta/α & P coords
info_text = uicontrol('Style','text','Position',[20 100 295 100], ...
    'FontSize',10,'HorizontalAlignment','left');


% Axes for plotting
%------------------------------------------------------
ax = axes('Units','pixels','Position',[320 100 550 450]);
axis equal
grid on
xlabel('X'); ylabel('Y');
title('Four-Bar Linkage');
xlim([-1 1.4]*1.2);
ylim([-1.2 1.2]*1.2);
hold(ax,'on');

%------------------------------------------------------
% Data storage (use guidata)
%------------------------------------------------------
data.name          = 'Fourbar Linkage';
data.version       = 0.9;
data.geoEd         = geoEd;
data.modeBtn       = modeBtn;
data.directPanel   = directPanel;
data.inversePanel  = inversePanel;
data.thetaSl       = thetaSlider;
data.thetaTxt      = thetaValTxt;
data.alphaSl       = alphaSlider;
data.alphaTxt      = alphaValTxt;
data.sols_checkbox = sols_checkbox;
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



    function modeChanged(~, event)
        data = guidata(hFig);

        % First, always grab the current geometry vector (g)
        g = zeros(1,7);
        for ii = 1:7
            g(ii) = str2double( get(data.geoEd(ii), 'String') );
        end
        % Conversion to radians from the input values in degrees
        g(6)=deg2rad(g(6));
        g(7)=deg2rad(g(7));

        % If user just clicked “Direct”:
        if event.NewValue == rad1   % Direct mode selected
            % Enable Direct controls, disable Inverse controls
            set(data.thetaSl,  'Enable','on');
            set(data.thetaTxt, 'Enable','on');
            set(data.alphaSl,  'Enable','off');
            set(data.alphaTxt, 'Enable','off');

            % 1. Read the CURRENT Inverse sliders (alpha) so we can convert them
            alpha = get(data.alphaSl, 'Value');
            alpha_current = alpha;

            % 2. Compute all inverse‐solutions at that P
            invSol = fourbar_inverse_kinematics(g, deg2rad(alpha_current));

            if ~isempty(invSol)
                % 3. Pick the first valid solution (or whichever branch you want)
                theta = invSol(1).theta;

                % 4. Push those thetas into the Direct sliders/text
                set(data.thetaSl,  'Value', rad2deg(theta));
                set(data.thetaTxt, 'String', sprintf('%.1f', rad2deg(theta)));
            end

            % 6. Redraw exactly that same pose
            updatePlot([], []);

        else  % “Inverse” was selected
            % Enable Inverse controls, disable Direct controls
            set(data.thetaSl,  'Enable','off');
            set(data.thetaTxt, 'Enable','off');
            set(data.alphaSl,  'Enable','on');
            set(data.alphaTxt, 'Enable','on');

            % 1. Read CURRENT Direct sliders (theta)
            theta = get(data.thetaSl, 'Value');
            currentTheta = theta;

            % 2. Compute direct‐kinematics to get (Px,Py)
            solDir = fourbar_direct_kinematics(g, deg2rad(currentTheta));

            if ~isempty(solDir)
                % 3. Take the same branch you were looking at (e.g. solDir(1))
                alpha = solDir(1).alpha;   % assume

                % 4. Push alpha into the Inverse sliders/text
                set(data.alphaSl,  'Value', rad2deg(alpha));
                set(data.alphaTxt,  'String', sprintf('%.2f', rad2deg(alpha)));
            end

            % 6. Redraw exactly that same pose
            updatePlot([], []);
        end
        % Store back (if you made any changes to data)
        guidata(hFig, data);
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
            modeStr = get(data.modeBtn.SelectedObject,'String');

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
                        info_str = sprintf('%sSol %d: P = [%.2f; %.2f]\t α=%.2f\t Φ=%.2f\n', ...
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
                        info_str = sprintf('%sSol %d: P = [%.2f; %.2f]\t θ=%.2f\t Φ=%.2f\n', ...
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
                    plot(ax, P_traj(1,:), P_traj(2,:), 'k:', 'LineWidth',1);
                end
                if any(selSol==2)
                    plot(ax, P_traj_alt(1,:), P_traj_alt(2,:), 'k--', 'LineWidth',1);
                end
            end

            % 5) enforce GUI limits and look
            axis(data.ax,'equal');
            xlim(data.ax, data.limits(1:2));
            ylim(data.ax, data.limits(3:4));
            drawnow;

        catch ME
            % Invalid/unreachable → show error
            cla(ax);
            text(0,0,'Unreachable','Parent',ax,'Color','r','FontSize',14,'HorizontalAlignment','center');
            set(info_text,'String','Unreachable configuration');
        end
    end


%-------------------------------------------------------------------------
    function toggleAnimation(~,~)
        % Start/stop the animation timer
        data = guidata(hFig);
        if data.animateFlag
            % Stop and delete existing timer
            if ~isempty(data.timerObj) && isvalid(data.timerObj)
                stop(data.timerObj);
                delete(data.timerObj);
            end
            data.timerObj = [];
            data.animateFlag = false;
            set(data.animateBtn,'String','Animate');
            guidata(hFig,data);
        else
            % Record current theta values as offsets
            data.th_offset = get(data.thetaSl,'Value');
            data.al_offset = get(data.alphaSl,'Value');
            data.time_offset=now*24*3600;
            % Ensure no old timer remains
            if ~isempty(data.timerObj) && isvalid(data.timerObj)
                stop(data.timerObj); delete(data.timerObj);
            end
            % Create timer for animation steps (e.g., 30 fps)
            t = timer('ExecutionMode','fixedRate', ...
                'Period',0.033, ...   % ~30 Hz
                'TimerFcn',@animateStep);
            data.timerObj = t;
            data.animateFlag = true;
            set(data.animateBtn,'String','Stop');
            guidata(hFig,data);
            start(t);
        end
    end

%-------------------------------------------------------------------------
    function animateStep(~,~)
        % Called at each timer tick: update sliders for motion, then redraw
        data = guidata(hFig);
        mode = data.modeBtn.SelectedObject.String;
        tnow = now*24*3600;  % seconds since some reference
        switch mode
            case 'Direct'
                % Sinusoidal motion parameters
                %Am = 180; f1 = 0.02; % amplitude/degrees, freq/Hz
                %th = data.th_offset + Am * sin(2*pi*f1*tnow);
                speed=10;
                th = wrapTo180(data.th_offset + speed*(tnow-data.time_offset));
                % Update sliders (silently)
                set(data.thetaSl,'Value',th);
            case 'Inverse'
                speed=10;
                al = wrapTo180(data.al_offset + speed*(tnow-data.time_offset));
                % Update sliders (silently)
                set(data.alphaSl,'Value',al);
        end
        updatePlot();
    end


end