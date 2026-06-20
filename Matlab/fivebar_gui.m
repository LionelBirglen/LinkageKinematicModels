function fivebar_gui
% FIVEBAR_GUI  Interactive GUI for five‐bar linkage using analytic kinematics
%
%   This GUI lets you:
%     • Enter geometry (a,b,c,d,e,h,eta)
%     • Choose Direct or Inverse mode
%     • In Direct mode: slide θ1 and θ2 (in degrees) and see the five‐bar update
%     • In Inverse mode: type (Px,Py), click “Solve,” and see up to 4 solutions
%
%   Requirements: fivebar_direct_kinematics.m and fivebar_inverse_kinematics.m
%   must be on the MATLAB path.
%
%   To run: save this file as fivebar_gui.m and type >> fivebar_gui

%Afficher trajectoire lors des animations
%afficher unreachable


% Create main figure
hFig = figure('Name','Five‐Bar Linkage GUI','NumberTitle','off', ...
    'MenuBar','none', 'Position',[200 100 800 600]);

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
        [f,p] = uiputfile({'*.mat','MAT-file (*.mat)'}, 'Save Data As', 'fivebar_session.mat');
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
        for ii = 1:8
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
            try
                if ~isempty(get(ax(k),'ZLim')) %#ok<*GETFID>
                    % Attempt to set a sensible 3D view if Z exists
                    view(ax(k), 3);
                end
            catch
                % 2D axes
                try, view(ax(k), 2); catch, end
            end
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

% a,b,c,d,e,h,eta labels and edit boxes
txtX = 0.025;  wX = 0.07;  hX = 0.03;  dy = 0.04;  sy = 0.91;
names = {'a (O→A)','b (A→B)','c (B→C)','d (C→D)','e (O→D)', 'X→e (deg)', 'h (A→P)','b→h (deg)'};
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
for k = 5:8
    uicontrol('Style','text', ...
        'Units','normalized', ...
        'Position',[0.15+txtX sy-(k-5)*dy wX hX], ...
        'String',names{k}, ...
        'HorizontalAlignment','right');
    geoEd(k) = uicontrol('Style','edit', ...
        'Units','normalized', ...
        'Position',[0.15+txtX+wX+0.01 sy-(k-5)*dy wX hX], ...
        'String','0','Callback',@updatePlot);
end

% Preload some default geometry
set(geoEd(1),'String','0.6');  % a
set(geoEd(2),'String','0.7');  % b
set(geoEd(3),'String','0.9');  % c
set(geoEd(4),'String','0.6');  % d
set(geoEd(5),'String','1.0');  % e
set(geoEd(6),'String','0.0');  % alpha
set(geoEd(7),'String','0.5');  % h
set(geoEd(8),'String','45');   % eta

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
directPanel = uipanel('Title','Direct: Slide θ1, θ2','FontSize',10, ...
    'Position',[0.02 0.52 0.33 0.15], ...
    'Visible','on');

uicontrol('Parent',directPanel,'Style','text','Units','normalized', ...
    'Position',[0.04 0.58 0.3 0.25],'String','θ1','HorizontalAlignment','left');
theta1Slider = uicontrol('Parent',directPanel,'Style','slider', ...
    'Units','normalized','Position',[0.10 0.55 0.75 0.3], ...
    'Min',-180,'Max',180,'Value',120,'SliderStep', [0.01/7.2, 0.1/7.2],...
    'Callback',@updatePlot);
theta1ValTxt = uicontrol('Parent',directPanel,'Style','edit', ...
    'Units','normalized','Position',[0.88 0.58 0.3 0.25], ...
    'BackgroundColor',[1 1 1], ...
    'String','45','HorizontalAlignment','left', ...
    'Callback',@cbThetaEdit);

uicontrol('Parent',directPanel,'Style','text','Units','normalized', ...
    'Position',[0.04 0.1 0.3 0.25],'String','θ2','HorizontalAlignment','left');
theta2Slider = uicontrol('Parent',directPanel,'Style','slider', ...
    'Units','normalized','Position',[0.10 0.10 0.75 0.30], ...
    'Min',-180,'Max',180,'Value',75,'SliderStep', [0.01/7.2, 0.1/7.2], ...
    'Callback',@updatePlot);
theta2ValTxt = uicontrol('Parent',directPanel,'Style','edit', ...
    'Units','normalized','Position',[0.88 0.10 0.3 0.25], ...
    'BackgroundColor',[1 1 1], ...
    'String','-30','HorizontalAlignment','left', ...
    'Callback',@cbThetaEdit);


%------------------------------------------------------
% Inverse Mode Controls
%------------------------------------------------------
inversePanel = uipanel('Title','Inverse: Slide Px, Py','FontSize',10, ...
    'Position',[0.02 0.38 0.33 0.15], ...
    'Visible','on');
uicontrol('Parent',inversePanel,'Style','text','Units','normalized', ...
    'Position',[0.04 0.58 0.3 0.25],'String','Px:','HorizontalAlignment','left');
PxSlider = uicontrol('Parent',inversePanel,'Style','slider','Units','normalized', ...
    'Position',[0.10 0.55 0.75 0.30],'Min',-1.5,'Max',1.5,'Value',0.2, ...
    'Callback',@updatePlot);
PxValTxt = uicontrol('Parent',inversePanel,'Style','edit', ...
    'Units','normalized','Position',[0.88 0.58 0.3 0.25], ...
    'BackgroundColor',[1 1 1], ...
    'String',num2str(get(PxSlider,'Value')),'HorizontalAlignment','left', ...
    'Callback',@cbPosEdit);
uicontrol('Parent',inversePanel,'Style','text','Units','normalized', ...
    'Position',[0.04 0.10 0.3 0.25],'String','Py:','HorizontalAlignment','left');
PySlider = uicontrol('Parent',inversePanel,'Style','slider','Units','normalized', ...
    'Position',[0.10 0.10 0.75 0.30],'Min',-1.5,'Max',1.5,'Value',1, ...
    'Callback',@updatePlot);
PyValTxt = uicontrol('Parent',inversePanel,'Style','edit', ...
    'Units','normalized','Position',[0.88 0.10 0.3 0.25], ...
    'BackgroundColor',[1 1 1], ...
    'String',num2str(get(PySlider,'Value')),'HorizontalAlignment','left', ...
    'Callback',@cbPosEdit);


% Checkbox: display solutions
solsPanel = uipanel('Title','Display solutions:','FontSize',10, ...
    'Position',[0.02 0.29 0.33 0.08], ...
    'Visible','on');
for i=1:4
    sols_checkbox(i) = uicontrol('Parent',solsPanel,'Units','normalized','Style','checkbox','Position',[0.2*i 0.3 0.7 0.6], ...
        'String',num2str(i),'Value',1,'Callback',@updatePlot);
end

% Define color schemes
colors = lines(4);

% Animate button
animate_btn = uicontrol('Style','pushbutton','String','Animate', ...
    'Position',[16 140 265 30], 'Callback',@toggleAnimation);

% Text area for theta & P coords
info_text = uicontrol('Style','text','Position',[10 10 300 120], ...
    'FontSize',10,'HorizontalAlignment','left');

% Axes for plotting
%------------------------------------------------------
ax = axes('Units','normalized','Position',[0.42 0.10 0.56 0.80]);
axis equal
grid on
xlabel('X'); ylabel('Y');
title('Five‐Bar Linkage');
xlim([-1 1.4]*1.2);
ylim([-1.2 1.2]*1.2);
hold(ax,'on');

%------------------------------------------------------
% Data storage (use guidata)
%------------------------------------------------------
data.name        = 'Fivebar Linkage';
data.version     = 0.9;
data.geoEd       = geoEd;
data.modeBtn     = modeBtn;
data.directPanel = directPanel;
data.inversePanel= inversePanel;
data.theta1Sl    = theta1Slider;
data.theta2Sl    = theta2Slider;
data.theta1Txt   = theta1ValTxt;
data.theta2Txt   = theta2ValTxt;
data.PxSlider    = PxSlider;
data.PySlider    = PySlider;
data.PxValTxt    = PxValTxt;
data.PyValTxt    = PyValTxt;
data.sols_checkbox= sols_checkbox;
data.colors       = colors;
data.animateFlag  = false;
data.timerObj     = [];
data.animateBtn   = animate_btn;
data.th1_offset   = get(theta1Slider,'Value');
data.th2_offset   = get(theta2Slider,'Value');
data.Px_offset   = get(PxSlider,'Value');
data.Py_offset   = get(PySlider,'Value');
data.info_text    = info_text;
data.ax           = ax;
guidata(hFig,data);

% Store fixed axis limits in data
data.limits = [get(ax,'XLim'), get(ax,'YLim')]*1.1;
guidata(hFig,data);

% Initial draw (Direct mode)
updatePlot([],[]);

%============ Nested callback functions ============%

    function modeChanged(~, event)
        data = guidata(hFig);

        % First, always grab the current geometry vector (g)
        g = zeros(1,8);
        for ii = 1:8
            g(ii) = str2double( get(data.geoEd(ii), 'String') );
        end

        % If user just clicked “Direct”:
        if event.NewValue == rad1   % Direct mode selected
            % 1. Read the CURRENT Inverse sliders (Px, Py) so we can convert them
            Px = get(data.PxSlider, 'Value');
            Py = get(data.PySlider, 'Value');
            P_current = [Px; Py];

            % 2. Compute all inverse‐solutions at that P
            invSol = fivebar_inverse_kinematics(g, P_current);

            if ~isempty(invSol)
                % 3. Pick the first valid solution (or whichever branch you want)
                theta1 = invSol(1).theta(1);
                theta2 = invSol(1).theta(2);

                % 4. Push those thetas into the Direct sliders/text
                set(data.theta1Sl,  'Value', theta1);
                set(data.theta2Sl,  'Value', theta2);
                set(data.theta1Txt, 'String', sprintf('%.1f', theta1));
                set(data.theta2Txt, 'String', sprintf('%.1f', theta2));
            end

            % 6. Redraw exactly that same pose
            updatePlot([], []);

        else  % “Inverse” was selected
            % 1. Read CURRENT Direct sliders (theta1, theta2)
            theta1 = get(data.theta1Sl, 'Value');
            theta2 = get(data.theta2Sl, 'Value');
            currentThetas = [theta1; theta2];

            % 2. Compute direct‐kinematics to get (Px,Py)
            solDir = fivebar_direct_kinematics(g, currentThetas);

            if ~isempty(solDir)
                % 3. Take the same branch you were looking at (e.g. solDir(1))
                P = solDir(1).P;   % assume Positions.P is [Px; Py]

                % 4. Push P into the Inverse sliders/text
                set(data.PxSlider,  'Value', P(1));
                set(data.PySlider,  'Value', P(2));
                set(data.PxValTxt,  'String', sprintf('%.2f', P(1)));
                set(data.PyValTxt,  'String', sprintf('%.2f', P(2)));
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
        g = zeros(1,8);
        for ii = 1:8
            g(ii) = str2double(get(data.geoEd(ii),'String'));
        end

        % 2) Find which solutions the user wants to see (1..4)
        selSol = [];
        if isfield(data,'sols_checkbox')
            for k = 1:numel(data.sols_checkbox)
                if get(data.sols_checkbox(k),'Value')
                    selSol(end+1) = k; 
                end
            end
        end

        % 3) Common plotting options for fivebar_plot
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
            theta1 = get(data.theta1Sl,'Value');
            theta2 = get(data.theta2Sl,'Value');

            % keep the text boxes in sync
            set(data.theta1Txt,'String',sprintf('%.1f',theta1));
            set(data.theta2Txt,'String',sprintf('%.1f',theta2));

            % call the new unified plotter
            if exist('fivebar_plot','file') ~= 2
                error('fivebar_plot.m is not on the path.');
            end
            [~, sol] = fivebar_plot(g, 'direct', [theta1 theta2], opts);

            % enable/disable the 4 checkboxes exactly as before
            for i = 1:4
                if i <= numel(sol) && (~isfield(sol(i),'valid') || sol(i).valid)
                    set(data.sols_checkbox(i),'Enable','on');
                else
                    set(data.sols_checkbox(i),'Enable','off');
                end
            end

            % build the info text (like original GUI)
            info_str = sprintf('Direct mode:\nθ1 = %.1f°\nθ2 = %.1f°\n',theta1,theta2);
            for i = 1:numel(sol)
                if ~isfield(sol(i),'valid') || sol(i).valid
                    Pp = sol(i).P;
                    info_str = sprintf('%sSol %d: P = (%.3f, %.3f)\n', ...
                        info_str, i, Pp(1), Pp(2));
                else
                    info_str = sprintf('%sSol %d: invalid\n', info_str, i);
                end
            end
            set(data.info_text,'String',info_str);

        else
            %----------------------------------------------------------
            % INVERSE MODE
            %----------------------------------------------------------
            Px = get(data.PxSlider,'Value');
            Py = get(data.PySlider,'Value');

            % keep the edit boxes in sync
            set(data.PxValTxt,'String',sprintf('%.2f',Px));
            set(data.PyValTxt,'String',sprintf('%.2f',Py));

            % call the new unified plotter
            if exist('fivebar_plot','file') ~= 2
                error('fivebar_plot.m is not on the path.');
            end
            [~, sol] = fivebar_plot(g, 'inverse', [Px Py], opts);

            % enable/disable checkboxes according to valid solutions
            for i = 1:4
                if i <= numel(sol) && (~isfield(sol(i),'valid') || sol(i).valid)
                    set(data.sols_checkbox(i),'Enable','on');
                else
                    set(data.sols_checkbox(i),'Enable','off');
                end
            end

            % build info text
            info_str = sprintf('Inverse mode:\nP = (%.3f, %.3f)\n',Px,Py);
            for i = 1:numel(sol)
                if ~isfield(sol(i),'valid') || sol(i).valid
                    % your IK routines in the GUI store joint angles in .theta
                    if isfield(sol(i),'theta') && numel(sol(i).theta) >= 2
                        th1 = sol(i).theta(1);
                        th2 = sol(i).theta(2);
                        info_str = sprintf('%sSol %d: θ1 = %.1f°, θ2 = %.1f°\n', ...
                            info_str, i, th1, th2);
                    else
                        info_str = sprintf('%sSol %d: valid\n', info_str, i);
                    end
                else
                    info_str = sprintf('%sSol %d: invalid\n', info_str, i);
                end
            end
            set(data.info_text,'String',info_str);
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
            data.th1_offset = get(data.theta1Sl,'Value');
            data.th2_offset = get(data.theta2Sl,'Value');
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
                A1 = 60; f1 = 0.1; % amplitude/degrees, freq/Hz
                A2 = 45; f2 = 0.15;
                th1 = data.th1_offset + A1 * sin(2*pi*f1*tnow);
                th2 = data.th2_offset + A2 * sin(2*pi*f2*tnow);
                % Update sliders (silently)
                set(data.theta1Sl,'Value',th1);
                set(data.theta2Sl,'Value',th2);
            case 'Inverse'
                % Circular trajectory for P
                R = 0.25; f = 0.05; % radius, freq
                px = data.Px_offset + R * cos(2*pi*f*tnow);
                py = data.Py_offset + R * sin(2*pi*f*tnow);
                set(data.PxSlider,'Value',px);
                set(data.PySlider,'Value',py);
        end
        updatePlot();
    end

%-------------------------------------------------------------------------
% Nested helper: draws fivebar given joints, link radii, P, φ
% (Assumes joint structure has fields O,A,B,C,D)
    function plotFivebar(joint,r,P,phi,varargin)
        axh = data.ax;
        % Extract points
        O = joint.O; A = joint.A; B = joint.B; C = joint.C; D = joint.D;
        % Plot ground pivots
        plot(axh,O(1),O(2),'ko','MarkerSize',6,'MarkerFaceColor','k'); hold(axh,'on');
        plot(axh,D(1),D(2),'ko','MarkerSize',6,'MarkerFaceColor','k');
        % Plot links: OA, AB, BC, CD, DO
        plot(axh,[O(1) A(1) B(1) C(1) D(1) O(1)], [O(2) A(2) B(2) C(2) D(2) O(2)],'-b','LineWidth',2);
        % Plot P
        plot(axh,P(1),P(2),'ro','MarkerSize',6,'MarkerFaceColor','r');
        axis(axh,'equal');
    end

    function PlotLabelJoint(data,joint)
        axh = data.ax;
        % Extract points
        O = joint.Positions.O; A = joint.Positions.A; B = joint.Positions.B; C = joint.Positions.C; D = joint.Positions.D;P=joint.P;
        dx = 0.02; dy = 0.02;

        text(axh, O(1)+dx, O(2)+dy, 'O', 'FontSize', 10, 'Color', [0 0 0], 'HorizontalAlignment','left','VerticalAlignment','bottom', 'Interpreter','none');
        text(axh, A(1)+dx, A(2)+dy, 'A', 'FontSize', 10, 'Color', [0 0 0], 'HorizontalAlignment','left','VerticalAlignment','bottom', 'Interpreter','none');
        text(axh, B(1)+dx, B(2)+dy, 'B', 'FontSize', 10, 'Color', [0 0 0], 'HorizontalAlignment','left','VerticalAlignment','bottom', 'Interpreter','none');
        text(axh, C(1)+dx, C(2)+dy, 'C', 'FontSize', 10, 'Color', [0 0 0], 'HorizontalAlignment','left','VerticalAlignment','bottom', 'Interpreter','none');
        text(axh, D(1)+dx, D(2)+dy, 'D', 'FontSize', 10, 'Color', [0 0 0], 'HorizontalAlignment','left','VerticalAlignment','bottom', 'Interpreter','none');
        text(axh, P(1)+dx, P(2)+dy, 'P', 'FontSize', 10, 'Color', [0 0 0], 'HorizontalAlignment','left','VerticalAlignment','bottom', 'Interpreter','none');
    end



    % -- New: edit callbacks for θ1/θ2 and Px/Py -----------------------------
    function cbThetaEdit(hObj,~)
        data = guidata(hFig);
        if hObj == data.theta1Txt
            sl = data.theta1Sl;
        else
            sl = data.theta2Sl;
        end
        val = str2double(get(hObj,'String'));
        if isnan(val)
            val = get(sl,'Value');
        end
        % clamp and update
        val = max(min(val, get(sl,'Max')), get(sl,'Min'));
        set(sl,'Value',val);
        set(hObj,'String',sprintf('%.1f',val));
        updatePlot(hObj,[]);
    end

    function cbPosEdit(hObj,~)
        data = guidata(hFig);
        if hObj == data.PxValTxt
            sl = data.PxSlider;
            fmt = '%.3f';
        else
            sl = data.PySlider;
            fmt = '%.3f';
        end
        val = str2double(get(hObj,'String'));
        if isnan(val)
            val = get(sl,'Value');
        end
        % clamp and update
        val = max(min(val, get(sl,'Max')), get(sl,'Min'));
        set(sl,'Value',val);
        set(hObj,'String',sprintf(fmt,val));
        updatePlot(hObj,[]);
    end

end