function fivebar_gui()
% FIVEBAR_GUI - Interactive GUI for a Planar Five-Bar Linkage
%
% Compatible with both MATLAB and Octave.
% Supports direct/inverse kinematics, display-solutions checkboxes,
% animation, session save/load, and PNG export.
%
% Requires: fivebar_direct_kinematics.m, fivebar_inverse_kinematics.m,
%           fivebar_plot.m
%
% BY:
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Contact: lionel.birglen@polymtl.ca
% Code provided under GNU Affero General Public License v3.0

% ----------------------------------------------------------------
% 0) Environment detection
% ----------------------------------------------------------------
isOctave = exist('OCTAVE_VERSION','builtin') ~= 0;

% ----------------------------------------------------------------
% 1) Figure
% ----------------------------------------------------------------
if isOctave
    hFig = figure('Name','Five-Bar Linkage','NumberTitle','off', ...
        'MenuBar','figure','ToolBar','figure','Position',[100 80 900 600]);
    set(hFig,'Color',[1 1 1]);
    set(0,'DefaultUicontrolBackgroundColor',[1 1 1]);
else
    hFig = figure('Name','Five-Bar Linkage','NumberTitle','off', ...
        'MenuBar','none','ToolBar','figure','Position',[100 80 900 600]);
end

% ----------------------------------------------------------------
% 2) Menu bar
% ----------------------------------------------------------------
hMenuFig = gcf;
if isOctave, delete(findall(hMenuFig,'Type','uimenu')); end
isUIFigure = isa(hMenuFig,'matlab.ui.Figure');
if isUIFigure; parentProp='Text'; cbProp='MenuSelectedFcn';
else;          parentProp='Label'; cbProp='Callback'; end

existingMenus = findall(hMenuFig,'Type','uimenu','-depth',1);
if isempty(existingMenus)
    mFile    = uimenu(hMenuFig, parentProp,'File');
    mView    = uimenu(hMenuFig, parentProp,'View');
    mOptions = uimenu(hMenuFig, parentProp,'Options');
    uimenu(mFile, parentProp,'Open',           cbProp,@(~,~) cbOpen(hMenuFig));
    uimenu(mFile, parentProp,'Save',           cbProp,@(~,~) cbSave(hMenuFig));
    uimenu(mFile, parentProp,'Export PNG',     cbProp,@(~,~) cbExportPNG(hMenuFig));
    uimenu(mFile, parentProp,'Export EPS+PDF', cbProp,@(~,~) cbExportEPSPDF(hMenuFig));
    uimenu(mFile, parentProp,'Print',          cbProp,@(~,~) cbPrint(hMenuFig));
    if isOctave
        set(findall(hMenuFig,'Label','Export EPS+PDF'),'Enable','off');
        set(findall(hMenuFig,'Label','Print'),          'Enable','off');
    end
    uimenu(mFile, parentProp,'Exit',           cbProp,@(~,~) cbExit(hMenuFig));
    uimenu(mView,    parentProp,'Reset View',  cbProp,@(~,~) cbResetView(hMenuFig));
    uimenu(mOptions, parentProp,'Preferences', cbProp,@(~,~) cbPreferences(hMenuFig));
end

% ----------------------------------------------------------------
% 3) Defaults
% ----------------------------------------------------------------
def_a     = 0.6;
def_b     = 0.7;
def_c     = 0.9;
def_d     = 0.6;
def_e     = 1.0;
def_alpha = 0.0;
def_h     = 0.5;
def_eta   = 45.0;
def_th1   = 120.0;
def_th2   = 75.0;
def_Px    = 0.2;
def_Py    = 1.0;

% ----------------------------------------------------------------
% 4) Geometry panel  (two columns: a,b,c,d | e,alpha,h,eta)
% ----------------------------------------------------------------
geoPanel = uipanel('Title','Geometry','FontSize',10, ...
    'Units','normalized','Position',[0.02 0.77 0.32 0.21]);
geoNames = {'a (O→A)','b (A→B)','c (B→C)','d (C→D)', ...
             'e (O→D)','α (deg)','h (A→P)','η (deg)'};
geoDefs  = {def_a, def_b, def_c, def_d, def_e, def_alpha, def_h, def_eta};
geoEd    = zeros(1,8);
txtX     = [0.03 0.03 0.03 0.03   0.52 0.52 0.52 0.52];
edX      = [0.23 0.23 0.23 0.23   0.72 0.72 0.72 0.72];
rows     = [0.72 0.50 0.28 0.06   0.72 0.50 0.28 0.06];
wLbl = 0.19;  wEd = 0.22;  hRow = 0.18;
for k = 1:8
    uicontrol('Parent',geoPanel,'Style','text','Units','normalized', ...
        'Position',[txtX(k) rows(k) wLbl hRow], ...
        'String',geoNames{k},'HorizontalAlignment','right');
    geoEd(k) = uicontrol('Parent',geoPanel,'Style','edit','Units','normalized', ...
        'Position',[edX(k) rows(k) wEd hRow], ...
        'String',num2str(geoDefs{k}),'Callback',@updatePlot);
end
if isOctave
    set(geoPanel,'BackgroundColor',[1 1 1]);
    for k = 1:8, set(geoEd(k),'FontSize',8); end
end

% ----------------------------------------------------------------
% 5) Mode panel
% ----------------------------------------------------------------
modePanel = uipanel('Title','Mode','FontSize',10,'Units','normalized', ...
    'Position',[0.02 0.68 0.32 0.08]);
rad1 = uicontrol('Style','radiobutton','String','Direct', ...
    'Units','normalized','Position',[0.1 0.69 0.1 0.04], ...
    'Value',1,'Callback',@cbRad1);
rad2 = uicontrol('Style','radiobutton','String','Inverse', ...
    'Units','normalized','Position',[0.19 0.69 0.1 0.04], ...
    'Value',0,'Callback',@cbRad2);
if isOctave, set(modePanel,'BackgroundColor',[1 1 1]); end

% ----------------------------------------------------------------
% 6) Direct mode sliders (theta1, theta2)
% ----------------------------------------------------------------
directPanel = uipanel('Title','Direct Mode Sliders','FontSize',10, ...
    'Units','normalized','Position',[0.02 0.53 0.32 0.14]);
dirPanelBot = [0.55 0.10];
if isOctave
    dirLabels = {'th1','th2'};
else
    dirLabels = {[char(952),'1'],[char(952),'2']};
end
dirDefs     = [def_th1, def_th2];
dirSl  = zeros(1,2);
dirTxt = zeros(1,2);
for k = 1:2
    uicontrol('Parent',directPanel,'Style','text','Units','normalized', ...
        'Position',[0.04 dirPanelBot(k) 0.10 0.30], ...
        'String',dirLabels{k},'HorizontalAlignment','left');
    dirSl(k) = uicontrol('Parent',directPanel,'Style','slider','Units','normalized', ...
        'Position',[0.15 dirPanelBot(k) 0.60 0.30], ...
        'Min',-180,'Max',180,'Value',dirDefs(k), ...
        'SliderStep',[1/360 10/360],'Callback',@updatePlot);
    dirTxt(k) = uicontrol('Parent',directPanel,'Style','edit','Units','normalized', ...
        'Position',[0.77 dirPanelBot(k) 0.20 0.30], ...
        'BackgroundColor',[1 1 1], ...
        'String',sprintf('%.1f',dirDefs(k)),'Callback',@(~,~) cbDirEdit(k));
end
if isOctave
    set(directPanel,'BackgroundColor',[1 1 1]);
    for k=1:2
        set(dirTxt(k),'FontSize',9);
        set(dirTxt(k),'Position',[0.76 dirPanelBot(k) 0.22 0.34]);
    end
end

% ----------------------------------------------------------------
% 7) Inverse mode sliders (Px, Py)
% ----------------------------------------------------------------
inversePanel = uipanel('Title','Inverse Mode Sliders','FontSize',10, ...
    'Units','normalized','Position',[0.02 0.38 0.32 0.14]);
invPanelBot  = [0.55 0.10];
invLabels    = {'Px','Py'};
invDefs      = [def_Px, def_Py];
invMins      = [-1.5, -1.5];
invMaxs      = [ 1.5,  1.5];
invSl  = zeros(1,2);
invTxt = zeros(1,2);
for k = 1:2
    uicontrol('Parent',inversePanel,'Style','text','Units','normalized', ...
        'Position',[0.04 invPanelBot(k) 0.10 0.30], ...
        'String',invLabels{k},'HorizontalAlignment','left');
    invSl(k) = uicontrol('Parent',inversePanel,'Style','slider','Units','normalized', ...
        'Position',[0.15 invPanelBot(k) 0.60 0.30], ...
        'Min',invMins(k),'Max',invMaxs(k),'Value',invDefs(k), ...
        'SliderStep',[0.01/3 0.1/3],'Callback',@updatePlot);
    invTxt(k) = uicontrol('Parent',inversePanel,'Style','edit','Units','normalized', ...
        'Position',[0.77 invPanelBot(k) 0.20 0.30], ...
        'BackgroundColor',[1 1 1], ...
        'String',sprintf('%.2f',invDefs(k)),'Callback',@(~,~) cbInvEdit(k));
end
% Start in Direct: disable inverse
for k=1:2
    set(invSl(k),'Enable','off');
    set(invTxt(k),'Enable','off');
end
if isOctave
    set(inversePanel,'BackgroundColor',[1 1 1]);
    for k=1:2
        set(invTxt(k),'FontSize',9);
        set(invTxt(k),'Position',[0.76 invPanelBot(k) 0.22 0.34]);
    end
end

% ----------------------------------------------------------------
% 8) Display solutions panel (4 checkboxes)
% ----------------------------------------------------------------
solsPanel = uipanel('Title','Display solutions:','FontSize',10, ...
    'Units','normalized','Position',[0.02 0.29 0.32 0.08]);
sols_checkbox = zeros(1,4);
for i = 1:4
    sols_checkbox(i) = uicontrol('Parent',solsPanel,'Units','normalized', ...
        'Style','checkbox','Position',[0.1+(i-1)*0.22 0.20 0.20 0.65], ...
        'String',num2str(i),'Value',1,'Callback',@updatePlot);
end
if isOctave, set(solsPanel,'BackgroundColor',[1 1 1]); end

% ----------------------------------------------------------------
% 9) Animate button and info text
% ----------------------------------------------------------------
animate_btn = uicontrol('Style','pushbutton','String','Animate', ...
    'Position',[18 145 290 26],'Callback',@toggleAnimation);

info_text = uicontrol('Style','text','Position',[20 55 290 90], ...
    'FontSize',10-1*isOctave,'HorizontalAlignment','left');

% ----------------------------------------------------------------
% 10) Axes
% ----------------------------------------------------------------
ax = axes('Units','pixels','Position',[310 50 560 500]);
axis equal; grid on;
if isOctave
    xlabel(ax,'X','FontSize',14); ylabel(ax,'Y','FontSize',14);
    title(ax,'Five-Bar Linkage','FontSize',16);
else
    xlabel(ax,'X'); ylabel(ax,'Y');
    title(ax,'Five-Bar Linkage');
end
hold(ax,'on');
lims = computeLimits(def_a,def_b,def_c,def_d,def_e,def_alpha,def_h);
xlim(ax,lims(1:2)); ylim(ax,lims(3:4));

% ----------------------------------------------------------------
% 11) Store state
% ----------------------------------------------------------------
data.geoEd        = geoEd;
data.modeStr      = 'Direct';
data.rad1         = rad1;
data.rad2         = rad2;
data.dirSl        = dirSl;
data.dirTxt       = dirTxt;
data.invSl        = invSl;
data.invTxt       = invTxt;
data.sols_checkbox= sols_checkbox;
data.animateFlag  = false;
data.timerObj     = [];
data.animateBtn   = animate_btn;
data.th1_offset   = def_th1;
data.th2_offset   = def_th2;
data.Px_offset    = def_Px;
data.Py_offset    = def_Py;
data.info_text    = info_text;
data.ax           = ax;
data.limits       = lims;
guidata(hFig, data);

updatePlot([],[]);

% ================================================================
%  Callbacks
% ================================================================

    function cbRad1(~,~)
        set(rad1,'Value',1); set(rad2,'Value',0);
        % Read geo first (readGeo overwrites data via guidata internally)
        g = readGeo();
        % Now re-read and set modeStr AFTER readGeo
        data = guidata(hFig);
        data.modeStr = 'Direct';
        % Convert current Px,Py -> theta1,theta2 via IK
        Px   = get(data.invSl(1),'Value');
        Py   = get(data.invSl(2),'Value');
        invS = fivebar_inverse_kinematics(g, [Px;Py]);
        if ~isempty(invS)
            th1 = invS(1).theta(1);
            th2 = invS(1).theta(2);
            th1 = max(-180,min(180,th1));
            th2 = max(-180,min(180,th2));
            set(data.dirSl(1),'Value',th1);
            set(data.dirSl(2),'Value',th2);
            set(data.dirTxt(1),'String',sprintf('%.1f',th1));
            set(data.dirTxt(2),'String',sprintf('%.1f',th2));
        end
        for k=1:2
            set(data.dirSl(k),'Enable','on');
            set(data.dirTxt(k),'Enable','on');
            set(data.invSl(k),'Enable','off');
            set(data.invTxt(k),'Enable','off');
        end
        guidata(hFig,data);
        updatePlot([],[]);
    end

    function cbRad2(~,~)
        set(rad1,'Value',0); set(rad2,'Value',1);
        % Read geo first (readGeo overwrites data via guidata internally)
        g = readGeo();
        % Now re-read and set modeStr AFTER readGeo
        data = guidata(hFig);
        data.modeStr = 'Inverse';
        % Convert current theta1,theta2 -> Px,Py via DK
        th1 = get(data.dirSl(1),'Value');
        th2 = get(data.dirSl(2),'Value');
        dirS = fivebar_direct_kinematics(g, [th1;th2]);
        if ~isempty(dirS) && dirS(1).valid
            Px = dirS(1).P(1);
            Py = dirS(1).P(2);
            Px = max(get(data.invSl(1),'Min'),min(get(data.invSl(1),'Max'),Px));
            Py = max(get(data.invSl(2),'Min'),min(get(data.invSl(2),'Max'),Py));
            set(data.invSl(1),'Value',Px);
            set(data.invSl(2),'Value',Py);
            set(data.invTxt(1),'String',sprintf('%.2f',Px));
            set(data.invTxt(2),'String',sprintf('%.2f',Py));
        end
        for k=1:2
            set(data.dirSl(k),'Enable','off');
            set(data.dirTxt(k),'Enable','off');
            set(data.invSl(k),'Enable','on');
            set(data.invTxt(k),'Enable','on');
        end
        guidata(hFig,data);
        updatePlot([],[]);
    end

    function cbDirEdit(k)
        data = guidata(hFig);
        v = str2double(get(data.dirTxt(k),'String'));
        if ~isnan(v)
            v = max(-180, min(180, v));
            set(data.dirSl(k),'Value',v);
        end
        updatePlot([],[]);
    end

    function cbInvEdit(k)
        data = guidata(hFig);
        v = str2double(get(data.invTxt(k),'String'));
        if ~isnan(v)
            v = max(get(data.invSl(k),'Min'), min(get(data.invSl(k),'Max'), v));
            set(data.invSl(k),'Value',v);
        end
        updatePlot([],[]);
    end

    function updatePlot(~,~)
        data = guidata(hFig);
        currentMode = data.modeStr;  % save before readGeo can overwrite data
        try
            g = readGeo();
            % Restore modeStr in case readGeo overwrote data
            data = guidata(hFig);
            if ~strcmp(data.modeStr, currentMode)
                data.modeStr = currentMode;
                guidata(hFig,data);
            end

            selSol = [];
            for k = 1:4
                if get(data.sols_checkbox(k),'Value')
                    selSol(end+1) = k; %#ok<AGROW>
                end
            end

            % Recompute limits from current geometry
            data.limits = computeLimits(g(1),g(2),g(3),g(4),g(5),g(6),g(7));
            % Update inverse slider ranges to match geometry
            set(data.invSl(1),'Min',data.limits(1),'Max',data.limits(2));
            set(data.invSl(2),'Min',data.limits(3),'Max',data.limits(4));
            opts.ax         = data.ax;
            opts.clearAxes  = true;
            opts.limits     = data.limits;
            opts.showLabels = true;
            opts.solutions  = selSol;

            if strcmp(currentMode,'Direct')
                th1 = get(data.dirSl(1),'Value');
                th2 = get(data.dirSl(2),'Value');
                set(data.dirTxt(1),'String',sprintf('%.1f',th1));
                set(data.dirTxt(2),'String',sprintf('%.1f',th2));
                [~, sol] = fivebar_plot(g,'direct',[th1 th2],opts);
                % update checkbox enable states
                for i=1:4
                    if i<=numel(sol) && sol(i).valid
                        set(data.sols_checkbox(i),'Enable','on');
                    else
                        set(data.sols_checkbox(i),'Enable','off');
                    end
                end
                if isOctave
                    info = sprintf('Direct mode:\nth1=%.1f deg  th2=%.1f deg',th1,th2);
                else
                    info = sprintf('Direct mode:\nθ1=%.1f\xB0  θ2=%.1f\xB0',th1,th2);
                end
                for i=1:numel(sol)
                    if sol(i).valid
                        info = sprintf('%s\nSol %d: P=(%.3f, %.3f)', ...
                            info, i, sol(i).P(1), sol(i).P(2));
                    else
                        info = sprintf('%s\nSol %d: invalid', info, i);
                    end
                end
            else
                Px = get(data.invSl(1),'Value');
                Py = get(data.invSl(2),'Value');
                set(data.invTxt(1),'String',sprintf('%.2f',Px));
                set(data.invTxt(2),'String',sprintf('%.2f',Py));
                [~, sol] = fivebar_plot(g,'inverse',[Px Py],opts);
                for i=1:4
                    if i<=numel(sol) && isfield(sol(i),'valid') && sol(i).valid
                        set(data.sols_checkbox(i),'Enable','on');
                    else
                        set(data.sols_checkbox(i),'Enable','off');
                    end
                end
                info = sprintf('Inverse mode:\nP=(%.3f, %.3f)',Px,Py);
                if isempty(sol)
                    info = sprintf('%s\nUnreachable', info);
                end
                for i=1:numel(sol)
                    if isfield(sol(i),'valid') && sol(i).valid
                        if isOctave
                            info = sprintf('%s\nSol %d: th1=%.1f deg  th2=%.1f deg', ...
                                info, i, sol(i).theta(1), sol(i).theta(2));
                        else
                            info = sprintf('%s\nSol %d: θ1=%.1f\xB0  θ2=%.1f\xB0', ...
                                info, i, sol(i).theta(1), sol(i).theta(2));
                        end
                    else
                        info = sprintf('%s\nSol %d: invalid', info, i);
                    end
                end
            end

            axis(data.ax,'equal');
            xlim(data.ax, data.limits(1:2));
            ylim(data.ax, data.limits(3:4));
            set(data.info_text,'String',info);
            guidata(hFig,data);
            drawnow();

        catch ME
            cla(ax);
            text(0,0,'Error','Parent',ax,'Color','r','FontSize',14,'HorizontalAlignment','center');
            set(data.info_text,'String',ME.message);
        end
    end

    function g = readGeo()
        data = guidata(hFig);
        g = zeros(1,8);
        for ii = 1:8
            g(ii) = str2double(get(data.geoEd(ii),'String'));
        end
    end

    % ---- Animation ---------------------------------------------------
    function toggleAnimation(~,~)
        data = guidata(hFig);
        if data.animateFlag
            if ~isOctave
                if ~isempty(data.timerObj) && isvalid(data.timerObj)
                    stop(data.timerObj); delete(data.timerObj);
                end
                data.timerObj = [];
            end
            data.animateFlag = false;
            set(data.animateBtn,'String','Animate');
            guidata(hFig,data);
        else
            data.th1_offset = get(data.dirSl(1),'Value');
            data.th2_offset = get(data.dirSl(2),'Value');
            data.Px_offset  = get(data.invSl(1),'Value');
            data.Py_offset  = get(data.invSl(2),'Value');
            data.animateFlag = true;
            set(data.animateBtn,'String','Stop');
            if ~isOctave
                if ~isempty(data.timerObj) && isvalid(data.timerObj)
                    stop(data.timerObj); delete(data.timerObj);
                end
                t = timer('ExecutionMode','fixedRate','Period',0.033,'TimerFcn',@animateStep);
                data.timerObj = t;
                guidata(hFig,data);
                start(t);
            else
                guidata(hFig,data);
                while true
                    drawnow();
                    data = guidata(hFig);
                    if ~data.animateFlag || ~ishandle(hFig), break; end
                    animateStep([],[]);
                    pause(0.033);
                end
            end
        end
    end

    function animateStep(~,~)
        data = guidata(hFig);
        tnow = now*24*3600;
        if strcmp(data.modeStr,'Direct')
            A1=60; f1=0.1; A2=45; f2=0.15;
            th1 = data.th1_offset + A1*sin(2*pi*f1*tnow);
            th2 = data.th2_offset + A2*sin(2*pi*f2*tnow);
            th1 = max(-180,min(180,th1));
            th2 = max(-180,min(180,th2));
            set(data.dirSl(1),'Value',th1);
            set(data.dirSl(2),'Value',th2);
        else
            R=0.25; f=0.05;
            px = data.Px_offset + R*cos(2*pi*f*tnow);
            py = data.Py_offset + R*sin(2*pi*f*tnow);
            px = max(get(data.invSl(1),'Min'),min(get(data.invSl(1),'Max'),px));
            py = max(get(data.invSl(2),'Min'),min(get(data.invSl(2),'Max'),py));
            set(data.invSl(1),'Value',px);
            set(data.invSl(2),'Value',py);
        end
        updatePlot([],[]);
    end

    % ---- File callbacks ----------------------------------------------
    function cbOpen(hFig)
        warning('off','all');
        [f,p] = uigetfile({'*.mat','MAT-file (*.mat)'},'Open Session File');
        warning('on','all');
        if isequal(f,0), return; end
        warning('off','all');
        s = load(fullfile(char(p),char(f)),'-mat');
        warning('on','all');
        if ~isfield(s,'session'), errordlg('Unrecognised session file.','Open Error'); return; end
        sess = s.session;
        data = guidata(hFig);
        for ii=1:8, set(data.geoEd(ii),'String',num2str(sess.geo(ii))); end
        for k=1:2
            set(data.dirSl(k),'Value',sess.theta(k));
            set(data.dirTxt(k),'String',sprintf('%.1f',sess.theta(k)));
            Pv = [sess.Px, sess.Py];
            set(data.invSl(k),'Value',Pv(k));
            set(data.invTxt(k),'String',sprintf('%.2f',Pv(k)));
        end
        data.modeStr = sess.modeStr;
        if strcmp(sess.modeStr,'Direct')
            set(data.rad1,'Value',1); set(data.rad2,'Value',0);
            for k=1:2; set(data.dirSl(k),'Enable','on'); set(data.dirTxt(k),'Enable','on');
                       set(data.invSl(k),'Enable','off'); set(data.invTxt(k),'Enable','off'); end
        else
            set(data.rad1,'Value',0); set(data.rad2,'Value',1);
            for k=1:2; set(data.dirSl(k),'Enable','off'); set(data.dirTxt(k),'Enable','off');
                       set(data.invSl(k),'Enable','on'); set(data.invTxt(k),'Enable','on'); end
        end
        if isfield(sess,'solsVisible')
            for i=1:min(4,numel(sess.solsVisible))
                set(data.sols_checkbox(i),'Value',sess.solsVisible(i));
            end
        end
        guidata(hFig,data);
        updatePlot([],[]);
    end

    function cbSave(hFig)
        warning('off','all');
        [f,p] = uiputfile({'*.mat','MAT-file (*.mat)'},'Save Session As','fivebar_session.mat');
        warning('on','all');
        if isequal(f,0), return; end
        data = guidata(hFig);
        session.geo        = readGeo();
        session.theta      = [get(data.dirSl(1),'Value'), get(data.dirSl(2),'Value')];
        session.Px         = get(data.invSl(1),'Value');
        session.Py         = get(data.invSl(2),'Value');
        session.modeStr    = data.modeStr;
        session.solsVisible = arrayfun(@(h) get(h,'Value'), data.sols_checkbox);
        save(fullfile(char(p),char(f)),'session','-mat','-v6');
    end

    function cbExportPNG(hFig)
        warning('off','all');
        [f,p] = uiputfile({'*.png','PNG Image (*.png)'},'Export As');
        warning('on','all');
        if isequal(f,0), return; end
        target = fullfile(char(p),char(f));
        if isOctave
            origPU=get(hFig,'PaperUnits'); origPP=get(hFig,'PaperPosition'); origPPM=get(hFig,'PaperPositionMode');
            pos=get(hFig,'Position');
            set(hFig,'PaperUnits','inches','PaperPosition',[0 0 pos(3)/96 pos(4)/96],'PaperPositionMode','manual');
            print(hFig,'-dpng','-r96',target);
            set(hFig,'PaperUnits',origPU,'PaperPosition',origPP,'PaperPositionMode',origPPM);
        elseif exist('exportgraphics','file')
            exportgraphics(hFig,target);
        else
            print(hFig,'-dpng','-r300',target);
        end
    end

    function cbExportEPSPDF(hFig)
        warning('off','all');
        [f,p] = uiputfile({'*.eps','EPS Vector (*.eps)'},'Export As');
        warning('on','all');
        if isequal(f,0), return; end
        target = fullfile(char(p),char(f));
        if exist('exportgraphics','file'); exportgraphics(hFig,target);
        else; print(hFig,'-depsc','-r300',target); end
        unix(strcat(['epstopdf ',target]));
    end

    function cbPrint(hFig); printdlg(hFig); end

    function cbExit(hFig)
        if strcmp(questdlg('Close?','Exit','Yes','No','No'),'Yes'), delete(hFig); end
    end

    function cbResetView(hFig)
        data = guidata(hFig);
        xlim(data.ax,data.limits(1:2)); ylim(data.ax,data.limits(3:4));
        axis(data.ax,'equal');
    end

    function cbPreferences(hFig)
        data = guidata(hFig);
        if strcmp(get(data.ax,'XGrid'),'on'); grid(data.ax,'off');
        else; grid(data.ax,'on'); end
    end

    function lims = computeLimits(a,b,c,d,e,alpha,h)
        % Tight bounding box from the actual reachability circles.
        % Left chain reaches radius (a+b+h) from O=[0,0].
        % Right chain reaches radius (c+d) from D=[Dx,Dy].
        % The workspace intersection defines the tightest sensible box.
        alpha_rad = deg2rad(alpha);
        Dx = e * cos(alpha_rad);
        Dy = e * sin(alpha_rad);
        % Conservative bounds: each pivot ± its chain reach
        x_lo = max(-( a+b+h),   Dx - (c+d));
        x_hi = min(   a+b+h,    Dx + (c+d));
        y_lo = max(-(a+b+h),   Dy - (c+d));
        y_hi = min(   a+b+h,    Dy + (c+d));
        margin = (a+b+c+d) * 0.08;
        x_lo = x_lo - margin;  x_hi = x_hi + margin;
        y_lo = y_lo - margin;  y_hi = y_hi + margin;
        % Equal aspect: expand smaller dimension to match larger
        rx = (x_hi - x_lo) / 2;  cx = (x_lo + x_hi) / 2;
        ry = (y_hi - y_lo) / 2;  cy = (y_lo + y_hi) / 2;
        r  = max(rx, ry);
        lims = [cx-r cx+r cy-r cy+r];
    end

end
