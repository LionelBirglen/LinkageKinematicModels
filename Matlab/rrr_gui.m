function rrr_gui()
% RRR_GUI - Interactive GUI for a Planar RRR Serial Linkage
%
% OBJECTIVE:
%   Launch a graphical interface to simulate and visualize the kinematics
%   of a planar 3R serial manipulator. Supports direct and inverse
%   kinematics, elbow-up/down switching, alternate configuration display,
%   animation, session save/load, and PNG export.
%   Compatible with both MATLAB and Octave.
%
% USAGE EXAMPLE:
%   >> rrr_gui
%   Opens the GUI with default link lengths and joint values.
%
% BY:
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025

% ----------------------------------------------------------------
% 0) Environment detection
% ----------------------------------------------------------------
isOctave = exist('OCTAVE_VERSION','builtin') ~= 0;

% ----------------------------------------------------------------
% 1) Figure
% ----------------------------------------------------------------
FW = 900; FH = 600;   % figure width / height
if isOctave
    hFig = figure('Name','Planar RRR Linkage','NumberTitle','off', ...
        'MenuBar','figure','ToolBar','figure', ...
        'Position',[100 80 FW FH]);
    set(hFig,'Color',[1 1 1]);
    set(0,'DefaultUicontrolBackgroundColor',[1 1 1]);
else
    hFig = figure('Name','Planar RRR Linkage','NumberTitle','off', ...
        'MenuBar','none','ToolBar','figure', ...
        'Position',[100 80 FW FH]);
end

% ----------------------------------------------------------------
% 2) Menu bar
% ----------------------------------------------------------------
hMenuFig = gcf;
if isOctave
    delete(findall(hMenuFig,'Type','uimenu'));
end
isUIFigure = isa(hMenuFig,'matlab.ui.Figure');
if isUIFigure
    parentProp = 'Text';  cbProp = 'MenuSelectedFcn';
else
    parentProp = 'Label'; cbProp = 'Callback';
end
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
def_L   = [57  46  51];
def_t   = [39  37  40];       % direct mode joint angles (deg)
def_inv = [35  125  116];     % Px, Py, phi (deg)
def_elbow = 1;                % 1=elbow-up, 2=elbow-down

% ----------------------------------------------------------------
% 4) Geometry panel  (two columns: a,b,c,d | e,alpha,h,eta)
% ----------------------------------------------------------------
geoPanel = uipanel('Title','Geometry','FontSize',10, ...
    'Units','normalized','Position',[0.02 0.90 0.32 0.09]);
names  = {'L1','L2','L3'};
geoEd  = zeros(1,3);
xLbl   = [0.0 0.34 0.67];   % left edge of each label (panel-normalized)
xEd    = [0.12 0.46 0.79];   % left edge of each edit box
wLbl   = 0.10;  wEd = 0.18;  hRow = 0.55;  yRow = 0.20;
for k = 1:3
    uicontrol('Parent',geoPanel,'Style','text','Units','normalized', ...
        'Position',[xLbl(k) yRow wLbl hRow], ...
        'String',names{k},'HorizontalAlignment','right');
    geoEd(k) = uicontrol('Parent',geoPanel,'Style','edit','Units','normalized', ...
        'Position',[xEd(k) yRow+0.1 wEd hRow], ...
        'String',num2str(def_L(k)),'Callback',@updatePlot);
end

% ----------------------------------------------------------------
% 5) Mode Selection (Direct / Inverse)
% ----------------------------------------------------------------
modePanel = uipanel('Title','Mode','FontSize',10,'Units','normalized', ...
    'Position',[0.02 0.82 0.32 0.07]);
rad1 = uicontrol('Style','radiobutton','String','Direct', ...
    'Units','normalized','Position',[0.1 0.83 0.08 0.035], ...
    'Value',1,'Callback',@cbRad1);
rad2 = uicontrol('Style','radiobutton','String','Inverse', ...
    'Units','normalized','Position',[0.20 0.83 0.08 0.035], ...
    'Value',0,'Callback',@cbRad2);

% TO DO: Replace this with solution checkboxes
elbow_btn = uicontrol('Style','pushbutton','String','Elbow: Up', ...
    'Units','normalized','Position',[0.02 0.760 0.16 0.05], ...
    'Callback',@cbElbow);
show_both_cb = uicontrol('Style','checkbox','String','Show both', ...
    'Units','normalized','Position',[0.20 0.760 0.14 0.05], ...
    'Value',0,'Callback',@updatePlot);

% ----------------------------------------------------------------
% 6) Direct mode sliders (theta1, theta2, theta3)
% ----------------------------------------------------------------
directPanel = uipanel('Title','Direct Mode Sliders','FontSize',10, ...
    'Position',[0.02 0.58 0.32 0.17], ...
    'Visible','on');

dirPanels = zeros(1,3);
dirSl     = zeros(1,3);
dirTxt    = zeros(1,3);
lblDir    = {'theta1','theta2','theta3'};
def_tArr  = def_t;
dirPanelBot = [0.70 0.40 0.10];   % bottom of each single-slider panel
for k = 1:3
    dirLab(k) =uicontrol('Parent',directPanel,'Style','text','Units','normalized', ...
    'Position',[0.04 dirPanelBot(k) 0.1 0.2],'String',strcat(['θ' num2str(k)]),'HorizontalAlignment','left');
    dirSl(k) = uicontrol('Parent',directPanel,'Style','slider', ...
        'Units','normalized','Position',[0.1 dirPanelBot(k) 0.70 0.23], ...
        'Min',-360,'Max',360,'Value',def_tArr(k), ...
        'SliderStep',[1/720 10/720],'Callback',@updatePlot);
    dirTxt(k) = uicontrol('Parent',directPanel,'Style','edit', ...
        'Units','normalized','Position',[0.84 dirPanelBot(k) 0.15 0.23], ...
        'BackgroundColor',[1 1 1], ...
        'String',sprintf('%.1f',def_tArr(k)), ...
        'Callback',@(~,~) cbDirEdit(k));
end
if isOctave
    for k = 1:3
        set(dirTxt(k),'FontSize',10-1*isOctave);
        set(dirTxt(k),'Position',[0.76 dirPanelBot(k) 0.22 0.28]);
    end
end

% ----------------------------------------------------------------
% 7) Inverse mode sliders (x, y, phi)
% ----------------------------------------------------------------
inversePanel = uipanel('Title','Inverse Mode Sliders','FontSize',10, ...
    'Position',[0.02 0.40 0.32 0.17], ...
    'Visible','on');
invPanels = zeros(1,3);
invSl     = zeros(1,3);
invTxt    = zeros(1,3);
lblInv    = {'X target','Y target','phi'};
minInv    = [-500 -500 -360];  maxInv = [500 500 360];
invPanelBot = [0.70 0.40 0.10];
for k = 1:3
    invLab(k) =uicontrol('Parent',inversePanel,'Style','text','Units','normalized', ...
    'Position',[0.04 invPanelBot(k) 0.1 0.2],'String',strcat(['θ' num2str(k)]),'HorizontalAlignment','left');
    invSl(k) = uicontrol('Parent',inversePanel,'Style','slider', ...
        'Units','normalized','Position',[0.1 invPanelBot(k) 0.70 0.23], ...
        'Min',minInv(k),'Max',maxInv(k),'Value',def_inv(k), ...
        'SliderStep',[1/1000 10/1000],'Callback',@updatePlot);
    invTxt(k) = uicontrol('Parent',inversePanel,'Style','edit', ...
        'Units','normalized','Position',[0.84 invPanelBot(k) 0.15 0.23], ...
        'BackgroundColor',[1 1 1], ...
        'String',sprintf('%.1f',def_inv(k)), ...
        'Callback',@(~,~) cbInvEdit(k));
end
set(invLab(1),'String','X');
set(invLab(2),'String','Y');
set(invLab(3),'String','phi');
% Start in Direct mode: disable all inverse controls
for k = 1:3
    set(invSl(k), 'Enable','off');
    set(invTxt(k),'Enable','off');
end
if isOctave
    set(directPanel,'BackgroundColor',[1 1 1]);
    set(inversePanel,'BackgroundColor',[1 1 1]);
    set(geoPanel,'BackgroundColor',[1 1 1]);
    set(modePanel,'BackgroundColor',[1 1 1]);
    for k = 1:3
        set(invTxt(k),'FontSize',10-1*isOctave);
        set(invTxt(k),'Position',[0.76 invPanelBot(k) 0.22 0.28]);
    end
end

% ----------------------------------------------------------------
% 8) Display solutions panel (4 checkboxes)
% ----------------------------------------------------------------
% TO DO: ...


% ----------------------------------------------------------------
% 9) Animate button and info text
% ----------------------------------------------------------------
animate_btn = uicontrol('Style','pushbutton','String','Animate', ...
    'Position',[20 210 285 26],'Callback',@toggleAnimation);

info_text = uicontrol('Style','text','Position',[20 140 285 68], ...
    'FontSize',10-1*isOctave,'HorizontalAlignment','left');

% ----------------------------------------------------------------
% 10) Axes
% ----------------------------------------------------------------
ax = axes('Units','pixels','Position',[310 50 560 500]);
axis equal;
grid on;
if isOctave
    xlabel(ax,'X','FontSize',14); ylabel(ax,'Y','FontSize',14);
    title(ax,'Planar RRR Linkage','FontSize',16);
else
    xlabel(ax,'X'); ylabel(ax,'Y');
    title(ax,'Planar RRR Linkage');
end
hold(ax,'on');
%TO DO: compute and adjust limits here

% ----------------------------------------------------------------
% 11) Store state
% ----------------------------------------------------------------
data.name          = 'RRR Linkage';
data.type          = 1.01;
data.info          = 'Lorem ipsum';
data.author        = 'Lionel Birglen';
data.date          = '20260628';
data.version       = 0.1;
data.geoEd        = geoEd;
data.rad1         = rad1;
data.rad2         = rad2;
data.modeStr      = 'Direct';
data.elbowIdx     = def_elbow;  % 1=elbow-up, 2=elbow-down %TO DO: delete
data.elbow_btn    = elbow_btn;
data.show_both    = show_both_cb;
data.dirSl        = dirSl;
data.dirTxt       = dirTxt;
data.invSl        = invSl;
data.invTxt       = invTxt;
data.animate_btn  = animate_btn;
data.info_text    = info_text;
data.ax           = ax;
data.animateFlag  = false;
data.timerObj     = [];
data.time_offset  = 0;
data.th_offset    = def_t;
data.inv_offset   = def_inv;
data.limits       = [-200 200 -200 200];
data.userZoomed   = false;  % true after user pans/zooms
data.firstPlot    = true;   % skip zoom detection on first draw
guidata(hFig, data);

updatePlot([],[]);

% ================================================================
%  Callbacks
% ================================================================

    function cbRad1(~,~)
        set(rad1,'Value',1); set(rad2,'Value',0);
        data = guidata(hFig);
        data.modeStr = 'Direct';
        guidata(hFig,data);  % save modeStr BEFORE enabling sliders
        for kk = 1:3
            set(data.dirSl(kk), 'Enable','on');
            set(data.dirTxt(kk),'Enable','on');
            set(data.invSl(kk), 'Enable','off');
            set(data.invTxt(kk),'Enable','off');
        end
        geo = readGeo();
        Px  = get(data.invSl(1),'Value');
        Py  = get(data.invSl(2),'Value');
        phi = deg2rad(get(data.invSl(3),'Value'));
        sols = rrr_inverse_kinematics(geo,[Px Py phi]);
        sol  = sols(data.elbowIdx);
        if sol.valid
            th = sol.theta;
            vals = rad2deg(th).';
            for k = 1:3
                set(data.dirSl(k),'Value',vals(k));
                set(data.dirTxt(k),'String',sprintf('%.1f',vals(k)));
            end
        end
        guidata(hFig,data);
        updatePlot([],[]);
    end

    function cbRad2(~,~)
        set(rad1,'Value',0); set(rad2,'Value',1);
        data = guidata(hFig);
        data.modeStr = 'Inverse';
        guidata(hFig,data);  % save modeStr BEFORE enabling sliders
        for kk = 1:3
            set(data.dirSl(kk), 'Enable','off');
            set(data.dirTxt(kk),'Enable','off');
            set(data.invSl(kk), 'Enable','on');
            set(data.invTxt(kk),'Enable','on');
        end
        geo = readGeo();
        t1 = deg2rad(get(data.dirSl(1),'Value'));
        t2 = deg2rad(get(data.dirSl(2),'Value'));
        t3 = deg2rad(get(data.dirSl(3),'Value'));
        sol = rrr_direct_kinematics(geo,[t1 t2 t3]);
        P  = sol.Positions.P;
        phi_deg = mod(rad2deg(sol.phi) + 180, 360) - 180;  % wrap to [-180,180]
        vals = [P(1) P(2) phi_deg];
        for k = 1:3
            v = max(get(data.invSl(k),'Min'), min(get(data.invSl(k),'Max'), vals(k)));
            set(data.invSl(k),'Value',v);
            set(data.invTxt(k),'String',sprintf('%.1f',vals(k)));
        end
        guidata(hFig,data);
        updatePlot([],[]);
    end

    function cbElbow(~,~)
        data = guidata(hFig);
        data.elbowIdx = 3 - data.elbowIdx;  % toggle 1↔2
        if data.elbowIdx == 1
            set(data.elbow_btn,'String','Elbow: Up');
        else
            set(data.elbow_btn,'String','Elbow: Down');
        end
        guidata(hFig,data);
        updatePlot([],[]);
    end

    function cbDirEdit(k)
        data = guidata(hFig);
        v = str2double(get(data.dirTxt(k),'String'));
        if ~isnan(v)
            v = max(get(data.dirSl(k),'Min'), min(get(data.dirSl(k),'Max'), v));
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
        try
            geo = readGeo(); L1=geo.L1; L2=geo.L2; L3=geo.L3;
            R   = (L1+L2+L3) * 1.1;
            lim = [-R R -R R];
            data.limits = lim;
            % Save current axes limits BEFORE clearing (user may have zoomed/panned)
            prevXLim = xlim(data.ax);
            prevYLim = ylim(data.ax);
            tol = (data.limits(2)-data.limits(1)) * 1e-3;
            if ~isfield(data,'userZoomed'), data.userZoomed = false; end
            if ~isfield(data,'firstPlot'),  data.firstPlot  = true;  end
            if data.firstPlot
                userZoomed = false;
                data.firstPlot = false;
                guidata(hFig,data);
            else
                userZoomed = data.userZoomed || ...
                    abs(prevXLim(1)-data.limits(1))>tol || abs(prevXLim(2)-data.limits(2))>tol || ...
                    abs(prevYLim(1)-data.limits(3))>tol || abs(prevYLim(2)-data.limits(4))>tol;
            end
            opts.ax         = data.ax;
            opts.clearAxes  = true;
            opts.showLabels = true;
            opts.limits     = [];  % limits applied after plot, not inside

            if strcmp(data.modeStr,'Direct')
                t1 = deg2rad(get(data.dirSl(1),'Value'));
                t2 = deg2rad(get(data.dirSl(2),'Value'));
                t3 = deg2rad(get(data.dirSl(3),'Value'));
                t1=atan2(sin(t1),cos(t1)); t2=atan2(sin(t2),cos(t2)); t3=atan2(sin(t3),cos(t3));
                t_deg=rad2deg([t1 t2 t3]);
                for kk=1:3, set(data.dirSl(kk),'Value',max(-180,min(180,t_deg(kk)))); end
                for k = 1:3
                    vd = rad2deg([t1 t2 t3]);
                    set(data.dirTxt(k),'String',sprintf('%.1f',vd(k)));
                end
                rrr_plot(geo,'direct',[t1 t2 t3],opts);
                sol = rrr_direct_kinematics(geo,[t1 t2 t3]);
                P  = sol.Positions.P;
                phi_disp = mod(rad2deg(sol.phi) + 180, 360) - 180;
                info = sprintf('Direct mode:\nP=[%.2f; %.2f]  phi=%.2f deg', ...
                    P(1),P(2),phi_disp);
            else
                Px  = get(data.invSl(1),'Value');
                Py  = get(data.invSl(2),'Value');
                phi = deg2rad(get(data.invSl(3),'Value'));
                set(data.invTxt(1),'String',sprintf('%.1f',Px));
                set(data.invTxt(2),'String',sprintf('%.1f',Py));
                set(data.invTxt(3),'String',sprintf('%.1f',rad2deg(phi)));
                opts.showBoth = get(data.show_both,'Value');
                elbSign = 1; if data.elbowIdx==2, elbSign=-1; end
                opts.elbow = elbSign;
                rrr_plot(geo,'inverse',[Px Py phi],opts);
                sols = rrr_inverse_kinematics(geo,[Px Py phi]);
                sol  = sols(data.elbowIdx);
                if sol.valid
                    th = rad2deg(sol.theta);
                    info = sprintf('Inverse mode:\nSol 1: th1=%.1f  th2=%.1f  th3=%.1f deg', ...
                        th(1),th(2),th(3));
                else
                    info = 'Inverse mode: unreachable';
                end
                if get(data.show_both,'Value')
                    sol2 = sols(3 - data.elbowIdx);
                    if sol2.valid
                        th2 = rad2deg(sol2.theta);
                        info = sprintf('%s\nSol 2: th1=%.1f  th2=%.1f  th3=%.1f deg', ...
                            info,th2(1),th2(2),th2(3));
                    else
                        info = sprintf('%s\nSol 2: unreachable',info);
                    end
                end
            end

            if userZoomed
                xlim(data.ax, prevXLim);
                ylim(data.ax, prevYLim);
            else
                axis(data.ax,'equal');
                xlim(data.ax, lim(1:2));
                ylim(data.ax, lim(3:4));
            end
            set(data.info_text,'String',info);
            guidata(hFig,data);
            drawnow();

        catch ME
            cla(ax);
            text(0,0,'Error','Parent',ax,'Color','r','FontSize',14, ...
                'HorizontalAlignment','center');
            set(data.info_text,'String',ME.message);
        end
    end

    function geo = readGeo()
        data = guidata(hFig);
        L1 = str2double(get(data.geoEd(1),'String'));
        L2 = str2double(get(data.geoEd(2),'String'));
        L3 = str2double(get(data.geoEd(3),'String'));
        if isnan(L1)||L1<=0, L1=def_L(1); end
        if isnan(L2)||L2<=0, L2=def_L(2); end
        if isnan(L3)||L3<=0, L3=def_L(3); end
        geo = struct('L1',L1,'L2',L2,'L3',L3);
    end

    % ---- Animation ------------------------------------------------
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
            set(data.animate_btn,'String','Animate');
            guidata(hFig,data);
        else
            data.time_offset = now*24*3600;
            data.th_offset   = [get(data.dirSl(1),'Value') ...
                                 get(data.dirSl(2),'Value') ...
                                 get(data.dirSl(3),'Value')];
            data.inv_offset  = [get(data.invSl(1),'Value') ...
                                 get(data.invSl(2),'Value') ...
                                 get(data.invSl(3),'Value')];
            data.animateFlag = true;
            set(data.animate_btn,'String','Stop');
            if ~isOctave
                if ~isempty(data.timerObj) && isvalid(data.timerObj)
                    stop(data.timerObj); delete(data.timerObj);
                end
                t = timer('ExecutionMode','fixedRate','Period',0.033, ...
                    'TimerFcn',@animateStep);
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
        tnow  = now*24*3600;
        speed = 30;
        if strcmp(data.modeStr,'Direct')
            dt = tnow - data.time_offset;
            speeds = [1.0 0.7 0.5] * speed;  % different rates for each joint
            for k = 1:3
                th = mod(data.th_offset(k) + speeds(k)*dt + 180, 360) - 180;
                set(data.dirSl(k),'Value',th);
            end
        else
            geo = readGeo(); L1=geo.L1; L2=geo.L2; L3=geo.L3;
            R  = L1+L2+L3;
            Px = data.inv_offset(1) + (R*0.3)*sin(0.7*(tnow-data.time_offset));
            Px = max(-R, min(R, Px));
            set(data.invSl(1),'Value',Px);
        end
        updatePlot([],[]);
    end

    % ---- File callbacks ----------------------------------------------
    function cbOpen(hFig)
        warning('off','all');
        [f,p] = uigetfile({'*.mat','MAT-file (*.mat)'},'Open Session File');
        warning('on','all');
        if isequal(f,0), return; end
        full = fullfile(char(p),char(f));
        warning('off','all');
        s = load(full,'-mat');
        warning('on','all');
        if ~isfield(s,'session')
            errordlg('Unrecognised session file.','Open Error'); return;
        end
        sess = s.session;
        data = guidata(hFig);
        for k = 1:3
            set(data.geoEd(k),'String',num2str(sess.geo(k)));
        end
        for k = 1:3
            set(data.dirSl(k), 'Value', sess.theta(k));
            set(data.dirTxt(k),'String',sprintf('%.1f',sess.theta(k)));
        end
        inv_vals = [sess.Px sess.Py sess.phi];
        for k = 1:3
            set(data.invSl(k), 'Value', inv_vals(k));
            set(data.invTxt(k),'String',sprintf('%.1f',inv_vals(k)));
        end
        data.modeStr     = sess.modeStr;
        data.elbowIdx = 1;
        if isfield(sess,'elbowIdx'), data.elbowIdx = sess.elbowIdx;
        elseif isfield(sess,'configState')
            if sess.configState == -1, data.elbowIdx = 2; end
        end
        if data.elbowIdx == 1
            set(data.elbow_btn,'String','Elbow: Up');
        else
            set(data.elbow_btn,'String','Elbow: Down');
        end
        if strcmp(sess.modeStr,'Direct')
            set(data.rad1,'Value',1); set(data.rad2,'Value',0);
        else
            set(data.rad1,'Value',0); set(data.rad2,'Value',1);
        end
        if isfield(sess,'showBoth')
            set(data.show_both,'Value',sess.showBoth);
        end
        guidata(hFig,data);
        updatePlot([],[]);
    end

    function cbSave(hFig)
        warning('off','all');
        [f,p] = uiputfile({'*.mat','MAT-file (*.mat)'},'Save Session As','rrr_session.mat');
        warning('on','all');
        if isequal(f,0), return; end
        target = fullfile(char(p),char(f));
        data = guidata(hFig);
        session.geo         = [str2double(get(data.geoEd(1),'String')) ...
                               str2double(get(data.geoEd(2),'String')) ...
                               str2double(get(data.geoEd(3),'String'))];
        session.theta       = [get(data.dirSl(1),'Value') ...
                               get(data.dirSl(2),'Value') ...
                               get(data.dirSl(3),'Value')];
        session.Px          = get(data.invSl(1),'Value');
        session.Py          = get(data.invSl(2),'Value');
        session.phi         = get(data.invSl(3),'Value');
        session.modeStr     = data.modeStr;
        session.elbowIdx    = data.elbowIdx;
        session.showBoth    = get(data.show_both,'Value');
        save(target,'session','-mat','-v6');
    end

    function cbExportPNG(hFig)
        warning('off','all');
        [f,p] = uiputfile({'*.png','PNG Image (*.png)'},'Export As');
        warning('on','all');
        if isequal(f,0), return; end
        target = fullfile(char(p),char(f));
        if isOctave
            origPU  = get(hFig,'PaperUnits');
            origPP  = get(hFig,'PaperPosition');
            origPPM = get(hFig,'PaperPositionMode');
            pos = get(hFig,'Position');
            set(hFig,'PaperUnits','inches', ...
                     'PaperPosition',[0 0 pos(3)/96 pos(4)/96], ...
                     'PaperPositionMode','manual');
            data   = guidata(hFig);
            axH    = data.ax;
            scatH  = findall(axH,'Type','scatter');
            crossH = findall(axH,'Type','line','Marker','x');
            for kk=1:numel(scatH)
                set(scatH(kk),'SizeData', get(scatH(kk),'SizeData')*16);
            end
            for kk=1:numel(crossH)
                set(crossH(kk),'MarkerSize',get(crossH(kk),'MarkerSize')*4);
                set(crossH(kk),'LineWidth', get(crossH(kk),'LineWidth') *4);
            end
            print(hFig,'-dpng','-r96',target);
            for kk=1:numel(scatH)
                set(scatH(kk),'SizeData', get(scatH(kk),'SizeData')/16);
            end
            for kk=1:numel(crossH)
                set(crossH(kk),'MarkerSize',get(crossH(kk),'MarkerSize')/4);
                set(crossH(kk),'LineWidth', get(crossH(kk),'LineWidth') /4);
            end
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
        if exist('exportgraphics','file')
            exportgraphics(hFig,target);
        else
            print(hFig,'-depsc','-r300',target);
        end
        unix(strcat(['epstopdf ',target]));
    end

    function cbPrint(hFig)
        printdlg(hFig);
    end

    function cbExit(hFig)
        choice = questdlg('Close the RRR GUI?','Exit','Yes','No','No');
        if strcmp(choice,'Yes'), delete(hFig); end
    end

    function cbResetView(hFig)
        data = guidata(hFig);
        data.userZoomed = false;
        guidata(hFig,data);
        if isfield(data,'limits') && numel(data.limits)==4
            axis(data.ax,'equal');
            xlim(data.ax,data.limits(1:2));
            ylim(data.ax,data.limits(3:4));
        end
    end

    function cbPreferences(hFig)
        data = guidata(hFig);
        if strcmp(get(data.ax,'XGrid'),'on')
            grid(data.ax,'off');
        else
            grid(data.ax,'on');
        end
    end

end

function out = sc_ternary(cond, a, b)
if cond, out=a; else out=b; end
end
