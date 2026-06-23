function slidercrank_gui()
% SLIDERCRANK_GUI - Interactive GUI for a Planar Slider-Crank Mechanism
%
% Compatible with both MATLAB and Octave.
% Supports direct/inverse kinematics, elbow config toggle, show-both,
% animation, session save/load, and PNG export.
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
    hFig = figure('Name','Slider-Crank Linkage','NumberTitle','off', ...
        'MenuBar','figure','ToolBar','figure','Position',[100 80 900 600]);
    set(hFig,'Color',[1 1 1]);
    set(0,'DefaultUicontrolBackgroundColor',[1 1 1]);
else
    hFig = figure('Name','Slider-Crank Linkage','NumberTitle','off', ...
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
def_a     = 50;
def_b     = 120;
def_c     = 30;      % extension link B->P
def_sang  = 0;       % slider angle (deg)
def_phi   = 30;      % crank angle (deg)
def_xs    = 140;     % slider position
def_cfg   = 1;       % +1 elbow-down

% ----------------------------------------------------------------
% 4) Controls
% ----------------------------------------------------------------

% Geometry panel: a, b, slider_angle side by side
geoPanel = uipanel('Title','Geometry','FontSize',10, ...
    'Units','normalized','Position',[0.02 0.90 0.32 0.08]);
geoNames = {'a (OA)','b (AB)','c (BP)','Ang. (deg)'};
geoEd    = zeros(1,4);
geoDefaults = {def_a, def_b, def_c, def_sang};
xLbl = [-0.00 0.23 0.46 0.71];
xEd  = [0.03 0.26 0.49 0.79];
wLbl = [0.12 0.12 0.12 0.18];
wEd  = 0.10;  hRow = 0.65;  yRow = 0.20;
for k = 1:4
    labEd(k)=uicontrol('Parent',geoPanel,'Style','text','Units','normalized', ...
        'Position',[xLbl(k) yRow wLbl(k) hRow], ...
        'String',geoNames{k},'HorizontalAlignment','right');
    geoEd(k) = uicontrol('Parent',geoPanel,'Style','edit','Units','normalized', ...
        'Position',[xEd(k)+0.1 yRow+0.1 wEd hRow], ...
        'String',num2str(geoDefaults{k}),'Callback',@updatePlot);
end
if isOctave
    for k = 1:4
        set(geoEd(k),'FontSize',7);
        set(labEd(k),'FontSize',7);
        set(labEd(k),'Position',get(labEd(k),'Position')-0.01)
    end
end

%------------------------------------------------------
% Mode Selection (Direct / Inverse)
%------------------------------------------------------
modePanel = uipanel('Title','Mode','FontSize',10,'Units','normalized', ...
    'Position',[0.02 0.82 0.32 0.07]);
rad1 = uicontrol('Style','radiobutton','String','Direct', ...
    'Units','normalized','Position',[0.1 0.83 0.08 0.035], ...
    'Value',1,'Callback',@cbRad1);
rad2 = uicontrol('Style','radiobutton','String','Inverse', ...
    'Units','normalized','Position',[0.20 0.83 0.08 0.035], ...
    'Value',0,'Callback',@cbRad2);

%------------------------------------------------------
% Direct Mode Controls
%------------------------------------------------------
directPanel = uipanel('Title','Direct Mode Sliders','FontSize',10, ...
    'Units','normalized','Position',[0.02 0.73 0.32 0.08]);
dirPanelBot = [0.25];
uicontrol('Parent',directPanel,'Style','text','Units','normalized', ...
    'Position',[0.04 0.25 0.1 0.60],'String','θ','HorizontalAlignment','left');
dirSl(1) = uicontrol('Parent',directPanel,'Style','slider','Units','normalized', ...
    'Position',[0.1 0.25 0.7 0.6], ...
    'Min',-180,'Max',180,'Value',def_phi, ...
    'SliderStep',[1/720 10/720],'Callback',@updatePlot);
dirTxt(1) = uicontrol('Parent',directPanel,'Style','edit','Units','normalized', ...
    'Position',[0.88 0.25 0.1 0.6], ...
    'BackgroundColor',[1 1 1], ...
    'String',sprintf('%.1f',def_phi),'Callback',@(~,~) cbDirEdit(1));
if isOctave
    set(dirTxt(1),'FontSize',10-1*isOctave);
    set(dirTxt(1),'Position',[0.79 0.15 0.19 0.70]);
end

%------------------------------------------------------
% Inverse Mode Controls
%------------------------------------------------------
inversePanel = uipanel('Title','Inverse Mode Sliders','FontSize',10, ...
    'Units','normalized','Position',[0.02 0.64 0.32 0.08]);
uicontrol('Parent',inversePanel,'Style','text','Units','normalized', ...
    'Position',[0.04 0.25 0.1 0.60],'String','x','HorizontalAlignment','left');
invSl(1) = uicontrol('Parent',inversePanel,'Style','slider','Units','normalized', ...
    'Position',[0.1 0.25 0.70 0.60], ...
    'Min',-(def_a+def_b)*1.1,'Max',(def_a+def_b)*1.1,'Value',def_xs, ...
    'SliderStep',[1/500 10/500],'Callback',@updatePlot);
invTxt(1) = uicontrol('Parent',inversePanel,'Style','edit','Units','normalized', ...
    'Position',[0.83 0.25 0.15 0.60], ...
    'BackgroundColor',[1 1 1], ...
    'String',sprintf('%.1f',def_xs),'Callback',@(~,~) cbInvEdit(1));
if isOctave
    set(invTxt(1),'FontSize',10-1*isOctave);
    set(invTxt(1),'Position',[0.79 0.15 0.19 0.70]);
end

% Panel backgrounds white in Octave
if isOctave
    set(geoPanel,    'BackgroundColor',[1 1 1]);
    set(modePanel,   'BackgroundColor',[1 1 1]);
    set(directPanel, 'BackgroundColor',[1 1 1]);
    set(inversePanel,'BackgroundColor',[1 1 1]);
end

% Start in Direct mode: disable inverse
set(invSl(1), 'Enable','off');
set(invTxt(1),'Enable','off');

% Checkbox: display solutions
solsPanel = uipanel('Title','Display solutions:','FontSize',10, ...
    'Units','normalized','Position',[0.02 0.55 0.32 0.08]);
for i=1:2
    sols_checkbox(i) = uicontrol('Parent',solsPanel,'Units','normalized','Style','checkbox','Position',[0.2+0.2*i 0.3 0.7 0.6], ...
        'String',num2str(i),'Value',1,'Callback',@updatePlot);
end
if isOctave, set(solsPanel,'BackgroundColor',[1 1 1]); end

% Animate button and info text
animate_btn = uicontrol('Style','pushbutton','String','Animate', ...
    'Position',[20 290 285 30],'Callback',@toggleAnimation);

info_text = uicontrol('Style','text','Position',[20 225 285 60], ...
    'FontSize',10-1*isOctave,'HorizontalAlignment','left');

% Axes
ax = axes('Units','pixels','Position',[310 50 560 500]);
axis equal;
grid on;
if isOctave
    xlabel(ax,'X','FontSize',14); ylabel(ax,'Y','FontSize',14);
    title(ax,'Slider-Crank Linkage','FontSize',16);
else
    xlabel(ax,'X'); ylabel(ax,'Y');
    title(ax,'Slider-Crank Linkage');
end
hold(ax,'on');

% ----------------------------------------------------------------
% 5) Store state
% ----------------------------------------------------------------
data.geoEd        = geoEd;
data.rad1         = rad1;
data.rad2         = rad2;
data.modeStr      = 'Direct';
data.sols_checkbox = sols_checkbox;
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
data.phi_offset   = def_phi;
data.xs_offset    = def_xs;
data.limits       = [-(def_a+def_b) (def_a+def_b) -(def_a+def_b) (def_a+def_b)];
guidata(hFig, data);

updatePlot([],[]);

% ================================================================
%  Callbacks
% ================================================================

    function cbRad1(~,~)
        set(rad1,'Value',1); set(rad2,'Value',0);
        data = guidata(hFig);
        data.modeStr = 'Direct';
        guidata(hFig,data);
        set(data.dirSl(1), 'Enable','on');
        set(data.dirTxt(1),'Enable','on');
        set(data.invSl(1), 'Enable','off');
        set(data.invTxt(1),'Enable','off');
        % Convert x_P -> phi (slider holds x_P, back-compute x_B = x_P - c)
        [geo,a,b,c,sang] = readGeo();
        xP = get(data.invSl(1),'Value');
        xs = xP - c;  % x_B = x_P - c
        sol = slidercrank_inverse_kinematics(geo,xs,+1);
        if sol.valid
            phi_deg = rad2deg(sol.phi);
            phi_deg = max(-360, min(360, phi_deg));
            set(data.dirSl(1),'Value',phi_deg);
            set(data.dirTxt(1),'String',sprintf('%.1f',phi_deg));
        end
        guidata(hFig,data);
        updatePlot([],[]);
    end

    function cbRad2(~,~)
        set(rad1,'Value',0); set(rad2,'Value',1);
        data = guidata(hFig);
        data.modeStr = 'Inverse';
        guidata(hFig,data);
        set(data.dirSl(1), 'Enable','off');
        set(data.dirTxt(1),'Enable','off');
        set(data.invSl(1), 'Enable','on');
        set(data.invTxt(1),'Enable','on');
        % Convert phi -> x_P (position of P, not B)
        [geo,a,b,c,sang] = readGeo();
        phi = deg2rad(get(data.dirSl(1),'Value'));
        sol = slidercrank_direct_kinematics(geo,phi,+1);
        if sol.valid
            xP = dot(sol.Positions.P - sol.Positions.O, sol.slider_dir);
            xP = max(get(data.invSl(1),'Min'), min(get(data.invSl(1),'Max'), xP));
            set(data.invSl(1),'Value',xP);
            set(data.invTxt(1),'String',sprintf('%.1f',xP));
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
            [geo,a,b,c,sang] = readGeo();
            R   = (a + b + abs(c)) * 1.15;
            lim = [-R R -R R];
            data.limits = lim;

            % Inverse slider controls x_P; x_B in [-(a+b),(a+b)] => x_P in that range shifted by c
            set(data.invSl(1),'Min',-(a+b)+c,'Max',(a+b)+c);

            opts.ax         = data.ax;
            opts.clearAxes  = true;
            opts.showLabels = true;
            opts.limits     = lim;
            sol1_on = get(data.sols_checkbox(1),'Value');
            sol2_on = get(data.sols_checkbox(2),'Value');
            opts.showBoth   = sol2_on;
            opts.showFirst  = sol1_on;

            if strcmp(data.modeStr,'Direct')
                phi_deg = get(data.dirSl(1),'Value');
                set(data.dirTxt(1),'String',sprintf('%.1f',phi_deg));
                cfg1 = +1; cfg2 = -1;
                inputs = [deg2rad(phi_deg), cfg1];
                opts.showFirst = sol1_on;
                slidercrank_plot(geo,'direct',inputs,opts);
                info = 'Direct mode:';
                if sol1_on
                    sol = slidercrank_direct_kinematics(geo,deg2rad(phi_deg),cfg1);
                    if sol.valid
                        xP1 = dot(sol.Positions.P - sol.Positions.O, sol.slider_dir);
                        info = sprintf('%s\nSol 1: phi=%.2f deg   x=%.2f', info, phi_deg, xP1);
                    else
                        info = sprintf('%s\nSol 1: unreachable', info);
                    end
                end
                if sol2_on
                    sol2 = slidercrank_direct_kinematics(geo,deg2rad(phi_deg),cfg2);
                    if sol2.valid
                        xP2 = dot(sol2.Positions.P - sol2.Positions.O, sol2.slider_dir);
                        info = sprintf('%s\nSol 2: phi=%.2f deg   x=%.2f', info, phi_deg, xP2);
                    else
                        info = sprintf('%s\nSol 2: unreachable', info);
                    end
                end
                if strcmp(info,'Direct mode:'), info = 'Direct mode: no solution selected'; end
            else
                xP = get(data.invSl(1),'Value');
                set(data.invTxt(1),'String',sprintf('%.1f',xP));
                xs = xP - c;  % x_B = x_P - c
                cfg1 = +1; cfg2 = -1;
                inputs = [xs, cfg1];
                opts.showFirst = sol1_on;
                slidercrank_plot(geo,'inverse',inputs,opts);
                info = 'Inverse mode:';
                if sol1_on
                    sol = slidercrank_inverse_kinematics(geo,xs,cfg1);
                    if sol.valid
                        info = sprintf('%s\nSol 1: x=%.2f   phi=%.2f deg', info, xP, rad2deg(sol.phi));
                    else
                        info = sprintf('%s\nSol 1: unreachable', info);
                    end
                end
                if sol2_on
                    sol2 = slidercrank_inverse_kinematics(geo,xs,cfg2);
                    if sol2.valid
                        info = sprintf('%s\nSol 2: x=%.2f   phi=%.2f deg', info, xP, rad2deg(sol2.phi));
                    else
                        info = sprintf('%s\nSol 2: unreachable', info);
                    end
                end
                if strcmp(info,'Inverse mode:'), info = 'Inverse mode: no solution selected'; end
            end

            axis(data.ax,'equal');
            xlim(data.ax, lim(1:2));
            ylim(data.ax, lim(3:4));
            set(data.info_text,'String',info);
            guidata(hFig,data);
            drawnow();

        catch ME
            cla(ax);
            text(0,0,'Error','Parent',ax,'Color','r','FontSize',14,'HorizontalAlignment','center');
            set(data.info_text,'String',ME.message);
        end
    end

    function [geo,a,b,c,sang] = readGeo()
        data = guidata(hFig);
        a    = str2double(get(data.geoEd(1),'String'));
        b    = str2double(get(data.geoEd(2),'String'));
        c    = str2double(get(data.geoEd(3),'String'));
        sang = str2double(get(data.geoEd(4),'String'));
        if isnan(a)||a<=0, a=def_a; end
        if isnan(b)||b<=0, b=def_b; end
        if isnan(c), c=def_c; end
        if isnan(sang), sang=def_sang; end
        geo = struct('a',a,'b',b,'c',c,'slider_angle',deg2rad(sang));
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
            set(data.animate_btn,'String','Animate');
            guidata(hFig,data);
        else
            data.time_offset = now*24*3600;
            data.phi_offset  = get(data.dirSl(1),'Value');
            data.xs_offset   = get(data.invSl(1),'Value');
            data.animateFlag = true;
            set(data.animate_btn,'String','Stop');
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
        tnow  = now*24*3600;
        speed = 40;  % deg/s for direct; units/s for inverse
        if strcmp(data.modeStr,'Direct')
            phi = mod(data.phi_offset + speed*(tnow-data.time_offset) + 180, 360) - 180;
            set(data.dirSl(1),'Value',phi);
        else
            [geo,a,b,c,~] = readGeo();
            xP_min = -(a+b) + c;  % min x_P = min x_B + c
            xP_max =  (a+b) + c;  % max x_P = max x_B + c
            xP_mid = (xP_min + xP_max) / 2;
            xP_amp = (xP_max - xP_min) / 2;
            xPv = xP_mid + xP_amp * sin(0.8*(tnow - data.time_offset));
            set(data.invSl(1),'Value',xPv);
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
        set(data.geoEd(1),'String',num2str(sess.a));
        set(data.geoEd(2),'String',num2str(sess.b));
        if isfield(sess,'c'), set(data.geoEd(3),'String',num2str(sess.c)); end
        set(data.geoEd(4),'String',num2str(sess.sang));
        set(data.dirSl(1), 'Value',sess.phi);
        set(data.dirTxt(1),'String',sprintf('%.1f',sess.phi));
        xs = max(get(data.invSl(1),'Min'),min(get(data.invSl(1),'Max'),sess.xs));
        set(data.invSl(1), 'Value',xs);
        set(data.invTxt(1),'String',sprintf('%.1f',sess.xs));
        data.modeStr     = sess.modeStr;
        if strcmp(sess.modeStr,'Direct')
            set(data.rad1,'Value',1); set(data.rad2,'Value',0);
            set(data.dirSl(1),'Enable','on');  set(data.dirTxt(1),'Enable','on');
            set(data.invSl(1),'Enable','off'); set(data.invTxt(1),'Enable','off');
        else
            set(data.rad1,'Value',0); set(data.rad2,'Value',1);
            set(data.dirSl(1),'Enable','off'); set(data.dirTxt(1),'Enable','off');
            set(data.invSl(1),'Enable','on');  set(data.invTxt(1),'Enable','on');
        end
        if isfield(sess,'solsVisible')
            for ii = 1:min(2,numel(sess.solsVisible))
                set(data.sols_checkbox(ii),'Value',sess.solsVisible(ii));
            end
        end
        guidata(hFig,data);
        updatePlot([],[]);
    end

    function cbSave(hFig)
        warning('off','all');
        [f,p] = uiputfile({'*.mat','MAT-file (*.mat)'},'Save Session As','slidercrank_session.mat');
        warning('on','all');
        if isequal(f,0), return; end
        data = guidata(hFig);
        session.a           = str2double(get(data.geoEd(1),'String'));
        session.b           = str2double(get(data.geoEd(2),'String'));
        session.c           = str2double(get(data.geoEd(3),'String'));
        session.sang        = str2double(get(data.geoEd(4),'String'));
        session.phi         = get(data.dirSl(1),'Value');
        session.xs          = get(data.invSl(1),'Value');
        session.modeStr     = data.modeStr;
        session.solsVisible = [get(data.sols_checkbox(1),'Value') get(data.sols_checkbox(2),'Value')];
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
            data=guidata(hFig); axH=data.ax;
            scatH=findall(axH,'Type','scatter'); crossH=findall(axH,'Type','line','Marker','x');
            for kk=1:numel(scatH), set(scatH(kk),'SizeData',get(scatH(kk),'SizeData')*16); end
            for kk=1:numel(crossH), set(crossH(kk),'MarkerSize',get(crossH(kk),'MarkerSize')*4); set(crossH(kk),'LineWidth',get(crossH(kk),'LineWidth')*4); end
            print(hFig,'-dpng','-r96',target);
            for kk=1:numel(scatH), set(scatH(kk),'SizeData',get(scatH(kk),'SizeData')/16); end
            for kk=1:numel(crossH), set(crossH(kk),'MarkerSize',get(crossH(kk),'MarkerSize')/4); set(crossH(kk),'LineWidth',get(crossH(kk),'LineWidth')/4); end
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

    function cbPrint(hFig);  printdlg(hFig); end

    function cbExit(hFig)
        if strcmp(questdlg('Close?','Exit','Yes','No','No'),'Yes'), delete(hFig); end
    end

    function cbResetView(hFig)
        data = guidata(hFig);
        if isfield(data,'limits') && numel(data.limits)==4
            xlim(data.ax,data.limits(1:2)); ylim(data.ax,data.limits(3:4));
        end
        axis(data.ax,'equal');
    end

    function cbPreferences(hFig)
        data = guidata(hFig);
        if strcmp(get(data.ax,'XGrid'),'on'); grid(data.ax,'off');
        else; grid(data.ax,'on'); end
    end

end
