function stephensonIII_gui
% STEPHENSONIII_GUI  Interactive GUI for a Stephenson III six-bar linkage
%
%   This GUI lets you:
%     - Enter the geometry [OA, Bx, By, OC, CD, DA, BF, FE, DE, EP, eta, delta]
%       (eta and delta in degrees)
%     - Choose Direct or Inverse mode
%     - Direct mode : slide thetaO (input crank angle) and see the linkage
%     - Inverse mode: slide thetaB (angle of BF w.r.t. BO) and see all
%       solutions
%     - Select which solutions are displayed (one color per solution)
%     - Animate the linkage (Animate / Stop button)
%     - Save and open sessions (File menu), as in fourbar_gui_extended
%
%   Drawing is delegated to stephensonIII_plot.m (same visual style as
%   fourbar_plot.m).
%
%   Requirements (on the path): stephensonIII_plot.m,
%   stephensonIII_direct_kinematics.m, stephensonIII_inverse_kinematics.m
%
%   To run: >> stephensonIII_gui
%
% AXES LIMITS:
%   The view is fixed from the geometry (local function computeLimits
%   at the end of this file), so it does not move during animation or
%   while the sliders are dragged. Once the user zooms or pans, that
%   view is kept until View > Reset View.
%
% ANIMATION (as in fourbar_gui_extended):
%   Direct mode : thetaO turns at 10 deg/s, wrapped to [-180,180).
%   Inverse mode: thetaB turns at 10 deg/s, wrapped to [-180,180).
%   MATLAB uses a timer (the GUI stays responsive), Octave a
%   drawnow/pause loop, at about 30 frames per second. Each solution
%   keeps its number (and color) from one frame to the next: the new
%   solutions are matched to the previous frame's by stephensonIII_plot
%   (opts.track), so the linkage never jumps to another assembly mode.
%   The same tracking applies when the sliders are moved; numbering
%   restarts when the geometry or the mode changes. Changing mode,
%   opening a session or closing the window stops it.
%
% SESSIONS (File menu, as in fourbar_gui_extended):
%   Save writes a MAT-file (-v6, readable by MATLAB and Octave) holding a
%   struct 'session' with fields geo (1x12), thetaO, thetaB (deg),
%   modeStr, solsVisible (1x6) and axesXLim/axesYLim. Open restores them.

% ----------------------------------------------------------------
% 0) Environment detection
% ----------------------------------------------------------------
isOctave = exist('OCTAVE_VERSION','builtin') ~= 0;

% ----------------------------------------------------------------
% 1) Figure and menus (as in fourbar_gui_extended)
% ----------------------------------------------------------------
if isOctave
    hFig = figure('Name','Stephenson III Linkage GUI','NumberTitle','off', ...
        'MenuBar','figure','ToolBar','figure','Position',[200 200 900 620]);
    set(hFig,'Color',[1 1 1]);
    % Remove Octave's built-in menus so only our custom ones remain
    delete(findall(hFig,'Type','uimenu'));
else
    hFig = figure('Name','Stephenson III Linkage GUI','NumberTitle','off', ...
        'MenuBar','none','ToolBar','figure','Position',[200 200 900 620]);
end

mFile = uimenu(hFig,'Label','File');
uimenu(mFile,'Label','Open','Callback',@cbOpen);
uimenu(mFile,'Label','Save','Callback',@cbSave);
uimenu(mFile,'Label','Exit','Callback',@cbExit,'Separator','on');
mView = uimenu(hFig,'Label','View');
uimenu(mView,'Label','Reset View','Callback',@cbResetView);

% ----------------------------------------------------------------
% 2) Geometry panel
% ----------------------------------------------------------------
geoLabels   = {'OA','Bx','By','OC','CD','DA','BF','FE','DE','EP','η (°)','δ (°)'};
geoDefaults = [40 70 30 50 20 50 30 30 -30 20 30 60];

uipanel('Title','Geometry','FontSize',10,'Position',[0.02 0.71 0.32 0.29]);
geoEd = zeros(1,12);
for k = 1:6
    uicontrol('Style','text','Units','normalized','Position',[0.03 0.96-0.04*k 0.035 0.035], ...
        'String',geoLabels{k},'HorizontalAlignment','right');
    geoEd(k) = uicontrol('Style','edit','Units','normalized','Position',[0.07 0.97-0.04*k 0.06 0.035], ...
        'String',num2strExact(geoDefaults(k)),'Callback',@cbGeometry);
end
for k = 7:12
    uicontrol('Style','text','Units','normalized','Position',[0.175 0.96-0.04*(k-6) 0.05 0.035], ...
        'String',geoLabels{k},'HorizontalAlignment','right');
    geoEd(k) = uicontrol('Style','edit','Units','normalized','Position',[0.23 0.97-0.04*(k-6) 0.06 0.035], ...
        'String',num2strExact(geoDefaults(k)),'Callback',@cbGeometry);
end

% ----------------------------------------------------------------
% 3) Mode selection (radio buttons, as in fourbar_gui_extended)
% ----------------------------------------------------------------
modePanel = uipanel('Title','Mode','FontSize',10,'Position',[0.02 0.61 0.32 0.09]);
rad1 = uicontrol('Parent',modePanel,'Style','radiobutton','String','Direct', ...
    'Units','normalized','Position',[0.10 0.15 0.35 0.7],'Value',1,'Callback',@cbRad1);
rad2 = uicontrol('Parent',modePanel,'Style','radiobutton','String','Inverse', ...
    'Units','normalized','Position',[0.55 0.15 0.35 0.7],'Value',0,'Callback',@cbRad2);

% ----------------------------------------------------------------
% 4) Direct and inverse sliders
% ----------------------------------------------------------------
uipanel('Title','Direct Kinematics','FontSize',10,'Position',[0.02 0.52 0.32 0.08]);
uicontrol('Style','text','Units','normalized','Position',[0.03 0.525 0.025 0.035], ...
    'String','θO','HorizontalAlignment','right');
thetaO_slider = uicontrol('Style','slider','Units','normalized','Position',[0.06 0.53 0.22 0.035], ...
    'Min',-180,'Max',180,'Value',90,'SliderStep',[1/360 0.1],'Callback',@updatePlot);
thetaO_edit = uicontrol('Style','edit','Units','normalized','Position',[0.29 0.53 0.045 0.035], ...
    'String','90','Callback',@cbThetaOEdit);

uipanel('Title','Inverse Kinematics','FontSize',10,'Position',[0.02 0.44 0.32 0.08]);
uicontrol('Style','text','Units','normalized','Position',[0.03 0.445 0.025 0.035], ...
    'String','θB','HorizontalAlignment','right');
thetaB_slider = uicontrol('Style','slider','Units','normalized','Position',[0.06 0.45 0.22 0.035], ...
    'Min',-180,'Max',180,'Value',69,'SliderStep',[1/360 0.1],'Callback',@updatePlot);
thetaB_edit = uicontrol('Style','edit','Units','normalized','Position',[0.29 0.45 0.045 0.035], ...
    'String','69','Callback',@cbThetaBEdit);

% ----------------------------------------------------------------
% 5) Display solutions (6 checkboxes, one color per solution: at most
%    4 solutions in direct mode and 6 in inverse mode)
% ----------------------------------------------------------------
nSlots = 6;
solsPanel = uipanel('Title','Display solutions:','FontSize',10, ...
    'Position',[0.02 0.34 0.32 0.10]);
sols_checkbox = zeros(1,nSlots);
for i = 1:nSlots
    row = floor((i-1)/3);  colI = mod(i-1,3);
    sols_checkbox(i) = uicontrol('Parent',solsPanel,'Units','normalized', ...
        'Style','checkbox','Position',[0.10+0.30*colI 0.52-0.45*row 0.25 0.4], ...
        'String',num2str(i),'Value',0,'Callback',@updatePlot);
end
set(sols_checkbox(2),'Value',1);

% ----------------------------------------------------------------
% 6) Animate button and info text
% ----------------------------------------------------------------
animate_btn = uicontrol('Style','pushbutton','String','Animate', ...
    'Units','normalized','Position',[0.02 0.28 0.32 0.05], ...
    'Callback',@toggleAnimation);

solTxt = uicontrol('Style','text','Units','normalized','Position',[0.02 0.02 0.33 0.25], ...
    'FontSize',10 - 1*isOctave,'HorizontalAlignment','left','String','');

% ----------------------------------------------------------------
% 7) Axes
% ----------------------------------------------------------------
axesPanel = uipanel('Title','Linkage Plot','FontSize',10,'Position',[0.36 0.09 0.62 0.87]);
ax = axes('Parent',axesPanel,'Units','normalized','Position',[0.08 0.08 0.88 0.86]);
axis(ax,'equal'); grid(ax,'on');
xlabel(ax,'X'); ylabel(ax,'Y');

% Octave: white background for all widgets (as in fourbar_gui_extended)
if isOctave
    set(0,'DefaultUicontrolBackgroundColor',[1 1 1]);
    objs = findobj(hFig,'Type','uicontrol');
    for oi = 1:numel(objs)
        if ~strcmp(get(objs(oi),'Style'),'pushbutton')
            set(objs(oi),'BackgroundColor',[1 1 1]);
        end
    end
    objs = findobj(hFig,'Type','uipanel');
    for oi = 1:numel(objs)
        set(objs(oi),'BackgroundColor',[1 1 1]);
    end
end

% ----------------------------------------------------------------
% 8) State
% ----------------------------------------------------------------
modeStr      = 'Direct';  % 'Direct' or 'Inverse'
last_limits  = [];        % limits last applied by the GUI
user_view    = false;     % true once the user zoomed/panned

anim_running = false;     % animation state
anim_timer   = [];        % MATLAB timer object
anim_t0      = [];        % tic reference of the animation start
anim_x0      = [];        % slider value when the animation started
prev_sols    = [];        % previous frame's solutions (for tracking)

% Freeze the view as soon as a zoom or pan ends (MATLAB); in Octave the
% limit comparison in updatePlot does the detection
try
    set(zoom(hFig),'ActionPostCallback',@(~,~) markUserView());
    set(pan(hFig), 'ActionPostCallback',@(~,~) markUserView());
catch
end

% Stop the animation (and delete the timer) when the window is closed
set(hFig,'DeleteFcn',@(~,~) stopAnimation());

applyMode('Direct');
updatePlot();

% ================================================================
%  Callbacks
% ================================================================

    function geo = getGeometry()
        geo = zeros(1,12);
        for i2 = 1:12
            geo(i2) = str2double(get(geoEd(i2),'String'));
        end
    end

    function applyMode(newMode)
        % APPLYMODE - Set the mode, radio buttons and enabled sliders
        modeStr = newMode;
        prev_sols = [];           % new mode: restart solution numbering
        if strcmp(newMode,'Direct')
            set(rad1,'Value',1); set(rad2,'Value',0);
            set([thetaO_slider thetaO_edit],'Enable','on');
            set([thetaB_slider thetaB_edit],'Enable','off');
        else
            set(rad1,'Value',0); set(rad2,'Value',1);
            set([thetaO_slider thetaO_edit],'Enable','off');
            set([thetaB_slider thetaB_edit],'Enable','on');
        end
    end

    function cbRad1(~,~)
        stopAnimation();
        applyMode('Direct');
        updatePlot();
    end

    function cbRad2(~,~)
        stopAnimation();
        applyMode('Inverse');
        updatePlot();
    end

    function cbGeometry(~,~)
        % New geometry: new default view, restart solution numbering
        prev_sols   = [];
        user_view   = false;
        last_limits = [];
        updatePlot();
    end

    function cbThetaOEdit(~,~)
        val = str2double(get(thetaO_edit,'String'));
        if isfinite(val)
            set(thetaO_slider,'Value',max(min(val,180),-180));
        end
        updatePlot();
    end

    function cbThetaBEdit(~,~)
        val = str2double(get(thetaB_edit,'String'));
        if isfinite(val)
            set(thetaB_slider,'Value',max(min(val,180),-180));
        end
        updatePlot();
    end

    function markUserView()
        user_view = true;
    end

    function cbResetView(~,~)
        user_view   = false;
        last_limits = [];
        updatePlot();
    end

    function updatePlot(~,~)
        % UPDATEPLOT - Compute kinematics and draw with stephensonIII_plot
        try
            geo = getGeometry();
            if any(~isfinite(geo))
                error('Invalid geometry value.');
            end

            % Axis limits: fixed from geometry unless the user zoomed/panned
            cur = [get(ax,'XLim') get(ax,'YLim')];
            if ~user_view && ~isempty(last_limits)
                tol = 1e-6 * max(abs(last_limits(2)-last_limits(1)), 1);
                user_view = any(abs(cur - last_limits) > tol);
            end
            if user_view
                lims = cur;
            else
                lims = computeLimits(geo);
            end

            % Solutions the user wants to see
            selSol = find(arrayfun(@(h) get(h,'Value') == 1, sols_checkbox));

            opts = struct('ax',ax,'clearAxes',true,'showLabels',true, ...
                          'limits',lims,'solutions',selSol, ...
                          'nSlots',nSlots);
            opts.track = prev_sols;   % keep each solution in its slot
            if isempty(selSol)
                opts.solutions = 0;   % nothing selected: draw no solution
            end

            if strcmp(modeStr,'Direct')
                thetaO = get(thetaO_slider,'Value');
                set(thetaO_edit,'String',sprintf('%.1f',thetaO));
                [~, sols] = stephensonIII_plot(geo, 'direct', thetaO, opts);
                info_str = sprintf('Direct mode: θO = %.1f°\n', thetaO);
            else
                thetaB = get(thetaB_slider,'Value');
                set(thetaB_edit,'String',sprintf('%.1f',thetaB));
                [~, sols] = stephensonIII_plot(geo, 'inverse', thetaB, opts);
                info_str = sprintf('Inverse mode: θB = %.1f°\n', thetaB);
            end
            prev_sols = sols;
            title(ax, sprintf('Stephenson III Linkage - %s Mode', modeStr));

            drawnow();
            last_limits = [get(ax,'XLim') get(ax,'YLim')];

            % Enable only the checkboxes of existing, valid solutions
            valid = arrayfun(@(s) isfield(s,'valid') && s.valid, sols);
            for i2 = 1:nSlots
                if i2 <= numel(sols) && valid(i2)
                    set(sols_checkbox(i2),'Enable','on');
                else
                    set(sols_checkbox(i2),'Enable','off');
                end
            end

            % Info text
            if ~any(valid)
                info_str = sprintf('%sNo valid solution.', info_str);
            else
                for i2 = find(valid(:).')   % row: the kinematics return a column
                    Pp = sols(i2).Positions.P;
                    info_str = sprintf('%sSol %d: P=[%.2f; %.2f] θO=%.1f° θB=%.1f°\n', ...
                        info_str, i2, Pp(1), Pp(2), ...
                        sols(i2).Angles.thetaO, sols(i2).Angles.thetaB);
                end
            end
            set(solTxt,'String',info_str);

        catch ME
            cla(ax);
            text(0.5,0.5,sprintf('Error: %s',ME.message),'Parent',ax, ...
                'Units','normalized','Color','r','FontSize',12, ...
                'HorizontalAlignment','center');
            set(solTxt,'String','Unreachable configuration');
        end
    end

    % ---- Animation (same scheme as fourbar_gui_extended) -------------
    function toggleAnimation(~,~)
        if anim_running
            stopAnimation();
            return;
        end
        if strcmp(modeStr,'Direct')
            anim_x0 = get(thetaO_slider,'Value');
        else
            anim_x0 = get(thetaB_slider,'Value');
        end
        anim_t0      = tic;
        anim_running = true;
        set(animate_btn,'String','Stop');
        if ~isOctave
            % MATLAB: timer-based, non-blocking
            anim_timer = timer('ExecutionMode','fixedRate', ...
                'Period',0.033,'BusyMode','drop', ...
                'TimerFcn',@(~,~) animateStep());
            start(anim_timer);
        else
            % Octave: no timers, loop while processing GUI events
            while anim_running && ishghandle(hFig)
                animateStep();
                drawnow();          % lets the Stop button be pressed
                pause(0.033);       % ~30 fps
            end
        end
    end

    function stopAnimation()
        anim_running = false;
        if ~isempty(anim_timer)
            try
                if isvalid(anim_timer)
                    stop(anim_timer);
                    delete(anim_timer);
                end
            catch
            end
            anim_timer = [];
        end
        if ishghandle(animate_btn)
            set(animate_btn,'String','Animate');
        end
    end

    function animateStep()
        if ~anim_running || ~ishghandle(hFig)
            stopAnimation();
            return;
        end
        speed = 10;   % deg/s, as in fourbar_gui_extended
        val = mod(anim_x0 + speed*toc(anim_t0) + 180, 360) - 180;
        if strcmp(modeStr,'Direct')
            set(thetaO_slider,'Value',val);
        else
            set(thetaB_slider,'Value',val);
        end
        updatePlot();
    end

    % ---- File callbacks (as in fourbar_gui_extended) ------------------
    function cbOpen(~,~)
        stopAnimation();
        warning('off','all');
        [f,p] = uigetfile({'*.mat','MAT-file (*.mat)'},'Open Session File');
        warning('on','all');
        if isequal(f,0), return; end
        full = fullfile(char(p),char(f));
        try
            warning('off','all');
            S = load(full,'-mat');
            warning('on','all');
        catch ME
            warning('on','all');
            errordlg(sprintf('Could not read file:\n%s',ME.message),'Open Error');
            return;
        end
        if isfield(S,'session') && isstruct(S.session) && ...
                isfield(S.session,'geo') && numel(S.session.geo) == 12
            session = S.session;
        elseif isfield(S,'geo') && isnumeric(S.geo) && numel(S.geo) == 12
            % Bare geometry vector: keep current inputs, mode and view
            session = struct('geo',S.geo);
        else
            errordlg(['Unrecognised session file format ' ...
                '(not a Stephenson III session).'],'Open Error');
            return;
        end

        for i2 = 1:12
            set(geoEd(i2),'String',num2strExact(session.geo(i2)));
        end
        prev_sols = [];           % new session: restart solution numbering
        if isfield(session,'thetaO')
            set(thetaO_slider,'Value',max(min(session.thetaO,180),-180));
        end
        if isfield(session,'thetaB')
            set(thetaB_slider,'Value',max(min(session.thetaB,180),-180));
        end
        if isfield(session,'modeStr') && any(strcmp(session.modeStr,{'Direct','Inverse'}))
            applyMode(session.modeStr);
        end
        if isfield(session,'solsVisible')
            for i2 = 1:min(nSlots,numel(session.solsVisible))
                set(sols_checkbox(i2),'Value',double(session.solsVisible(i2) ~= 0));
            end
        end

        % View: saved limits are restored (and frozen) if they differ
        % from the geometry-based ones
        user_view   = false;
        last_limits = [];
        if isfield(session,'axesXLim') && isfield(session,'axesYLim') && ...
                numel(session.axesXLim) == 2 && numel(session.axesYLim) == 2
            gl  = computeLimits(session.geo(:).');
            sav = [session.axesXLim(:).' session.axesYLim(:).'];
            tol = 1e-6 * max(abs(gl(2)-gl(1)), 1);
            if any(abs(sav - gl) > tol)
                xlim(ax, session.axesXLim);
                ylim(ax, session.axesYLim);
                user_view = true;
            end
        end
        updatePlot();
    end

    function cbSave(~,~)
        warning('off','all');
        [f,p] = uiputfile({'*.mat','MAT-file (*.mat)'},'Save Session As', ...
            'stephensonIII_session.mat');
        warning('on','all');
        if isequal(f,0), return; end
        target = fullfile(char(p),char(f));

        session = struct();
        session.name        = 'Stephenson III Linkage';
        session.version     = 1;
        session.geo         = getGeometry();         % eta, delta in deg
        session.thetaO      = get(thetaO_slider,'Value');   % deg
        session.thetaB      = get(thetaB_slider,'Value');   % deg
        session.modeStr     = modeStr;
        session.solsVisible = arrayfun(@(h) get(h,'Value'), sols_checkbox);
        session.axesXLim    = get(ax,'XLim');
        session.axesYLim    = get(ax,'YLim');
        try
            save(target,'session','-mat','-v6');
        catch ME
            errordlg(sprintf('Could not save file:\n%s',ME.message),'Save Error');
        end
    end

    function cbExit(~,~)
        stopAnimation();
        choice = questdlg('Are you sure you want to exit?','Exit', ...
            'Yes','No','No');
        if strcmp(choice,'Yes')
            close(hFig);
        end
    end

end


% =====================================================================
%  Local functions (outside the nested scope: no shared variables)
% =====================================================================
function lims = computeLimits(geo, margin)
% COMPUTELIMITS - Fixed axis limits enclosing every reachable pose
%
% Returns [xmin xmax ymin ymax] such that O, A, B, C, D, F, E and P stay
% inside the box for ANY input and assembly mode. The bound comes from
% the triangle inequality along the chains of rigid bodies starting at
% the ground pivots; bounds from several chains are intersected. It
% depends on the geometry only, so the view does not move while the
% linkage is animated.
if nargin < 2, margin = 0.05; end
OA = geo(1); B = [geo(2); geo(3)];
OC = abs(geo(4)); CD = geo(5); DA = abs(geo(6)); BF = abs(geo(7));
FE = geo(8); DE = geo(9); EP = geo(10); eta = geo(11); delta = geo(12);
O = [0; 0];  A = [OA; 0];

% Branch-independent distances C-E (body C-D-E) and F-P (body F-E-P),
% built with the same formulas as stephensonIII_direct_kinematics
Eloc = [CD; 0] + DE * [cosd(delta) -sind(delta); sind(delta) cosd(delta)] * [-1; 0];
CE   = norm(Eloc);
FP   = norm([FE; 0] - EP * [cosd(eta); sind(eta)]);

bC = discBox(O, OC);
bD = isectBox(discBox(A, DA), discBox(O, OC + abs(CD)));
bE = isectBox(discBox(A, DA + abs(DE)), discBox(O, OC + CE));
bF = isectBox(discBox(B, BF), growBox(bE, abs(FE)));
bP = isectBox(growBox(bE, abs(EP)), discBox(B, BF + FP));
boxes = [bC; bD; bE; bF; bP; ...
         [O(1) O(1) O(2) O(2)]; [A(1) A(1) A(2) A(2)]; [B(1) B(1) B(2) B(2)]];

lims = [min(boxes(:,1)) max(boxes(:,2)) min(boxes(:,3)) max(boxes(:,4))];
m = margin * max(lims(2)-lims(1), lims(4)-lims(3));
lims = lims + [-m m -m m];
end

function b = discBox(c, r)
b = [c(1)-r, c(1)+r, c(2)-r, c(2)+r];
end

function b = growBox(b, r)
b = b + [-r r -r r];
end

function b = isectBox(b1, b2)
b = [max(b1(1),b2(1)), min(b1(2),b2(2)), max(b1(3),b2(3)), min(b1(4),b2(4))];
if b(1) > b(2) || b(3) > b(4)   % empty (inconsistent geometry): keep b1
    b = b1;
end
end

function str = num2strExact(v)
% NUM2STREXACT - Shortest decimal string that reads back as exactly v
if ~isfinite(v)
    str = num2str(v);
    return;
end
p0 = min(max(1, floor(log10(abs(v))) + 1), 17);
for prec = p0:17
    str = sprintf(sprintf('%%.%dg', prec), v);
    if str2double(str) == v
        return;
    end
end
end
