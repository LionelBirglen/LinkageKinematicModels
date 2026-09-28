function Q2_leg_mechanism_gui()
% Q2_LEG_MECHANISM_GUI - Interactive GUI for a Planar Q2 Leg Mechanism
%
% OBJECTIVE:
%   Launch a MATLAB GUI to simulate and visualize the direct and inverse
%   kinematics of a complex planar Q2 leg mechanism. Allows switching between
%   direct and inverse modes, selecting one or several solution branches
%   (checkboxes 1-8, one per branch of Q2_leg_mechanism_direct_kinematics,
%   greyed out when that branch does not assemble),
%   and adjusting link and joint parameters. Drawing is delegated to
%   Q2_leg_mechanism_plot.m (same visual style as fourbar_plot.m).
%
% INPUTS:
%   None. All mechanism parameters and mode selections are controlled via
%   GUI controls created within this function.
%
% OUTPUTS:
%   No return values. Visual output is rendered in the GUI figure window.
%
% USAGE EXAMPLE:
%   >> Q2_leg_mechanism_gui
%   Opens the GUI with default link lengths, joint angles, and direct mode.
%
% REQUIRES (on the path):
%   Q2_leg_mechanism_plot.m, Q2_leg_mechanism_direct_kinematics.m,
%   Q2_leg_mechanism_workspace.m
%
% WORKSPACE:
%   "Show workspace of P" draws the region reachable by point P for
%   thetaA and rho within the given ranges (one area per displayed
%   branch, in the branch color). It is computed by
%   Q2_leg_mechanism_workspace on an ntheta x nrho grid, cached, and only
%   recomputed when the parameters, ranges or grid change -- not when
%   the sliders move.
%
% AXES LIMITS:
%   The view is fixed from the geometry (local function
%   computeLimits at the end of this file), so it
%   does not move while the sliders are dragged. It is recomputed when a
%   parameter changes. Once the user zooms or pans (toolbar buttons or
%   scroll), that view is frozen: moving sliders or editing parameters
%   no longer changes the limits until View > Reset View.
%
% ANIMATION:
%   "Animate" moves both inputs simultaneously along sines of the same
%   period, around predefined central positions with predefined
%   amplitudes; the button then reads "Stop". As in
%   fourbar_gui_extended, MATLAB uses a timer (the GUI stays responsive)
%   and Octave a drawnow/pause loop, at about 30 frames per second.
%   The trajectory is defined in the local function animTrajectory at
%   the end of this file (centers, amplitudes, period, phase):
%     Direct mode : thetaA = thetaC + thetaAmp*sin(2*pi*t/T)
%                   rho    = rhoC   + rhoAmp  *sin(2*pi*t/T + phase)
%     Inverse mode: future work
%
% SESSIONS (File menu):
%   Save writes a MAT-file (-v6, readable by MATLAB and Octave) holding
%   a single struct 'session' with the parameters, mode, inputs,
%   displayed solutions, construction-points checkbox, workspace
%   settings (checkbox, ranges, grid) and current axes limits. The
%   workspace itself is not stored; it is recomputed on Open. Open restores all of
%   it. Open also accepts a MAT-file containing just a 'parameters'
%   struct (e.g. saved from the workspace).

% ---------------------------------------------------------------------
% INITIALIZATION
% ---------------------------------------------------------------------

% Environment detection
isOctave = exist('OCTAVE_VERSION','builtin') ~= 0;

% Create main figure window (menus handled as in fourbar_gui_extended)
if isOctave
    f = figure('Name', 'Q2_Leg Mechanism GUI', 'NumberTitle', 'off', ...
        'MenuBar','figure','ToolBar','figure', ...
        'Position', [100 100 1000 600]);
    % Remove Octave's built-in menus so only our custom ones remain
    delete(findall(f,'Type','uimenu'));
else
    f = figure('Name', 'Q2_Leg Mechanism GUI', 'NumberTitle', 'off', ...
        'MenuBar','none','ToolBar','figure', ...
        'Position', [100 100 1000 600]);
end

% File menu: session open/save
mFile = uimenu(f, 'Label', 'File');
uimenu(mFile, 'Label', 'Open',  'Callback', @cbOpen);
uimenu(mFile, 'Label', 'Save',  'Callback', @cbSave);
uimenu(mFile, 'Label', 'Exit',  'Callback', @cbExit, 'Separator', 'on');
% View menu: reset a zoomed/panned view (as in fourbar_gui_extended)
mView = uimenu(f, 'Label', 'View');
uimenu(mView, 'Label', 'Reset View', 'Callback', @resetView);

% --- Default Mechanism Parameters: 17 independent values ---
% (field order = order of the edit boxes, 3 per row)
parameters.AB  = 150;          % Length AB (ground)
parameters.AC  = 90;           % Length AC (input crank)
parameters.BD  = 120;          % Length BD
parameters.CD  = 100;          % Length CD
parameters.FG  = 80;           % Length FG
parameters.CF  = 90;           % Length CF
parameters.BE  = 50;           % Length BE
parameters.GK  = 50;           % Length GK
parameters.GJ  = 80;           % Length GJ
parameters.IF  = 80;           % Length IF
parameters.IJ  = 100;          % Length IJ
parameters.IP  = 50;           % Length IP (point P on body I-J-P)
% Fixed linkage angles (radians)
parameters.DCF = deg2rad(80);  % angle D-C-F
parameters.GFC = deg2rad(110); % angle G-F-C
parameters.DBE = deg2rad(45);  % angle D-B-E
parameters.JGK = deg2rad(30);  % angle J-G-K
parameters.JIP = -0.5235987755982988;  % angle J-I-P (= -30 deg)

% Default input values for direct mode
default.thetaA = deg2rad(-80); % Input joint angle θA
default.rho    = 220;          % Prismatic joint extension ρ

% Per-solution colors (passed to Q2_leg_mechanism_plot). First two match
% fourbar_plot (red, blue).
sol_colors = [1    0    0   ;
              0    0    1   ;
              0    0.6  0   ;
              0.9  0.5  0   ;
              0.55 0    0.75;
              0    0.6  0.6 ;
              0.45 0.45 0.45;
              0.8  0    0.45];

% Mode
mode = 'Direct';               % Current mode: 'Direct' or 'Inverse'

% View state: limits last applied by the GUI ([] = none yet / reset)
last_limits = [];
% True once the user has zoomed/panned (or a saved view was loaded): the
% view is then frozen -- sliders and parameter edits no longer change the
% limits -- until View > Reset View
user_view = false;

% ---------------------------------------------------------------------
% GUI COMPONENTS
% ---------------------------------------------------------------------

% Parameter input fields (3 per row) for all 'parameters' fields
% (angles DCF, GFC, DBE, JGK, JIP are edited in radians, as stored)
param_names = fieldnames(parameters);
param_edits = zeros(1, numel(param_names));   % handles, same order
for k = 1:length(param_names)
    pname = param_names{k};
    row = floor((k-1)/3);
    col = mod((k-1),3);
    xpos = 10 + col * 90;
    ypos = 575 - row * 23;   % rows at 575 ... 460
    uicontrol('Style','text', 'Position', [xpos-15 ypos 40 20], ...
        'String', pname, 'HorizontalAlignment', 'right');
    param_edits(k) = uicontrol('Style','edit', 'Position', [xpos+30 ypos+3 35 20], ...
        'String', num2strExact(parameters.(pname)), ...
        'Callback', @(src,~) updateParameter(pname, src));
end

% (No mode button: the inverse kinematics is not ready yet, so the GUI
% stays in Direct mode. The inverse-mode code is kept for later; to
% re-enable it, add back a button calling switchMode. Reset View is in
% the View menu.)
mode_button = [];

% Direct mode inputs: θA and ρ with sliders and edit fields
lbl_thetaA = uicontrol('Style','text','Position',[10 414 120 20], ...
    'String','Input angle θA (deg)');
input_thetaA = uicontrol('Style','edit', ...
    'String', num2str(rad2deg(default.thetaA)), ...
    'Position',[140 414 100 20], 'Callback', @updatePlot);
thetaA_slider = uicontrol('Style','slider', 'Min', -180, 'Max', 180, ...
    'Value', rad2deg(default.thetaA), 'Position',[10 392 230 20], 'Callback', @syncThetaA);

lbl_rho = uicontrol('Style','text','Position',[10 368 120 20], ...
    'String','Prismatic length ρ (mm)');
input_rho = uicontrol('Style','edit', ...
    'String', num2str(default.rho), 'Position',[140 368 100 20], 'Callback', @updatePlot);
rho_slider = uicontrol('Style','slider', 'Min', 50, 'Max', 400, ...
    'Value', default.rho, 'Position',[10 346 230 20], 'Callback', @syncRho);

% Inverse mode inputs: target X_i and Y_i (hidden initially). They use
% the same place as the direct inputs, since only one set is visible.
lbl_xI = uicontrol('Style','text','Position',[10 414 120 20], ...
    'String','Target Xi (mm)','Visible','off');
input_xI = uicontrol('Style','edit','String','0','Position',[140 414 100 20], ...
    'Callback',@updatePlot,'Visible','off');
xI_slider = uicontrol('Style','slider','Min',-300,'Max',300,'Value',0, ...
    'Position',[10 392 230 20],'Callback',@syncXI,'Visible','off');

lbl_yI = uicontrol('Style','text','Position',[10 368 120 20], ...
    'String','Target Yi (mm)','Visible','off');
input_yI = uicontrol('Style','edit','String','0','Position',[140 368 100 20], ...
    'Callback',@updatePlot,'Visible','off');
yI_slider = uicontrol('Style','slider','Min',-300,'Max',300,'Value',0, ...
    'Position',[10 346 230 20],'Callback',@syncYI,'Visible','off');

% Workspace of P: checkbox, input ranges and grid size
ws_checkbox = uicontrol('Style','checkbox','Position',[10 322 285 18], ...
    'String','Show workspace of P', 'Value',0, 'Callback',@updatePlot);
uicontrol('Style','text','Position',[10 298 110 18], ...
    'String','θA range (deg)', 'HorizontalAlignment','left');
ws_tmin = uicontrol('Style','edit','String','-180','Position',[120 300 55 20], ...
    'Callback',@updatePlot);
ws_tmax = uicontrol('Style','edit','String','180','Position',[180 300 55 20], ...
    'Callback',@updatePlot);
uicontrol('Style','text','Position',[10 276 110 18], ...
    'String','ρ range (mm)', 'HorizontalAlignment','left');
ws_rmin = uicontrol('Style','edit','String','50','Position',[120 278 55 20], ...
    'Callback',@updatePlot);
ws_rmax = uicontrol('Style','edit','String','400','Position',[180 278 55 20], ...
    'Callback',@updatePlot);
uicontrol('Style','text','Position',[10 254 110 18], ...
    'String','Grid nθ × nρ', 'HorizontalAlignment','left');
ws_nt = uicontrol('Style','edit','String','91','Position',[120 256 55 20], ...
    'Callback',@updatePlot);
ws_nr = uicontrol('Style','edit','String','51','Position',[180 256 55 20], ...
    'Callback',@updatePlot);
ws_status = uicontrol('Style','text','Position',[10 226 285 28], ...
    'String','', 'FontSize', 8, 'HorizontalAlignment','left');

% Workspace cache: recomputed only when its inputs change
ws_data = [];
ws_key  = [];

% Animate button (toggles to "Stop" while running), above the solution
% selector, same position and width as the sliders
animate_btn = uicontrol('Style','pushbutton', 'String','Animate', ...
    'Position',[10 203 230 20], 'Callback', @toggleAnimation);

% Display solutions: one checkbox per branch (8 branches, 2 rows of 4),
% as in the fourbar and Stephenson III GUIs
nSols = 8;
solsPanel = uipanel('Title','Display solutions:','FontSize',9, ...
    'Units','pixels','Position',[10 150 285 50]);
sols_checkbox = zeros(1, nSols);
for k = 1:nSols
    row = floor((k-1)/4);  colI = mod(k-1,4);
    sols_checkbox(k) = uicontrol('Parent',solsPanel,'Style','checkbox', ...
        'Units','normalized','Position',[0.06+0.24*colI 0.48-0.46*row 0.2 0.44], ...
        'String',num2str(k),'Value',0,'Callback',@updatePlot);
end
set(sols_checkbox(2),'Value',1);   % default: solution 2, as before
solution_label = uicontrol('Style','text','Position',[10 2 285 146], ...
    'String','', 'FontSize', 8, 'HorizontalAlignment', 'left');

% Checkbox: show line intersections M (AC x BD) and N (IF x GJ)
% with their construction lines (left panel, under the parameters)
constr_checkbox = uicontrol('Style','checkbox','Position',[10 438 285 18], ...
    'String','Show M (AC∩BD), N (IF∩GJ) and lines', ...
    'Value',0,'Callback',@updatePlot);

% Axes for plotting the mechanism
ax = axes('Units','pixels','Position',[300 100 650 450]);
axis(ax,'equal'); grid(ax,'on');

% Mark the view as user-defined as soon as a zoom or pan ends (MATLAB).
% Octave lacks these mode objects; there the limit comparison in
% updatePlot does the detection instead.
try
    set(zoom(f), 'ActionPostCallback', @(~,~) markUserView());
    set(pan(f),  'ActionPostCallback', @(~,~) markUserView());
catch
end

% Animation state
anim_running = false;   % true while animating
anim_timer   = [];      % MATLAB timer object
anim_t0      = [];      % tic reference of the animation start
anim_x0      = [];      % inputs at the start ([thetaA_deg rho] or [xI yI])

% Stop the animation (and delete the timer) when the window is closed
set(f, 'DeleteFcn', @(~,~) stopAnimation());

% Initial plot draw
updatePlot();

% ---------------------------------------------------------------------
% Nested callback functions and helpers
% ---------------------------------------------------------------------

    function switchMode(~, ~)
        % SWITCHMODE - Toggle between direct and inverse modes
        stopAnimation();
        if strcmp(mode, 'Direct')
            applyMode('Inverse');
        else
            applyMode('Direct');
        end
        updatePlot();
    end

    function applyMode(newMode)
        % APPLYMODE - Set mode and show/hide the matching controls
        % (set/get used instead of dot notation for Octave compatibility)
        dirH = [lbl_thetaA input_thetaA thetaA_slider lbl_rho input_rho rho_slider];
        invH = [lbl_xI input_xI xI_slider lbl_yI input_yI yI_slider];
        if strcmp(newMode, 'Inverse')
            mode = 'Inverse';
            if ~isempty(mode_button), set(mode_button, 'String', 'Switch to Direct'); end
            set(dirH, 'Visible', 'off');
            set(invH, 'Visible', 'on');
        else
            mode = 'Direct';
            if ~isempty(mode_button), set(mode_button, 'String', 'Switch to Inverse'); end
            set(dirH, 'Visible', 'on');
            set(invH, 'Visible', 'off');
        end
    end

    function setSlider(h, v)
        % SETSLIDER - Set a slider value clamped to its range (an
        % out-of-range Value makes MATLAB hide the slider with a warning)
        if isfinite(v)
            set(h, 'Value', min(max(v, get(h,'Min')), get(h,'Max')));
        end
    end

    function syncThetaA(src, ~)
        set(input_thetaA, 'String', num2str(get(src, 'Value')));
        updatePlot();
    end

    function syncRho(src, ~)
        set(input_rho, 'String', num2str(get(src, 'Value')));
        updatePlot();
    end

    function syncXI(src, ~)
        set(input_xI, 'String', num2str(get(src, 'Value')));
        updatePlot();
    end

    function syncYI(src, ~)
        set(input_yI, 'String', num2str(get(src, 'Value')));
        updatePlot();
    end

    function W = getWorkspace()
        % GETWORKSPACE - Cached workspace of P ([] if not shown)
        W = [];
        if get(ws_checkbox,'Value') ~= 1
            set(ws_status, 'String', '');
            return;
        end
        tl = [str2double(get(ws_tmin,'String')) str2double(get(ws_tmax,'String'))];
        rl = [str2double(get(ws_rmin,'String')) str2double(get(ws_rmax,'String'))];
        ng = [str2double(get(ws_nt,'String'))   str2double(get(ws_nr,'String'))];
        if any(~isfinite([tl rl ng])) || tl(1) >= tl(2) || rl(1) >= rl(2) || any(ng < 2)
            set(ws_status, 'String', ...
                'Workspace: invalid ranges or grid (need min < max, n >= 2).');
            return;
        end
        key = struct('p', parameters, 'tl', tl, 'rl', rl, 'ng', round(ng));
        if isequal(key, ws_key) && ~isempty(ws_data)
            W = ws_data;
            return;
        end
        set(ws_status, 'String', sprintf('Computing workspace (%d x %d)...', ...
            round(ng(1)), round(ng(2))));
        set(f, 'Pointer', 'watch');
        drawnow();
        try
            W = Q2_leg_mechanism_workspace(parameters, deg2rad(tl), rl, round(ng));
            ws_data = W;
            ws_key  = key;
            set(ws_status, 'String', sprintf( ...
                'Workspace: %d x %d grid, %.1f s\nvalid nodes/branch: %s', ...
                W.n(1), W.n(2), W.time, sprintf('%d ', W.nValid)));
        catch ME
            W = [];
            ws_data = [];
            ws_key  = [];
            set(ws_status, 'String', ['Workspace error: ' ME.message]);
        end
        set(f, 'Pointer', 'arrow');
    end

    function markUserView()
        % MARKUSERVIEW - Freeze the current (zoomed/panned) limits
        user_view = true;
    end

    function resetView(~, ~)
        % RESETVIEW - Drop any user zoom/pan and return to fixed limits
        user_view   = false;
        last_limits = [];
        updatePlot();
    end

    function updatePlot(~, ~)
        % UPDATEPLOT - Compute kinematics and draw mechanism
        try
            if strcmp(mode, 'Direct')
                thetaA_deg = str2double(get(input_thetaA,'String'));
                rho = str2double(get(input_rho,'String'));
                setSlider(thetaA_slider, thetaA_deg);
                setSlider(rho_slider, rho);
                plot_mode = 'direct';
                plot_in   = [deg2rad(thetaA_deg); rho];
            else
                xI = str2double(get(input_xI,'String'));
                yI = str2double(get(input_yI,'String'));
                setSlider(xI_slider, xI);
                setSlider(yI_slider, yI);
                plot_mode = 'inverse';
                plot_in   = [xI; yI];
            end

            % Axis limits: fixed from geometry unless the user zoomed/panned.
            % A zoom/pan is detected either by the zoom/pan callbacks or
            % because the current limits differ from the last ones set by
            % the GUI; from then on the user's view is kept (user_view
            % stays true) until View > Reset View.
            cur = [get(ax,'XLim') get(ax,'YLim')];
            if ~user_view && ~isempty(last_limits)
                tol = 1e-6 * max(abs(last_limits(2)-last_limits(1)), 1);
                user_view = any(abs(cur - last_limits) > tol);
            end
            if user_view
                lims = cur;                       % keep the user's view
            else
                lims = computeLimits(parameters); % fixed, geometry-based
            end

            % Solutions ticked by the user (none ticked: draw no linkage)
            sel = find(arrayfun(@(h) get(h,'Value') == 1, sols_checkbox));

            % Draw with the shared plot function
            opts = struct();
            opts.limits     = lims;
            opts.ax         = ax;
            opts.clearAxes  = true;
            opts.showLabels = true;
            opts.colors     = sol_colors;
            opts.solutions  = sel;
            if isempty(sel), opts.solutions = 0; end
            opts.showConstruction = get(constr_checkbox,'Value') == 1;
            opts.workspace  = getWorkspace();
            opts.workspaceStyle = 'both';   % areas, plus dots only for the
                                            % isolated positions outside
                                            % them (thin regions)
            [~, solutions, kin, constr] = Q2_leg_mechanism_plot(parameters, plot_mode, plot_in, opts);

            if strcmp(mode, 'Inverse') && kin.ik_error > 0.01
                warning('Inverse kinematics error: %.4f mm', kin.ik_error);
            end

            % Remember the limits actually in effect (read back, since
            % axis equal may adjust them, e.g. in Octave)
            drawnow();
            last_limits = [get(ax,'XLim') get(ax,'YLim')];

            % Enable only the checkboxes of branches that assemble
            valid = arrayfun(@(q) isfield(q,'valid') && q.valid, solutions);
            for k = 1:nSols
                if k <= numel(solutions) && valid(k)
                    set(sols_checkbox(k), 'Enable', 'on');
                else
                    set(sols_checkbox(k), 'Enable', 'off');
                end
            end
            if isempty(solutions) || ~any(valid)
                error('Unreachable configuration (no branch assembles).');
            end

            % Final plot adjustments
            title(ax, sprintf('Q2 Leg Mechanism - %s Mode', mode));
            xlabel(ax, 'X [mm]'); ylabel(ax, 'Y [mm]');

            % Info: actuated inputs, then I, P and angle of I->J per branch
            if strcmp(mode, 'Direct')
                txt = sprintf('θA = %.1f°, ρ = %.1f mm\n', ...
                    rad2deg(kin.thetaA), kin.rho);
            else
                txt = sprintf('IK: θA = %.1f°, ρ = %.1f mm (err %.3f mm)\n', ...
                    rad2deg(kin.thetaA), kin.rho, kin.ik_error);
            end
            mnTxt = {};   % |MN| of each displayed solution, listed compactly
            for ii = sel(:).'
                if ii > numel(solutions), continue; end
                s = solutions(ii);
                if ~valid(ii)
                    txt = sprintf('%sSol %d: invalid\n', txt, ii);
                else
                    Ip = s.Positions.I;
                    txt = sprintf('%sSol %d: I=(%.1f, %.1f)', ...
                        txt, ii, Ip(1), Ip(2));
                    if all(isfinite(s.P))
                        txt = sprintf('%s P=(%.1f, %.1f)', txt, s.P(1), s.P(2));
                    end
                    txt = sprintf('%s Φ=%.1f°\n', txt, rad2deg(s.phi));
                    % Distance MN, only when those points are displayed
                    if get(constr_checkbox,'Value') == 1
                        if isfinite(constr(ii).dMN)
                            mnTxt{end+1} = sprintf('%d: %.2f', ii, constr(ii).dMN); %#ok<AGROW>
                        else
                            mnTxt{end+1} = sprintf('%d: ∞', ii);                     %#ok<AGROW>
                        end
                    end
                end
            end
            % |MN| values: 4 per line, so 8 solutions take only 2 lines
            for q = 1:4:numel(mnTxt)
                if q == 1, lead = '|MN| (mm): '; else, lead = '                   '; end
                txt = sprintf('%s%s%s\n', txt, lead, ...
                    strjoin(mnTxt(q:min(q+3,numel(mnTxt))), '  '));
            end
            set(solution_label, 'String', txt);

        catch ME
            % Handle errors (e.g. unreachable)
            cla(ax);
            text(0.5, 0.5, sprintf('Error: %s', ME.message), ...
                'Parent', ax, 'Units', 'normalized', 'FontSize', 12, ...
                'Color', 'r', 'HorizontalAlignment', 'center');
            set(solution_label, 'String', 'Invalid configuration.');
        end
    end

    % -----------------------------------------------------------------
    % File callbacks (session MAT-files, as in fourbar_gui_extended)
    % -----------------------------------------------------------------
    function cbOpen(~, ~)
        % CBOPEN - Load a session MAT-file and restore the GUI state
        stopAnimation();
        warning('off','all');
        [fname, fpath] = uigetfile({'*.mat','MAT-file (*.mat)'}, 'Open Session File');
        warning('on','all');
        if isequal(fname,0), return; end
        full = fullfile(char(fpath), char(fname));
        try
            warning('off','all');
            S = load(full, '-mat');
            warning('on','all');
        catch ME
            warning('on','all');
            errordlg(sprintf('Could not read file:\n%s', ME.message), 'Open Error');
            return;
        end

        if isfield(S,'session') && isstruct(S.session) && ...
                isfield(S.session,'parameters')
            session = S.session;
        elseif isfield(S,'parameters') && isstruct(S.parameters)
            % Bare parameter struct: keep current inputs/mode/view
            session = struct('parameters', S.parameters);
        else
            errordlg(['Unrecognised session file format ' ...
                '(not a Q2 leg mechanism session).'], 'Open Error');
            return;
        end

        % --- Parameters: only known fields; missing ones keep their
        % current value. Files from the previous parameter set are
        % converted: FCD -> DCF, FI -> IF (same values); GD is dropped
        % (redundant: follows from CD, CF, FG, DCF and GFC).
        legacy = {'FCD','DCF'; 'FI','IF'};
        for ii = 1:size(legacy,1)
            if isfield(session.parameters, legacy{ii,1}) && ...
                    ~isfield(session.parameters, legacy{ii,2})
                session.parameters.(legacy{ii,2}) = ...
                    session.parameters.(legacy{ii,1});
            end
        end
        for ii = 1:numel(param_names)
            nm = param_names{ii};
            if isfield(session.parameters, nm)
                v = session.parameters.(nm);
                if isnumeric(v) && isscalar(v) && isfinite(v)
                    parameters.(nm) = v;
                    set(param_edits(ii), 'String', num2strExact(v));
                end
            end
        end

        % --- Inputs (degrees / mm, as displayed)
        if isfield(session,'thetaA'), set(input_thetaA,'String',num2strExact(session.thetaA)); end
        if isfield(session,'rho'),    set(input_rho,   'String',num2strExact(session.rho));    end
        if isfield(session,'xI'),     set(input_xI,    'String',num2strExact(session.xI));     end
        if isfield(session,'yI'),     set(input_yI,    'String',num2strExact(session.yI));     end

        % --- Mode: always Direct for now (inverse mode is disabled; a
        % session saved in inverse mode opens in direct mode)
        applyMode('Direct');

        % --- Construction points checkbox
        if isfield(session,'showConstruction')
            set(constr_checkbox, 'Value', double(session.showConstruction ~= 0));
        end

        % --- Workspace settings
        wsFields = {'wsThetaMin',ws_tmin; 'wsThetaMax',ws_tmax; ...
                    'wsRhoMin',ws_rmin;   'wsRhoMax',ws_rmax; ...
                    'wsNTheta',ws_nt;     'wsNRho',ws_nr};
        for ii = 1:size(wsFields,1)
            if isfield(session, wsFields{ii,1})
                set(wsFields{ii,2}, 'String', num2strExact(session.(wsFields{ii,1})));
            end
        end
        if isfield(session,'showWorkspace')
            set(ws_checkbox, 'Value', double(session.showWorkspace ~= 0));
        end

        % --- Displayed solutions: 1x8 checkbox states (as in the other
        % GUIs); files saved with the former listbox hold a list of
        % indices instead, converted here
        if isfield(session,'solsVisible') && ~isempty(session.solsVisible)
            sv = session.solsVisible(:).';
            if numel(sv) == nSols && all(sv == 0 | sv == 1)
                flags = sv ~= 0;
            else
                idx = round(sv);  idx = idx(idx >= 1 & idx <= nSols);
                flags = false(1, nSols);  flags(idx) = true;
            end
            for ii = 1:nSols
                set(sols_checkbox(ii), 'Value', double(flags(ii)));
            end
        end

        % --- View: restore saved limits; if they differ from the
        % geometry-based limits they are frozen as a user view
        user_view   = false;
        last_limits = [];
        if isfield(session,'axesXLim') && isfield(session,'axesYLim') && ...
                numel(session.axesXLim)==2 && numel(session.axesYLim)==2
            geo = computeLimits(parameters);
            sav = [session.axesXLim(:).' session.axesYLim(:).'];
            tol = 1e-6 * max(abs(geo(2)-geo(1)), 1);
            if any(abs(sav - geo) > tol)
                xlim(ax, session.axesXLim);
                ylim(ax, session.axesYLim);
                user_view = true;
            end
        end

        updatePlot();
    end

    function cbSave(~, ~)
        % CBSAVE - Save the current GUI state to a session MAT-file
        warning('off','all');
        [fname, fpath] = uiputfile({'*.mat','MAT-file (*.mat)'}, ...
            'Save Session As', 'Q2_leg_session.mat');
        warning('on','all');
        if isequal(fname,0), return; end
        target = fullfile(char(fpath), char(fname));

        session = struct();
        session.name        = 'Q2 Leg Mechanism';
        session.version     = 1;
        session.parameters  = parameters;      % angles in rad, as stored
        session.modeStr     = mode;
        session.thetaA      = str2double(get(input_thetaA,'String'));  % deg
        session.rho         = str2double(get(input_rho,'String'));
        session.xI          = str2double(get(input_xI,'String'));
        session.yI          = str2double(get(input_yI,'String'));
        session.solsVisible = arrayfun(@(h) get(h,'Value'), sols_checkbox);  % 1x8 flags
        session.showConstruction = get(constr_checkbox,'Value');
        session.showWorkspace = get(ws_checkbox,'Value');
        session.wsThetaMin  = str2double(get(ws_tmin,'String'));   % deg
        session.wsThetaMax  = str2double(get(ws_tmax,'String'));   % deg
        session.wsRhoMin    = str2double(get(ws_rmin,'String'));
        session.wsRhoMax    = str2double(get(ws_rmax,'String'));
        session.wsNTheta    = str2double(get(ws_nt,'String'));
        session.wsNRho      = str2double(get(ws_nr,'String'));
        session.axesXLim    = get(ax,'XLim');
        session.axesYLim    = get(ax,'YLim');
        try
            save(target, 'session', '-mat', '-v6');
        catch ME
            errordlg(sprintf('Could not save file:\n%s', ME.message), 'Save Error');
        end
    end

    function cbExit(~, ~)
        stopAnimation();
        choice = questdlg('Are you sure you want to exit?', 'Exit', ...
            'Yes','No','No');
        if strcmp(choice,'Yes')
            close(f);
        end
    end

    % -----------------------------------------------------------------
    % Animation (same scheme as fourbar_gui_extended)
    % -----------------------------------------------------------------
    function toggleAnimation(~, ~)
        % TOGGLEANIMATION - Start or stop the animation
        if anim_running
            stopAnimation();
            return;
        end
        % Start from the current inputs
        if strcmp(mode, 'Direct')
            anim_x0 = [str2double(get(input_thetaA,'String')) ...
                       str2double(get(input_rho,'String'))];
        else
            anim_x0 = [str2double(get(input_xI,'String')) ...
                       str2double(get(input_yI,'String'))];
        end
        if any(~isfinite(anim_x0))
            return;
        end
        anim_t0      = tic;
        anim_running = true;
        set(animate_btn, 'String', 'Stop');
        if ~isOctave
            % MATLAB: timer-based, non-blocking
            anim_timer = timer('ExecutionMode','fixedRate', ...
                'Period', 0.033, 'BusyMode', 'drop', ...
                'TimerFcn', @(~,~) animateStep());
            start(anim_timer);
        else
            % Octave: no timers, loop while processing GUI events
            while anim_running && ishghandle(f)
                animateStep();
                drawnow();          % lets the Stop button be pressed
                pause(0.033);       % ~30 fps
            end
        end
    end

    function stopAnimation()
        % STOPANIMATION - Stop the animation and clean up the timer
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
            set(animate_btn, 'String', 'Animate');
        end
    end

    function animateStep()
        % ANIMATESTEP - Set the inputs at the current time and redraw
        if ~anim_running || ~ishghandle(f)
            stopAnimation();
            return;
        end
        x = animTrajectory(toc(anim_t0), mode, anim_x0);
        if strcmp(mode, 'Direct')
            set(input_thetaA, 'String', num2str(x(1)));
            set(input_rho,    'String', num2str(x(2)));
        else
            set(input_xI, 'String', num2str(x(1)));
            set(input_yI, 'String', num2str(x(2)));
        end
        updatePlot();
    end

    function updateParameter(pname, src)
        % UPDATEPARAMETER - Update mechanism parameter and redraw
        val = str2double(get(src,'String'));
        if ~isnan(val)
            parameters.(pname) = val;
            updatePlot();
        end
    end
end


% =====================================================================
%  Local functions (outside the nested scope: no shared variables)
% =====================================================================
function lims = computeLimits(p, margin)
% COMPUTELIMITS - Fixed axis limits enclosing every reachable pose
%
% Returns [xmin xmax ymin ymax] such that every joint (and P) stays
% inside the box for ANY thetaA, rho and assembly
% branch. The limits depend on the geometry only, so they stay constant
% while the inputs are dragged with the sliders.
%
% The box is a rigorous bound built from the triangle inequality: each
% point is bounded by discs around the ground pivots (through the chain
% of rigid bodies that holds it), and bounds from several chains are
% intersected to keep it reasonably tight.
%
% MARGIN : relative margin added on each side (default 0.05).

if nargin < 2, margin = 0.05; end

A = [0; 0];
B = [p.AB; 0];

% Branch-independent distances in the quaternary body C-D-G-F, built
% with the same formulas as Q2_leg_mechanism_direct_kinematics (C at the origin,
% D on +x).
Emat = [0 -1; 1 0];
Cl = [0; 0];  Dl = [p.CD; 0];
xC = [1; 0];  yC = Emat * xC;
F  = Cl + p.CF * (cos(p.DCF) * xC - sin(p.DCF) * yC);
xF = (Cl - F) / norm(Cl - F);  yF = Emat * xF;
G  = F + p.FG * (cos(p.GFC) * xF - sin(p.GFC) * yF);
rCF = p.CF;           rDF = norm(F - Dl);
rCG = norm(G - Cl);   rDG = norm(G - Dl);

% Bounding boxes [xmin xmax ymin ymax] of each point
bC = limDiscBox(A, p.AC);
bD = limIntersect(limDiscBox(A, p.AC + p.CD), limDiscBox(B, p.BD));
bF = limIntersect(limDiscBox(A, p.AC + rCF),  limDiscBox(B, p.BD + rDF));
bG = limIntersect(limDiscBox(A, p.AC + rCG),  limDiscBox(B, p.BD + rDG));
bE = limDiscBox(B, p.BE);
bK = limGrow(bG, p.GK);
bJ = limGrow(bG, p.GJ);
bI = limIntersect(limGrow(bF, p.IF), limGrow(bJ, p.IJ));
boxes = [bC; bD; bE; bF; bG; bK; bJ; bI; ...
         [A(1) A(1) A(2) A(2)]; [B(1) B(1) B(2) B(2)]; ...
         limGrow(bI, abs(p.IP))];

lims = [min(boxes(:,1)) max(boxes(:,2)) min(boxes(:,3)) max(boxes(:,4))];
m = margin * max(lims(2)-lims(1), lims(4)-lims(3));
lims = lims + [-m m -m m];
end

function b = limDiscBox(c, r)
% Bounding box of the disc of center c and radius r
b = [c(1)-r, c(1)+r, c(2)-r, c(2)+r];
end

function b = limGrow(b, r)
% Box grown by r on every side (box of all points within r of box b)
b = b + [-r r -r r];
end

function b = limIntersect(b1, b2)
% Intersection of two boxes (falls back to b1 if empty, i.e. the
% geometry is inconsistent)
b = [max(b1(1),b2(1)), min(b1(2),b2(2)), max(b1(3),b2(3)), min(b1(4),b2(4))];
if b(1) > b(2) || b(3) > b(4)
    b = b1;
end
end

function str = num2strExact(v)
% NUM2STREXACT - Shortest decimal string that reads back as exactly v
% (unlike num2str, which rounds to about 4 decimals). E.g. 150 -> '150',
% 0.35 -> '0.35', deg2rad(80) -> '1.3962634015954636'.
if ~isfinite(v)
    str = num2str(v);
    return;
end
% start with enough significant digits for the integer part, so that
% e.g. 150 gives '150' rather than '1.5e+02'
p0 = min(max(1, floor(log10(abs(v))) + 1), 17);
for prec = p0:17
    str = sprintf(sprintf('%%.%dg', prec), v);
    if str2double(str) == v
        return;
    end
end
end


function x = animTrajectory(t, mode, x0)
% ANIMTRAJECTORY - Predefined animation trajectory (edit here to change)
%
% Both inputs move simultaneously along sines of the same period T:
%   Direct : thetaA = thetaC + thetaAmp*sin(2*pi*t/T)          (deg)
%            rho    = rhoC   + rhoAmp  *sin(2*pi*t/T + phase)  (mm)
%   Inverse: xI     = x0(1)  + xAmp    *sin(2*pi*t/T)          (mm)
%            yI     = x0(2)  + yAmp    *sin(2*pi*t/T + phase)  (mm)
%            (centered on the target for I when Animate is pressed)
% phase = 0 moves the inputs in step (a straight segment in the input
% plane); phase = 90 makes them trace an ellipse.
%
% t    : time since the animation started (s)
% mode : 'Direct' or 'Inverse'
% x0   : inputs when Animate was pressed: [thetaA_deg rho] or [xI yI]
% x    : inputs at time t, same units as x0

% --- predefined trajectory --------------------------------------------
thetaC   = -80;   % deg: central position of thetaA
thetaAmp =  40;   % deg: amplitude of thetaA
rhoC     = 220;   % mm : central position of rho
rhoAmp   =  20;   % mm : amplitude of rho
xAmp     =  20;   % mm : amplitude of xI (inverse mode)
yAmp     =  20;   % mm : amplitude of yI (inverse mode)
T        =   6;   % s  : period, common to both inputs
phase    =   0;   % deg: phase of the second input relative to the first
% ----------------------------------------------------------------------

w = 2*pi*t/T;
if strcmp(mode, 'Direct')
    x = [thetaC + thetaAmp*sin(w), ...
         rhoC   + rhoAmp  *sin(w + deg2rad(phase))];
else
    x = [x0(1) + xAmp*sin(w), ...
         x0(2) + yAmp*sin(w + deg2rad(phase))];
end
end
