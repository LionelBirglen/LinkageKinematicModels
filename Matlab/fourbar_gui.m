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
% Last Update: 2025/05/15
% Contact: lionel.birglen@polymtl.ca
% 
% Code provided under GNU Affero General Public License v3.0


% ---------------------------------------------------------------------
% INITIALIZATION
% ---------------------------------------------------------------------

% Create main figure window
f = figure('Name', 'Four-Bar Linkage','Position', [100 100 850 600]);

% Default linkage parameters
default = struct('a', 81, ...  % Output link length
    'b', 88, ...               % Coupler link length
    'c', 92, ...               % Input crank length
    'd', 151, ...              % Ground link length
    'theta', deg2rad(106),...  % Initial input angle (rad)
    'config', -1);             % Configuration flag: +1 open, -1 crossed

% State variables
configState    = default.config;   % Current linkage configuration
modeState      = 1;                % Kinematics mode: 1=direct, 2=inverse
showAlternate  = true;             % Flag for alternate solution display
animating      = false;            % Animation on/off flag

% ---------------------------------------------------------------------
% GUI COMPONENTS
% ---------------------------------------------------------------------

% Input link length c
uicontrol('Style','text','Position',[10 550 120 20],'String','Input link c');
input_c = uicontrol('Style','edit','String',num2str(default.c),...
    'Position',[130 550 150 20],'Callback',@updatePlot);

% Coupler link length b
uicontrol('Style','text','Position',[10 520 120 20],'String','Coupler link b');
input_b = uicontrol('Style','edit','String',num2str(default.b),...
    'Position',[130 520 150 20],'Callback',@updatePlot);

% Output link length a
uicontrol('Style','text','Position',[10 490 120 20],'String','Output link a');
input_a = uicontrol('Style','edit','String',num2str(default.a),...
    'Position',[130 490 150 20],'Callback',@updatePlot);

% Ground link length d
uicontrol('Style','text','Position',[10 460 120 20],'String','Ground link d');
input_d = uicontrol('Style','edit','String',num2str(default.d),...
    'Position',[130 460 150 20],'Callback',@updatePlot);

% Label for input/output angle
angle_label = uicontrol('Style','text','Position',[10 430 120 20],...
    'String','Input angle θ (deg)');

% Angle edit box
input_angle = uicontrol('Style','edit','String',num2str(rad2deg(default.theta)),...
    'Position',[130 430 150 20],'Callback',@syncSlider);

% Angle slider control
angle_slider = uicontrol('Style','slider','Min',0,'Max',360,'Value',rad2deg(default.theta),...
    'Position',[10 400 270 20],'Callback',@syncEdit);

% Toggle button for configuration (open vs crossed)
toggle_btn = uicontrol('Style','togglebutton','String','Config: Crossed',...
    'Position',[10 360 270 30],'Callback',@toggleConfig);

% Mode selection popup (Direct/Inverse)
uicontrol('Style','text','Position',[10 320 120 20],'String','Mode');
mode_menu = uicontrol('Style','popupmenu', 'Position', [130 320 150 20], ...
    'String', {'Direct', 'Inverse'}, 'Callback', @modeChanged);

% Checkbox to show/hide alternate configuration
show_alt_checkbox = uicontrol('Style', 'checkbox', 'Position', [10 290 270 20], ...
    'String', 'Show alternate config', 'Value', 1, 'Callback', @toggleAlt);

% Text area to display computed angles
angle_text = uicontrol('Style','text','Position',[10 250 300 40],...
    'FontSize',12,'HorizontalAlignment','left');

% Animate button to run continuous motion
animate_btn = uicontrol('Style','pushbutton','String','Animate',...
    'Position',[10 200 270 30],'Callback',@toggleAnimation);

% Axes for drawing linkage
ax = axes('Units','pixels','Position',[320 100 500 450]);
axis equal; grid on;

% Initial plot render
updatePlot();

% ---------------------------------------------------------------------
% Nested callback functions and helpers
% ---------------------------------------------------------------------

    function toggleConfig(src, ~)
        % TOGGLECONFIG - Flip linkage configuration (open vs crossed)
        configState = -configState;
        set(src, 'String', ['Config: ', ternary(configState==1, 'Open', 'Crossed')]);
        updatePlot();
    end

    function toggleAlt(src, ~)
        % TOGGLEALT - Show or hide the alternate kinematic solution
        showAlternate = logical(get(src, 'Value'));
        updatePlot();
    end

    function modeChanged(~, ~)
        % MODECHANGED - Handle switch between direct & inverse kinematics
        oldMode = modeState;
        modeState = get(mode_menu, 'Value');
        % Update angle label text based on mode
        set(angle_label, 'String', ternary(modeState==1, 'Input angle θ (deg)', 'Output angle α (deg)'));

        % Read current parameters
        c = str2double(get(input_c,'String'));
        b = str2double(get(input_b,'String'));
        a = str2double(get(input_a,'String'));
        d = str2double(get(input_d,'String'));
        val_deg = str2double(get(input_angle,'String'));
        angle_rad = deg2rad(val_deg);

        % Attempt to preserve previous configuration across mode change
        try
            if oldMode == 1  % Direct → Inverse
                [~, alpha, ~] = fourbar_direct_kinematics(c, b, a, d, angle_rad, configState);
                new_val = rad2deg(alpha);
            else            % Inverse → Direct
                [theta, ~, ~] = fourbar_inverse_kinematics(c, b, a, d, angle_rad, configState);
                new_val = rad2deg(theta);
            end
            set(input_angle, 'String', num2str(new_val));
            set(angle_slider, 'Value', new_val);
        catch
            warning('Could not preserve linkage when switching modes.');
        end

        % Redraw with new mode
        updatePlot();
    end

    function syncSlider(~, ~)
        % SYNCSLIDER - Clamp edit box value and sync slider handle
        val = str2double(get(input_angle,'String'));
        % Ensure within slider bounds
        val = max(min(val, get(angle_slider,'Max')), get(angle_slider,'Min'));
        set(angle_slider, 'Value', val);
        updatePlot();
    end

    function syncEdit(src, ~)
        % SYNCEDIT - Update edit box when slider moves
        val = get(src, 'Value');
        set(input_angle, 'String', num2str(val));
        updatePlot();
    end

    function toggleAnimation(~,~)
        % TOGGLEANIMATION - Toggle animation on/off
        animating = ~animating;
        if animating
            set(animate_btn, 'String', 'Stop');
            runAnimation();
        else
            set(animate_btn, 'String', 'Animate');
        end
    end

    function runAnimation()
        % RUNANIMATION - Continuously animate input/output angle
        c = str2double(get(input_c,'String'));
        b = str2double(get(input_b,'String'));
        a = str2double(get(input_a,'String'));
        d = str2double(get(input_d,'String'));

        range = computeFeasibleAngleRange(a, b, c, d, configState, modeState);
        if isempty(range)
            return;  % No valid motion if empty range
        end

        angles = linspace(range(1), range(2), 200);  % Smooth animation steps

        while animating && ishandle(f)
            for val = angles
                if ~animating || ~ishandle(f)
                    break;
                end
                set(input_angle, 'String', num2str(val));
                set(angle_slider, 'Value', val);
                updatePlot();
                drawnow;
                pause(0.02);
            end
        end
    end

    function updatePlot(~,~)
        % UPDATEPLOT - Compute and draw linkage based on inputs
        c = str2double(get(input_c,'String'));
        b = str2double(get(input_b,'String'));
        a = str2double(get(input_a,'String'));
        d = str2double(get(input_d,'String'));
        val_deg = str2double(get(input_angle,'String'));
        angle_rad = deg2rad(val_deg);

        % Determine valid range for motion (optional slider bounds)
        range = computeFeasibleAngleRange(a, b, c, d, configState, modeState);
        if isempty(range)
            set(angle_text, 'String', 'No valid angles.');
            cla(ax); return;
        end
        % Optionally set slider bounds here if desired
        set(angle_slider, 'SliderStep', [1/360 10/360]);

        try
            % Define ground pivot points
            O = [0, 0];              % Origin pivot
            C = [d, 0];              % Ground link end

            % Compute kinematics based on mode
            if modeState == 1  % Direct kinematics
                theta = angle_rad;
                [phi, alpha, A] = fourbar_direct_kinematics(c, b, a, d, theta, configState);
                B = C + c * [cos(theta), sin(theta)];
                if showAlternate
                    [~, ~, A_alt] = fourbar_direct_kinematics(c, b, a, d, theta, -configState);
                    B_alt = C + c * [cos(theta), sin(theta)];
                end
            else  % Inverse kinematics
                alpha = angle_rad;
                [theta, phi, A] = fourbar_inverse_kinematics(c, b, a, d, alpha, configState);
                B = C + c * [cos(theta), sin(theta)];
                if showAlternate
                    [theta_alt, ~, A_alt] = fourbar_inverse_kinematics(c, b, a, d, alpha, -configState);
                    B_alt = C + c * [cos(theta_alt), sin(theta_alt)];
                end
            end

            % Clear and hold axes for new plot
            cla(ax); hold(ax, 'on');

            % Plot main linkage lines
            plot(ax, [O(1) A(1)], [O(2) A(2)], 'r-', 'LineWidth', 2);
            plot(ax, [A(1) B(1)], [A(2) B(2)], 'g-', 'LineWidth', 2);
            plot(ax, [B(1) C(1)], [B(2) C(2)], 'b-', 'LineWidth', 2);
            % Draw ground symbols at O and C
            drawGroundSymbol(ax, O, C);
            % Plot joint markers
            plot(ax, [O(1), A(1), B(1), C(1)], [O(2), A(2), B(2), C(2)], 'ko', 'MarkerFaceColor', 'k');

            % Plot alternate configuration if enabled
            if showAlternate
                plot(ax, [O(1) A_alt(1)], [O(2) A_alt(2)], 'r--', 'LineWidth', 1.5);
                plot(ax, [A_alt(1) B_alt(1)], [A_alt(2) B_alt(2)], 'g--', 'LineWidth', 1.5);
                plot(ax, [B_alt(1) C(1)], [B_alt(2) C(2)], 'b--', 'LineWidth', 1.5);
                plot(ax, [O(1), A_alt(1), B_alt(1), C(1)], [O(2), A_alt(2), B_alt(2), C(2)], ...
                    'ko', 'MarkerFaceColor', [0.5 0.5 0.5]);
            end

            % Set axes limits and title
            xlim(ax, [-200 200]); ylim(ax, [-150 150]);
            title(ax, 'Four-Bar Linkage');

            % Update text info based on mode
            if modeState == 1
                str = sprintf('Coupler angle φ: %.2f°\nOutput angle α: %.2f°', ...
                    rad2deg(phi), rad2deg(alpha));
            else
                str = sprintf('Coupler angle φ: %.2f°\nInput angle θ: %.2f°', ...
                    rad2deg(phi), rad2deg(theta));
            end
            set(angle_text, 'String', str);

        catch ME
            % Handle unreachable configurations
            cla(ax);
            text(0.2, 0.5, 'Configuration not possible', 'Parent', ax, ...
                'Color', 'r', 'FontSize', 14);
            set(angle_text, 'String', ME.message);
        end
    end
end

% Helper: draw ground link symbols at two pivot points

function drawGroundSymbol(ax, P1, P2)
for P = [P1; P2]'
    drawSymbolAt(ax, P);
end
end

function drawSymbolAt(ax, jointCenter)
numLines = 3;   % Number of short lines
lineLength = 10; lineSpacing = 5;
angle = pi/4;   % Tilt of ground symbol lines
R = [cos(angle+pi), -sin(angle+pi); sin(angle+pi), cos(angle+pi)];
v = [lineLength; 0]; vR = R * v;
for i = 0:numLines-1
    offset = jointCenter + [i*lineSpacing - (numLines-1)/2*lineSpacing; 0];
    endPt = offset + vR;
    plot(ax, [offset(1), endPt(1)], [offset(2), endPt(2)], 'k', 'LineWidth', 1.5);
end
plot(ax, jointCenter(1) + [-(numLines-1)/2*lineSpacing, (numLines-1)/2*lineSpacing], ...
    [jointCenter(2), jointCenter(2)], 'k', 'LineWidth', 1.5);
end

% Helper: compute workspace of the linkage for direct and inverse modes

function range = computeFeasibleAngleRange(a, b, c, d, config, mode)
% COMPUTEFEASIBLERANGER - Determine valid motion range of input/output
angle_vals = linspace(0, 2*pi, 361);
valid = false(size(angle_vals));
for i = 1:length(angle_vals)
    try
        if mode == 1
            fourbar_direct_kinematics(c, b, a, d, angle_vals(i), config);
        else
            fourbar_inverse_kinematics(c, b, a, d, angle_vals(i), config);
        end
        valid(i) = true;
    catch
        valid(i) = false;
    end
end
degs = rad2deg(angle_vals(valid));
if isempty(degs)
    range = [];
else
    range = [min(degs), max(degs)];
end
end

% Helper: misc

function out = ternary(cond, a, b)
% TERNARY - Simple conditional helper
if cond, out = a; else, out = b; end
end