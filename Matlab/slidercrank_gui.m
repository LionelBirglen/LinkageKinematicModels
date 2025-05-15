function slidercrank_gui()
% SLIDERCRANK_GUI - Interactive GUI for a Planar Slider-Crank Mechanism
%
% OBJECTIVE:
%   Launch a MATLAB GUI to simulate and visualize a planar slider-crank
%   linkage mechanism. Supports both direct (forward) and inverse kinematics,
%   configuration toggling (elbow-up/down), and animation of mechanism motion.
%
% INPUTS:
%   None. All parameters (link lengths, angles, modes) are controlled
%   via the GUI elements created within this function.
%
% OUTPUTS:
%   No direct output arguments. Visualization and interaction occur in the
%   generated figure window.
%
% USAGE EXAMPLE:
%   >> slidercrank_gui
%   Opens the GUI with default mechanism parameters.
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

% Create the main figure window
f = figure('Name', 'Slider-Crank Linkage Kinematics', ...
    'Position', [100 100 850 600]);

% Default linkage parameters
default = struct('a', 50, ...                   % Crank length
    'b', 120, ...                  % Coupler length
    'phi', deg2rad(30), ...        % Initial crank angle (rad)
    'slider_angle', deg2rad(0),... % Slider axis orientation (rad)
    'config', 1);                  % Configuration flag (+1 elbow-down, -1 elbow-up)

% State variables for current configuration and mode
configState = default.config;  % Current elbow configuration
modeState   = 1;               % Kinematic mode: 1=direct, 2=inverse
animating   = false;           % Animation on/off flag
animation_timer = [];          % MATLAB timer object for animation

% ---------------------------------------------------------------------
% GUI COMPONENTS
% ---------------------------------------------------------------------

% Crank length 'a' input field
uicontrol('Style','text','Position',[10 550 120 20],'String','Crank length a');
input_a = uicontrol('Style','edit', ...
    'String',num2str(default.a), ...
    'Position',[130 550 150 20], ...
    'Callback',@updatePlot);

% Coupler length 'b' input field
uicontrol('Style','text','Position',[10 520 120 20],'String','Coupler length b');
input_b = uicontrol('Style','edit', ...
    'String',num2str(default.b), ...
    'Position',[130 520 150 20], ...
    'Callback',@updatePlot);

% Slider axis orientation input (in degrees)
uicontrol('Style','text','Position',[10 490 120 20],'String','Slider angle (deg)');
input_slider_angle = uicontrol('Style','edit', ...
    'String',num2str(rad2deg(default.slider_angle)), ...
    'Position',[130 490 150 20], ...
    'Callback',@updatePlot);

% Label and input for crank angle or slider position
angle_label = uicontrol('Style','text', ...
    'Position',[10 430 120 20], ...
    'String','Crank angle φ (deg)');
input_angle = uicontrol('Style','edit', ...
    'String',num2str(rad2deg(default.phi)), ...
    'Position',[130 430 150 20], ...
    'Callback',@syncSlider);

% Slider control linked to the above edit field
angle_slider = uicontrol('Style','slider', ...
    'Min',0,'Max',360, ...
    'Value',rad2deg(default.phi), ...
    'Position',[10 400 270 20], ...
    'Callback',@syncEdit);

% Toggle button to switch between elbow-down and elbow-up
toggle_btn = uicontrol('Style','togglebutton', ...
    'String','Config: Elbow-Down', ...
    'Position',[10 360 270 30], ...
    'Callback',@toggleConfig);

% Popup menu to choose direct (forward) or inverse kinematics
uicontrol('Style','text','Position',[10 320 120 20],'String','Mode');
mode_menu = uicontrol('Style','popupmenu', ...
    'String',{'Direct','Inverse'}, ...
    'Position',[130 320 150 20], ...
    'Callback',@modeChanged);

% Checkbox to optionally show both kinematic solutions
show_both_checkbox = uicontrol('Style','checkbox', ...
    'String','Show both configs', ...
    'Position',[10 290 270 20], ...
    'Value',0, ...
    'Callback',@updatePlot);

% Button to start/stop animation
animate_btn = uicontrol('Style','togglebutton', ...
    'String','Animate', ...
    'Position',[10 260 270 30], ...
    'Callback',@toggleAnimation);

% Text area displaying current slider/crank values
slider_pos_text = uicontrol('Style','text', ...
    'Position',[10 210 300 40], ...
    'FontSize',12, ...
    'HorizontalAlignment','left');

% Axes for drawing the mechanism
ax = axes('Units','pixels','Position',[320 100 500 450]);
axis equal;
grid on;

% Initial rendering of the mechanism
updatePlot();

% ---------------------------------------------------------------------
% Nested callback functions and helpers
% ---------------------------------------------------------------------

    function toggleAnimation(src, ~)
        % TOGGLEANIMATION - enable or disable the animation timer
        animating = get(src, 'Value');
        if animating
            % Disable UI controls while animating
            set([input_a, input_b, input_slider_angle, input_angle, angle_slider, toggle_btn, mode_menu], 'Enable', 'off');
            animation_timer = timer('ExecutionMode','fixedRate', ...
                'Period',0.05, ...
                'TimerFcn',@animateStep);
            start(animation_timer);
        else
            % Stop timer and re-enable UI
            if ~isempty(animation_timer) && isvalid(animation_timer)
                stop(animation_timer);
                delete(animation_timer);
            end
            set([input_a, input_b, input_slider_angle, input_angle, angle_slider, toggle_btn, mode_menu], 'Enable', 'on');
        end
    end

    function animateStep(~, ~)
        % ANIMATESTEP - update mechanism variables each frame
        if modeState == 1
            % In direct mode: rotate crank continuously
            current_angle = str2double(get(input_angle, 'String'));
            new_angle = mod(current_angle + 2, 360);
            set(input_angle, 'String', num2str(new_angle));
            set(angle_slider, 'Value', new_angle);
        else
            % In inverse mode: oscillate slider along its rail
            current_pos = str2double(get(input_angle, 'String'));
            a = str2double(get(input_a,'String'));
            b = str2double(get(input_b,'String'));
            slider_min = max(a - b, 0);
            slider_max = a + b;
            persistent direction;
            if isempty(direction), direction = 1; end

            new_pos = current_pos + direction * 2;
            if new_pos >= slider_max
                new_pos = slider_max;
                direction = -1;
            elseif new_pos <= slider_min
                new_pos = slider_min;
                direction = 1;
            end
            set(input_angle, 'String', num2str(new_pos));
            set(angle_slider, 'Value', new_pos);
        end
        updatePlot();
    end

    function toggleConfig(~, ~)
        % TOGGLECONFIG - flip between elbow-down and elbow-up configurations
        configState = -configState;
        % Update button label
        set(toggle_btn, 'String', ...
            ['Config: ', ternary(configState==1, 'Elbow-Down', 'Elbow-Up')]);
        updatePlot();
    end

    function modeChanged(~, ~)
        % MODECHANGED - switch between direct and inverse kinematics
        oldMode = modeState;
        modeState = get(mode_menu,'Value');
        % Update label text to reflect mode
        set(angle_label, 'String', ...
            ternary(modeState==1, 'Crank angle φ (deg)', 'Slider position x'));

        a = str2double(get(input_a,'String'));
        b = str2double(get(input_b,'String'));

        if oldMode == 1 && modeState == 2
            % Switched from direct → inverse: compute slider pos from crank angle
            phi = deg2rad(str2double(get(input_angle,'String')));
            slider_angle = deg2rad(str2double(get(input_slider_angle,'String')));
            try
                [x_slider, ~, ~] = slidercrank_direct_kinematics(a, b, phi, slider_angle);
                set(input_angle, 'String', num2str(x_slider));
                set(angle_slider, 'Value', x_slider);
                % Adjust slider range for inverse mode
                slider_min = max(a - b, 0);
                slider_max = a + b;
                set(angle_slider, 'Min', slider_min);
                set(angle_slider, 'Max', slider_max);
                set(angle_slider, 'SliderStep', [(slider_max-slider_min)/100, (slider_max-slider_min)/10]);
            catch
                warning('Could not preserve linkage when switching modes.');
            end
        elseif oldMode == 2 && modeState == 1
            % Switched from inverse → direct: compute crank angle from slider pos
            x_slider = str2double(get(input_angle,'String'));
            slider_angle = deg2rad(str2double(get(input_slider_angle,'String')));
            try
                [phi, ~] = slidercrank_inverse_kinematics(a, b, x_slider, configState, slider_angle);
                new_val = rad2deg(phi);
                set(input_angle, 'String', num2str(new_val));
                set(angle_slider, 'Value', new_val);
                % Reset slider range for direct mode
                set(angle_slider, 'Min', 0);
                set(angle_slider, 'Max', 360);
                set(angle_slider, 'SliderStep', [1/360, 10/360]);
            catch
                warning('Could not preserve linkage when switching modes.');
            end
        end

        updatePlot();
    end

    function syncSlider(~, ~)
        % SYNCSLIDER - clamp text input and sync slider handle
        val = str2double(get(input_angle,'String'));
        if modeState == 1
            val = mod(val, 360);
        else
            slider_min = get(angle_slider, 'Min');
            slider_max = get(angle_slider, 'Max');
            val = max(min(val, slider_max), slider_min);
        end
        set(input_angle, 'String', num2str(val));
        set(angle_slider, 'Value', val);
        updatePlot();
    end

    function syncEdit(src, ~)
        % SYNCEDIT - update text input when slider is moved
        val = get(src, 'Value');
        set(input_angle, 'String', num2str(val));
        updatePlot();
    end

    function updatePlot(~, ~)
        % UPDATEPLOT - redraw mechanism based on current parameters
        a = str2double(get(input_a,'String'));
        b = str2double(get(input_b,'String'));
        slider_angle = deg2rad(str2double(get(input_slider_angle,'String')));

        O = [0, 0];  % Ground joint origin
        slider_dir = [cos(slider_angle), sin(slider_angle)];

        try
            cla(ax); hold(ax, 'on');
            fixed_rail_length = 600;
            rail_start = -fixed_rail_length/2 * slider_dir;
            rail_end   =  fixed_rail_length/2 * slider_dir;
            % Draw slider rail
            plot(ax, [rail_start(1) rail_end(1)], [rail_start(2) rail_end(2)], 'k-.', 'LineWidth', 1);
            drawGroundSymbol(ax, O);

            if modeState == 1
                % DIRECT MODE: compute positions from crank angle
                phi = deg2rad(str2double(get(input_angle,'String')));
                [x_slider, B, P] = slidercrank_direct_kinematics(a, b, phi, slider_angle, configState);

                % Plot links and joint markers
                plot(ax, [O(1) B(1)], [O(2) B(2)], 'r-', 'LineWidth', 2);
                plot(ax, [B(1) P(1)], [B(2) P(2)], 'g-', 'LineWidth', 2);
                drawSlider(ax, P, slider_dir, slider_angle);
                plot(ax, [O(1) B(1) P(1)], [O(2) B(2) P(2)], 'ko', 'MarkerFaceColor','k');

                % OPTIONALLY plot alternate config
                if get(show_both_checkbox,'Value')
                    try
                        [x_alt, B_alt, P_alt] = slidercrank_direct_kinematics(a, b, phi, slider_angle, -configState);
                        plot(ax, [O(1) B_alt(1)], [O(2) B_alt(2)], 'r--', 'LineWidth', 2);
                        plot(ax, [B_alt(1) P_alt(1)], [B_alt(2) P_alt(2)], 'g--', 'LineWidth', 2);
                        plot(ax, [O(1) B_alt(1) P_alt(1)], [O(2) B_alt(2) P_alt(2)], 'ko');
                        drawDashedSlider(ax, P_alt, slider_dir, slider_angle);
                    catch
                        % Ignore if alternate impossible
                    end
                end

                % Update info text
                set(slider_pos_text, 'String', sprintf('Slider position x: %.2f\nCrank angle φ: %.2f°', x_slider, rad2deg(phi)));

            else
                % INVERSE MODE: compute crank angle from slider position
                x_slider = str2double(get(input_angle,'String'));
                [phi, B] = slidercrank_inverse_kinematics(a, b, x_slider, configState, slider_angle);
                P = x_slider * slider_dir;

                % Plot mechanism
                plot(ax, [O(1) B(1)], [O(2) B(2)], 'r-', 'LineWidth', 2);
                plot(ax, [B(1) P(1)], [B(2) P(2)], 'g-', 'LineWidth', 2);
                drawSlider(ax, P, slider_dir, slider_angle);
                plot(ax, [O(1) B(1) P(1)], [O(2) B(2) P(2)], 'ko', 'MarkerFaceColor','k');

                % OPTIONALLY plot alternate
                if get(show_both_checkbox,'Value')
                    try
                        [phi_alt, B_alt] = slidercrank_inverse_kinematics(a, b, x_slider, -configState, slider_angle);
                        P_alt = x_slider * slider_dir;
                        plot(ax, [O(1) B_alt(1)], [O(2) B_alt(2)], 'r--', 'LineWidth', 2);
                        plot(ax, [B_alt(1) P_alt(1)], [B_alt(2) P_alt(2)], 'g--', 'LineWidth', 2);
                        plot(ax, [O(1) B_alt(1) P_alt(1)], [O(2) B_alt(2) P_alt(2)], 'ko');
                        drawDashedSlider(ax, P_alt, slider_dir, slider_angle);
                    catch
                        % Ignore if alternate impossible
                    end
                end

                % Update info text
                set(slider_pos_text, 'String', sprintf('Crank angle φ: %.2f°\nSlider position x: %.2f', rad2deg(phi), x_slider));
            end

            % Set plot limits and title
            axis_limit = fixed_rail_length/2 * 0.7;
            xlim(ax, [-axis_limit axis_limit]);
            ylim(ax, [-axis_limit axis_limit]);
            title(ax, 'Slider-Crank Linkage');

        catch ME
            % Display error message if kinematics fail
            cla(ax);
            text(0.1,0.5, ['Error: ', ME.message], 'Parent', ax, 'Color', 'r', 'FontSize', 12);
            set(slider_pos_text, 'String', 'Configuration not possible');
        end
    end

% Helper: draw ground link symbols and slider

    function drawGroundSymbol(ax, jointCenter)
        numLines = 3;
        lineLength = 10;
        lineSpacing = 5;
        angle = +pi/4;
        R = [cos(angle+pi) -sin(angle+pi); sin(angle+pi) cos(angle+pi)];
        x=[lineLength;0];
        Rx=R*x;
        for i = 0:numLines-1
            v1 = jointCenter + [i*lineSpacing - (numLines-1)/2*lineSpacing;0];
            v2 = v1+Rx;
            plot(ax, [v1(1) v2(1)], [v1(2) v2(2)], 'k', 'LineWidth', 1.5);
        end
        plot(ax, jointCenter(1)+[-(numLines-1)/2*lineSpacing +(numLines-1)/2*lineSpacing], [jointCenter(2) jointCenter(2)], 'k', 'LineWidth', 1.5);

    end
    function drawSlider(ax, P, slider_dir, slider_angle)
        slider_length = 20;
        slider_width = 12;
        perp_dir = [-sin(slider_angle), cos(slider_angle)];
        slider_corners = [
            P + slider_length/2*slider_dir + slider_width/2*perp_dir;
            P + slider_length/2*slider_dir - slider_width/2*perp_dir;
            P - slider_length/2*slider_dir - slider_width/2*perp_dir;
            P - slider_length/2*slider_dir + slider_width/2*perp_dir
            ];
        h_slider = fill(ax, slider_corners(:,1), slider_corners(:,2), 'b');
        set(h_slider, 'FaceAlpha', 0.5);
    end
    function drawDashedSlider(ax, P, slider_dir, slider_angle)
        slider_length = 20;
        slider_width = 12;
        perp_dir = [-sin(slider_angle), cos(slider_angle)];
        slider_corners = [
            P + slider_length/2*slider_dir + slider_width/2*perp_dir;
            P + slider_length/2*slider_dir - slider_width/2*perp_dir;
            P - slider_length/2*slider_dir - slider_width/2*perp_dir;
            P - slider_length/2*slider_dir + slider_width/2*perp_dir;
            P + slider_length/2*slider_dir + slider_width/2*perp_dir % Close the rectangle
            ];
        plot(ax, slider_corners(:,1), slider_corners(:,2), 'b:', 'LineWidth', 1.5);
    end
    function out = ternary(cond, a, b)
        % TERNARY - conditional helper
        if cond, out = a; else, out = b; end
    end

% Ensure timer cleanup on figure close
set(f, 'DeleteFcn', @(~,~) cleanupTimers());
    function cleanupTimers()
        if ~isempty(animation_timer) && isvalid(animation_timer)
            stop(animation_timer);
            delete(animation_timer);
        end
    end
end
