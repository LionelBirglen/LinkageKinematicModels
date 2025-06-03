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
% INITIALIZATION
% ---------------------------------------------------------------------

%— Default geometry/state —
default = struct(...
    'a', 81, ...               % output link O→A
    'b', 88, ...               % coupler B→A
    'c', 92, ...               % input crank C→B
    'd', 151, ...              % ground link O→C
    'e', 80, ...                % distance A→P along coupler
    'epsilon', pi/6, ...          % ε in radians (converted from degrees)
    'angle_deg', 106, ...      % initial theta (if Direct) or α (if Inverse) in degrees
    'config', -1 ...           % –1 = crossed, +1 = open
    );

%— GUI state flags —
configState   = default.config;  % +1=open, –1=crossed
modeState     = 1;               % 1=Direct, 2=Inverse
showAlternate = true;            % show alternate config?
animating     = false;           % animation on/off

% ---------------------------------------------------------------------
% GUI COMPONENTS
% ---------------------------------------------------------------------

%— Create figure & controls —
f = figure('Name','Four-Bar Linkage GUI','Position',[300 100 900 600]);

% a (output link)
uicontrol('Style','text','Position',[10 550 120 20],'String','a (O→A)');
input_a = uicontrol('Style','edit','Position',[130 550 150 20], ...
    'String',num2str(default.a), 'Callback',@updatePlot);

% b (coupler)
uicontrol('Style','text','Position',[10 520 120 20],'String','b (B→A)');
input_b = uicontrol('Style','edit','Position',[130 520 150 20], ...
    'String',num2str(default.b), 'Callback',@updatePlot);

% c (input crank)
uicontrol('Style','text','Position',[10 490 120 20],'String','c (C→B)');
input_c = uicontrol('Style','edit','Position',[130 490 150 20], ...
    'String',num2str(default.c), 'Callback',@updatePlot);

% d (ground link)
uicontrol('Style','text','Position',[10 460 120 20],'String','d (O→C)');
input_d = uicontrol('Style','edit','Position',[130 460 150 20], ...
    'String',num2str(default.d), 'Callback',@updatePlot);

% e (distance A→P)
uicontrol('Style','text','Position',[10 430 120 20],'String','e = |A–P|');
input_e = uicontrol('Style','edit','Position',[130 430 150 20], ...
    'String',num2str(default.e), 'Callback',@updatePlot);

% ε (angle from coupler‐extension, in degrees)
uicontrol('Style','text','Position',[10 400 120 20],'String','ε (deg)');
input_epsilon = uicontrol('Style','edit','Position',[130 400 150 20], ...
    'String',num2str(rad2deg(default.epsilon)), 'Callback',@updatePlot);

% Angle (theta if Direct, α if Inverse), in degrees
uicontrol('Style','text','Position',[10 370 120 20],'String','Angle (°)');
input_angle = uicontrol('Style','edit','Position',[130 370 150 20], ...
    'String',num2str(default.angle_deg), 'Callback',@syncSlider);

% Slider for angle
angle_slider = uicontrol('Style','slider','Position',[10 340 270 20], ...
    'Min',-180,'Max',180,'Value',default.angle_deg, ...
    'Callback',@syncAngleEdit);

% Toggle configuration button
toggle_btn = uicontrol('Style','togglebutton','String','Config: Crossed', ...
    'Position',[10 300 270 30], 'Callback',@toggleConfig);

% Mode popup: Direct / Inverse
uicontrol('Style','text','Position',[10 260 120 20],'String','Mode');
mode_menu = uicontrol('Style','popupmenu','Position',[130 260 150 20], ...
    'String',{'Direct','Inverse'}, 'Callback',@modeChanged);

% Checkbox: show alternate configuration
show_alt_checkbox = uicontrol('Style','checkbox','Position',[10 230 270 20], ...
    'String','Show alternate config','Value',1, ...
    'Callback',@toggleAlt);

% Checkbox: show P trajectory (for full crank rotation)
traj_checkbox = uicontrol('Style','checkbox','Position',[10 210 270 20], ...
    'String','Show P trajectory','Value',0, ...
    'Callback',@updatePlot);

% Text area for φ/theta/α & P coords
angle_text = uicontrol('Style','text','Position',[10 180 300 20], ...
    'FontSize',10,'HorizontalAlignment','left');

% Animate button
animate_btn = uicontrol('Style','pushbutton','String','Animate', ...
    'Position',[10 140 270 30], 'Callback',@toggleAnimation);

% Axes for drawing linkage
ax = axes('Units','pixels','Position',[320 100 550 450]);
axis(ax,'equal'); grid(ax,'on'); hold(ax,'on');

% Initial draw
updatePlot();

% ---------------------------------------------------------------------
% Nested callback functions and helpers
% ---------------------------------------------------------------------

    function toggleConfig(~,~)
        configState = -configState;
        if configState == +1
            set(toggle_btn,'String','Config: Open');
        else
            set(toggle_btn,'String','Config: Crossed');
        end
        updatePlot();
    end

    function toggleAlt(~,~)
        showAlternate = logical(get(show_alt_checkbox,'Value'));
        updatePlot();
    end

    function modeChanged(~,~)
        oldMode = modeState;
        modeState = get(mode_menu,'Value');  % 1=Direct, 2=Inverse
        try
            % Preserve “angle” when switching modes
            if oldMode == 1
                % Direct→Inverse: get α from current theta
                theta_curr = deg2rad(str2double(get(input_angle,'String')));
                geo_val = readGeo();
                [~, alpha_new, ~, ~] = fourbar_direct_kinematics(geo_val, theta_curr, configState);
                new_deg = rad2deg(alpha_new);
            else
                % Inverse→Direct: get theta from current α
                alpha_curr = deg2rad(str2double(get(input_angle,'String')));
                geo_val = readGeo();
                [theta_new, ~, ~, ~] = fourbar_inverse_kinematics(geo_val, alpha_curr, configState);
                new_deg = rad2deg(theta_new);
            end
            set(input_angle,'String',num2str(new_deg));
            set(angle_slider,'Value',new_deg);
        catch
            % If conversion fails, do nothing
        end
        updatePlot();
    end

    function syncSlider(~,~)
        val = str2double(get(input_angle,'String'));
        if isnan(val), return; end
        val = max(min(val,180), -180);
        set(input_angle,'String',num2str(val));
        set(angle_slider,'Value',val);
        updatePlot();
    end

    function syncAngleEdit(~,~)
        val = get(angle_slider,'Value');
        set(input_angle,'String',num2str(val));
        updatePlot();
    end

    function toggleAnimation(~,~)
        animating = ~animating;
        if animating
            set(animate_btn,'String','Stop');
            runAnimation();
        else
            set(animate_btn,'String','Animate');
        end
    end

    function runAnimation()
        % Get the current angle from the edit‐box (in degrees)
        start_deg = str2double(get(input_angle,'String'));
        if isnan(start_deg)
            animating = false;
            return;
        end

        % Compute feasible range in degrees for the current mode/config
        geo_val = readGeo();
        rng_deg = computeFeasibleAngleRange(geo_val, configState, modeState);
        if isempty(rng_deg)
            animating = false;
            return;
        end

        % Unpack the range
        min_deg = rng_deg(1);
        max_deg = rng_deg(2);

        % Clamp start_deg into [min_deg, max_deg]
        if start_deg < min_deg
            start_deg = min_deg;
        elseif start_deg > max_deg
            start_deg = max_deg;
        end

        % Number of steps for each segment
        N = 200;
        % First segment: from current angle → max_deg
        if max_deg > start_deg
            angles1 = linspace(start_deg, max_deg, ceil(N * (max_deg - start_deg)/(max_deg - min_deg)));
        else
            angles1 = start_deg;
        end
        % Second segment: from min_deg → current angle
        if start_deg > min_deg
            angles2 = linspace(min_deg, start_deg, N - numel(angles1));
        else
            angles2 = [];
        end

        angles = [angles1, angles2];

        % Loop through that custom ordering
        for ang = angles
            if ~animating || ~ishandle(f), break; end
            set(input_angle,'String',num2str(ang));
            set(angle_slider,'Value',ang);
            updatePlot();
            drawnow;
            pause(0.02);
        end

        % Once finished (or interrupted), reset button text
        if ishandle(animate_btn)
            set(animate_btn,'String','Animate');
        end
        animating = false;
    end


    function updatePlot(~,~)
        cla(ax); hold(ax,'on');
        try
            geo_val   = readGeo();

            %— Plot trajectory of P if requested —
            if get(traj_checkbox,'Value')
                thetas = linspace(0, 2*pi, 360);
                P_traj = nan(2, numel(thetas));
                if showAlternate
                    P_traj_alt = nan(2, numel(thetas));
                end
                for ii = 1:numel(thetas)
                    try
                        [~, ~, ~, P_temp] = fourbar_direct_kinematics(geo_val, thetas(ii), configState);
                        P_traj(:,ii) = P_temp(:);
                    catch
                        % skip invalid positions
                    end
                    if showAlternate
                        try
                            [~, ~, ~, P_temp_alt] = fourbar_direct_kinematics(geo_val, thetas(ii), -configState);
                            P_traj_alt(:,ii) = P_temp_alt(:);
                        catch
                            % skip invalid positions
                        end
                    end
                end
                % Current‐config trajectory
                plot(ax, P_traj(1,:), P_traj(2,:), 'k:', 'LineWidth',1);
                % Alternate‐config trajectory (if showing alternate)
                if showAlternate
                    plot(ax, P_traj_alt(1,:), P_traj_alt(2,:), 'k--', 'LineWidth',1);
                end
            end

            angle_deg = str2double(get(input_angle,'String'));
            angle_rad = deg2rad(angle_deg);

            if modeState == 1
                % Direct kinematics
                [phi, alpha, A, P] = fourbar_direct_kinematics(geo_val, angle_rad, configState);
                theta = angle_rad;
                B = [geo_val(4), 0] + geo_val(3)*[cos(theta), sin(theta)];
                if showAlternate
                    [~, ~, A_alt, P_alt] = fourbar_direct_kinematics(geo_val, theta, -configState);
                    B_alt = [geo_val(4), 0] + geo_val(3)*[cos(theta), sin(theta)];
                end
            else
                % Inverse kinematics
                alpha = angle_rad;
                [theta, phi, A, P] = fourbar_inverse_kinematics(geo_val, alpha, configState);
                B = [geo_val(4), 0] + geo_val(3)*[cos(theta), sin(theta)];
                if showAlternate
                    [theta_alt, ~, A_alt, P_alt] = fourbar_inverse_kinematics(geo_val, alpha, -configState);
                    B_alt = [geo_val(4), 0] + geo_val(3)*[cos(theta_alt), sin(theta_alt)];
                end
            end

            %— Draw the filled triangular “coupler body” (A–B–P) in translucent green —
            patch( ...
                'XData',[A(1), B(1), P(1)], ...
                'YData',[A(2), B(2), P(2)], ...
                'FaceColor','g', ...
                'EdgeColor','none', ...
                'FaceAlpha',0.5, ...
                'Parent',ax ...
                );

            %— Draw ground link O–C —
            O = [0, 0];
            C = [geo_val(4), 0];
            plot(ax, [O(1), C(1)], [O(2), C(2)], 'k-', 'LineWidth',2);

            %— Draw current-config links (on top of the patch) —
            plot(ax, [O(1), A(1)], [O(2), A(2)], 'r-', 'LineWidth',2);   % Outlink O→A
            plot(ax, [A(1), B(1)], [A(2), B(2)], 'g-', 'LineWidth',2);   % Coupler B→A
            plot(ax, [B(1), C(1)], [B(2), C(2)], 'b-', 'LineWidth',2);   % Crank C→B

            %— Plot joints as filled circles —
            scatter(ax, [O(1), A(1), B(1), C(1)], [O(2), A(2), B(2), C(2)], 40, 'k','filled');

            %— Plot point P (black “×”) —
            plot(ax, P(1), P(2), 'kx', 'MarkerSize',8, 'LineWidth',2);

            %— Plot alternate-config triangle and links if requested —
            if showAlternate
                patch( ...
                    'XData',[A_alt(1), B_alt(1), P_alt(1)], ...
                    'YData',[A_alt(2), B_alt(2), P_alt(2)], ...
                    'FaceColor','g', ...
                    'EdgeColor','none', ...
                    'FaceAlpha',0.3, ...
                    'Parent',ax ...
                    );
                plot(ax, [O(1), A_alt(1)], [O(2), A_alt(2)], 'r--', 'LineWidth',1.5);
                plot(ax, [A_alt(1), B_alt(1)], [A_alt(2), B_alt(2)], 'g--', 'LineWidth',1.5);
                plot(ax, [B_alt(1), C(1)], [B_alt(2), C(2)], 'b--', 'LineWidth',1.5);
                scatter(ax, [A_alt(1), B_alt(1)], [A_alt(2), B_alt(2)], 30, 'k','filled');
                plot(ax, P_alt(1), P_alt(2), 'g+', 'MarkerSize',8, 'LineWidth',1.5);
            end

            %— Draw tiny ground symbols at O and C —
            drawGroundSymbol(ax, O, C);

            %— Auto-scale axes —
            Lmax = max(geo_val(1:4)) * 1.3;
            axis(ax, [-Lmax Lmax -Lmax Lmax]);

            %— Display textual readout —
            if modeState == 1
                txt = sprintf('φ = %.1f°   α = %.1f°   P = (%.2f, %.2f)', ...
                    rad2deg(phi), rad2deg(alpha), P(1), P(2));
            else
                txt = sprintf('θ = %.1f°   φ = %.1f°   P = (%.2f, %.2f)', ...
                    rad2deg(theta), rad2deg(phi), P(1), P(2));
            end
            set(angle_text,'String',txt);

        catch ME
            % Invalid/unreachable → show error
            cla(ax);
            text(0,0,'Unreachable','Parent',ax,'Color','r','FontSize',14,'HorizontalAlignment','center');
            set(angle_text,'String',ME.message);
        end
    end


    function geo_out = readGeo()
        % READGEO  Read [a b c d e epsilon] from GUI fields
        a_val       = str2double(get(input_a,'String'));
        b_val       = str2double(get(input_b,'String'));
        c_val       = str2double(get(input_c,'String'));
        d_val       = str2double(get(input_d,'String'));
        e_val       = str2double(get(input_e,'String'));
        eps_deg_val = str2double(get(input_epsilon,'String'));
        epsilon_val = deg2rad(eps_deg_val);  % convert to radians
        geo_out = [a_val, b_val, c_val, d_val, e_val, epsilon_val];
    end

    function drawGroundSymbol(axh, O_pt, C_pt)
        % Draw small “L” shapes at O and C
        Ls = max(get(axh,'XLim')) * 0.02;
        plot(axh, [O_pt(1)-Ls, O_pt(1)+Ls], [O_pt(2), O_pt(2)], 'k-', 'LineWidth',2);
        plot(axh, [O_pt(1), O_pt(1)], [O_pt(2)-Ls, O_pt(2)+Ls], 'k-', 'LineWidth',2);
        plot(axh, [C_pt(1)-Ls, C_pt(1)+Ls], [C_pt(2), C_pt(2)], 'k-', 'LineWidth',2);
        plot(axh, [C_pt(1), C_pt(1)], [C_pt(2)-Ls, C_pt(2)+Ls], 'k-', 'LineWidth',2);
    end

    function rng_deg = computeFeasibleAngleRange(geo_val, cfg, md)
        % COMPUTEFEASIBLEANGLERANGE  Returns [min,max] in degrees for theta or α
        try
            a_len = geo_val(1);
            b_len = geo_val(2);
            c_len = geo_val(3);
            d_len = geo_val(4);

            if md == 1
                % Direct: vary theta ∈ [–π,π]
                thetas = linspace(-pi, pi, 361);
                valid_th = false(size(thetas));
                for i = 1:numel(thetas)
                    try
                        fourbar_direct_kinematics(geo_val, thetas(i), cfg);
                        valid_th(i) = true;
                    catch
                        valid_th(i) = false;
                    end
                end
                if ~any(valid_th), rng_deg = []; return; end
                ths = thetas(valid_th);
                rng_deg = [rad2deg(min(ths)), rad2deg(max(ths))];
            else
                % Inverse: vary α ∈ [–π,π]
                alphas = linspace(-pi, pi, 361);
                valid_al = false(size(alphas));
                for i = 1:numel(alphas)
                    try
                        fourbar_inverse_kinematics(geo_val, alphas(i), cfg);
                        valid_al(i) = true;
                    catch
                        valid_al(i) = false;
                    end
                end
                if ~any(valid_al), rng_deg = []; return; end
                als = alphas(valid_al);
                rng_deg = [rad2deg(min(als)), rad2deg(max(als))];
            end
        catch
            rng_deg = [];
        end
    end

end
