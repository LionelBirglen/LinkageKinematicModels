function rrr_gui()
% RRR_GUI - Interactive GUI for a Planar RRR Serial Linkage
%
% OBJECTIVE:
%   Launch a MATLAB graphical user interface to analyze and visualize
%   the kinematics of a planar 3R serial robot (RRR manipulator).
%   Supports both forward (direct) and inverse kinematics, elbow-up/down
%   configuration switching, and animated motion.
%
% INPUTS:
%   None. All parameters are set via UI controls within the GUI.
%
% OUTPUTS:
%   Visual feedback on the figure window. No MATLAB outputs.
%
% USAGE EXAMPLE:
%   >> rrr_gui
%   Opens the GUI with default link lengths and joint values.
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

% Create Main Figure
f = figure('Name', 'Planar RRR Linkage', ...
           'Position', [100 100 950 750], ...
           'DeleteFcn', @(~,~) cleanupTimers());

% Link lengths (mm)
default.L1    = 57;   % Length of link 1
default.L2    = 46;   % Length of link 2
default.L3    = 51;   % Length of link 3
% Joint angles (degrees) for direct kinematics
default.theta1 = 39;  % Joint 1 angle
default.theta2 = 37;  % Joint 2 angle
default.theta3 = 40;  % Joint 3 angle
% End-effector pose for inverse kinematics
default.Px    = 35;   % X coordinate (mm)
default.Py    = 125;  % Y coordinate (mm)
default.phi   = 116;  % Orientation (deg)
% Initial mode and configuration
default.config = 1;   % +1: elbow-up, -1: elbow-down
default.mode   = 1;   % 1: direct, 2: inverse

% State variables for current configuration and mode
configState     = default.config;
modeState       = default.mode;
animating      = false;
animation_timer = [];

% ---------------------------------------------------------------------
% GUI COMPONENTS
% ---------------------------------------------------------------------

% Link length inputs
uicontrol('Style','text','Position',[10 700 100 20],'String','Link 1 (L1)');
input_L1 = uicontrol('Style','edit','Position',[110 700 100 20], ...
    'String', num2str(default.L1), 'Callback', @updatePlot);

uicontrol('Style','text','Position',[10 670 100 20],'String','Link 2 (L2)');
input_L2 = uicontrol('Style','edit','Position',[110 670 100 20], ...
    'String', num2str(default.L2), 'Callback', @updatePlot);

uicontrol('Style','text','Position',[10 640 100 20],'String','Link 3 (L3)');
input_L3 = uicontrol('Style','edit','Position',[110 640 100 20], ...
    'String', num2str(default.L3), 'Callback', @updatePlot);

% Mode & Configuration Controls
uicontrol('Style','text','Position',[10 600 100 20],'String','Mode');
mode_menu = uicontrol('Style','popupmenu', 'String', {'Direct','Inverse'}, ...
    'Position',[110 600 100 20], 'Callback', @modeChanged);

toggle_btn = uicontrol('Style','togglebutton','String','Elbow: Up', ...
    'Position',[10 570 200 25],'Callback',@toggleConfig);

show_both_checkbox = uicontrol('Style','checkbox','String','Show both configs', ...
    'Position',[10 540 200 25],'Value',0,'Callback',@updatePlot);

% Joint Angle Sliders (Direct Mode)
slider_theta1 = makeSlider('θ₁ (deg)',10,490,0,360,default.theta1,@updatePlot);
theta1_val    = uicontrol('Style','text','Position',[220 490 60 20], ...
    'String',sprintf('%.1f°',default.theta1));

slider_theta2 = makeSlider('θ₂ (deg)',10,440,0,360,default.theta2,@updatePlot);
theta2_val    = uicontrol('Style','text','Position',[220 440 60 20], ...
    'String',sprintf('%.1f°',default.theta2));

slider_theta3 = makeSlider('θ₃ (deg)',10,390,0,360,default.theta3,@updatePlot);
theta3_val    = uicontrol('Style','text','Position',[220 390 60 20], ...
    'String',sprintf('%.1f°',default.theta3));

%  End-Effector Sliders (Inverse Mode)
slider_px  = makeSlider('X target',10,340,-200,200,default.Px,@updatePlot);
px_val     = uicontrol('Style','text','Position',[220 340 60 20], ...
    'String',sprintf('%.1f',default.Px));

slider_py  = makeSlider('Y target',10,290,-200,200,default.Py,@updatePlot);
py_val     = uicontrol('Style','text','Position',[220 290 60 20], ...
    'String',sprintf('%.1f',default.Py));

slider_phi = makeSlider('φ (deg)',10,240,-180,180,default.phi,@updatePlot);
phi_val    = uicontrol('Style','text','Position',[220 240 60 20], ...
    'String',sprintf('%.1f°',default.phi));

%  Animation Control & Info Text
animate_btn = uicontrol('Style','togglebutton','String','Animate', ...
    'Position',[10 170 270 30],'Callback',@toggleAnimation);

info_text = uicontrol('Style','text','Position',[10 90 270 60], ...
    'HorizontalAlignment','left','FontSize',11);

% Axes for drawing the mechanism
ax = axes('Position',[0.4 0.1 0.55 0.8]);
axis equal; grid on;

% Initial rendering of the mechanism
updatePlot();

% ---------------------------------------------------------------------
% Nested callback functions and helpers
% ---------------------------------------------------------------------

    function toggleConfig(~,~)
        % TOGGLECONFIG - flip between elbow-down and elbow-up configurations
        configState = -configState;
        % Update button label
        set(toggle_btn,'String',sprintf('Elbow: %s',ternary(configState==1,'Up','Down')));
        updatePlot();
    end

    function modeChanged(~,~)
        % Handle switching between direct (1) and inverse (2) modes
        prev_mode = modeState;
        modeState = get(mode_menu,'Value');
        
        % Read current link lengths
        L1 = str2double(get(input_L1,'String'));
        L2 = str2double(get(input_L2,'String'));
        L3 = str2double(get(input_L3,'String'));
        
        if prev_mode~=modeState
            if prev_mode==1 && modeState==2
                % Direct → Inverse: compute pose from joints
                t1 = deg2rad(get(slider_theta1,'Value'));
                t2 = deg2rad(get(slider_theta2,'Value'));
                t3 = deg2rad(get(slider_theta3,'Value'));
                [~,~,P3] = rrr_direct_kinematics(L1,L2,L3,t1,t2,t3);
                phi_tot = rad2deg(t1+t2+t3);
                set(slider_px,'Value',P3(1)); set(px_val,'String',sprintf('%.1f',P3(1)));
                set(slider_py,'Value',P3(2)); set(py_val,'String',sprintf('%.1f',P3(2)));
                set(slider_phi,'Value',phi_tot); set(phi_val,'String',sprintf('%.1f°',phi_tot));
            elseif prev_mode==2 && modeState==1
                % Inverse → Direct: compute joints from pose
                Px = get(slider_px,'Value'); Py = get(slider_py,'Value');
                phi = deg2rad(get(slider_phi,'Value'));
                [t1,t2,t3,ok] = rrr_inverse_kinematics(L1,L2,L3,Px,Py,phi,configState);
                if ok
                    set(slider_theta1,'Value',rad2deg(t1)); set(theta1_val,'String',sprintf('%.1f°',rad2deg(t1)));
                    set(slider_theta2,'Value',rad2deg(t2)); set(theta2_val,'String',sprintf('%.1f°',rad2deg(t2)));
                    set(slider_theta3,'Value',rad2deg(t3)); set(theta3_val,'String',sprintf('%.1f°',rad2deg(t3)));
                else
                    warning('Target unreachable.');
                end
            end
        end
        updatePlot();
    end

    function updatePlot(~,~)
        % Redraws linkage based on current UI values and mode
        cla(ax); hold(ax,'on');
        L1 = str2double(get(input_L1,'String'));
        L2 = str2double(get(input_L2,'String'));
        L3 = str2double(get(input_L3,'String'));
        if modeState==1
            % Forward kinematics
            t1 = deg2rad(get(slider_theta1,'Value'));
            t2 = deg2rad(get(slider_theta2,'Value'));
            t3 = deg2rad(get(slider_theta3,'Value'));
            set([slider_px slider_py slider_phi],'Enable','off');
            set([slider_theta1 slider_theta2 slider_theta3],'Enable','on');
            [P1,P2,P3] = rrr_direct_kinematics(L1,L2,L3,t1,t2,t3);
            drawLinkage(P1,P2,P3,t1+t2+t3,'-');
            set(info_text,'String',sprintf('X=%.2f, Y=%.2f, φ=%.2f°',P3(1),P3(2),rad2deg(t1+t2+t3)));
        else
            % Inverse kinematics
            Px = get(slider_px,'Value'); Py = get(slider_py,'Value');
            phi = deg2rad(get(slider_phi,'Value'));
            set([slider_theta1 slider_theta2 slider_theta3],'Enable','off');
            set([slider_px slider_py slider_phi],'Enable','on');
            [t1,t2,t3,ok] = rrr_inverse_kinematics(L1,L2,L3,Px,Py,phi,configState);
            if ~ok
                text(0,0,'Unreachable','Parent',ax,'Color','r','FontSize',14);
                return;
            end
            [P1,P2,P3] = rrr_direct_kinematics(L1,L2,L3,t1,t2,t3);
            drawLinkage(P1,P2,P3,phi,'-');
            if get(show_both_checkbox,'Value')
                [u1,u2,u3,ok2] = rrr_inverse_kinematics(L1,L2,L3,Px,Py,phi,-configState);
                if ok2
                    [Q1,Q2,Q3] = rrr_direct_kinematics(L1,L2,L3,u1,u2,u3);
                    drawLinkage(Q1,Q2,Q3,phi,'--');
                end
            end
            set(info_text,'String',sprintf('θ₁=%.1f°,θ₂=%.1f°,θ₃=%.1f°',rad2deg(t1),rad2deg(t2),rad2deg(t3)));
        end
        xlim([-200 200]); ylim([-200 200]); title('Planar RRR Linkage');
    end

    function drawLinkage(P1,P2,P3,phi,style)
        % Plots links and joints: yellow bases, colored links
        cols = {'r','g','b'};
        segs = {[0 P1(1);0 P1(2)], [P1(1) P2(1);P1(2) P2(2)], [P2(1) P3(1);P2(2) P3(2)]};
        for k=1:3
            plot(ax,segs{k}(1,:),segs{k}(2,:), 'Color',cols{k}, 'LineStyle',style,'LineWidth',2);
        end
        plot(ax, [0 P1(1) P2(1) P3(1)], [0 P1(2) P2(2) P3(2)], 'ko','MarkerFaceColor','k');
    end

    function toggleAnimation(src,~)
        % Starts/stops continuous update via MATLAB timer
        animating = get(src,'Value');
        if animating
            % Disable controls during animation
            set([input_L1 input_L2 input_L3 mode_menu toggle_btn show_both_checkbox ...
                 slider_theta1 slider_theta2 slider_theta3 slider_px slider_py slider_phi], 'Enable','off');
            animation_timer = timer('ExecutionMode','fixedRate','Period',0.05,'TimerFcn',@animateStep);
            start(animation_timer);
        else
            % Re-enable controls and stop timer
            if ~isempty(animation_timer) && isvalid(animation_timer)
                stop(animation_timer); delete(animation_timer);
            end
            set([input_L1 input_L2 input_L3 mode_menu toggle_btn show_both_checkbox ...
                 slider_theta1 slider_theta2 slider_theta3 slider_px slider_py slider_phi], 'Enable','on');
        end
    end

    function animateStep(~,~)
        % Advance the sliders for animation effect
        if modeState==1
            inc = [2,1,1];
            for i=1:3
                sld = eval(sprintf('slider_theta%d',i));
                val = mod(get(sld,'Value')+inc(i),360);
                set(sld,'Value',val);
            end
        else
            % Bounce Px between limits
            cur = get(slider_px,'Value');
            persistent dir; if isempty(dir), dir=1; end
            nxt = cur + dir*2;
            if nxt>200, nxt=200; dir=-1; elseif nxt<-200, nxt=-200; dir=1; end
            set(slider_px,'Value',nxt);
        end
        updatePlot();
    end

    function cleanupTimers()
        % Clean up timer when GUI closes
        if ~isempty(animation_timer) && isvalid(animation_timer)
            stop(animation_timer); delete(animation_timer);
        end
    end

end

% Helper: draw slider control

function s = makeSlider(label,x,y,minV,maxV,init,cb)
% MAKE SLIDER - Creates a labeled slider control
uicontrol('Style','text','Position',[x y+20 100 20],'String',label);
s = uicontrol('Style','slider','Position',[x y 200 20],...
    'Min',minV,'Max',maxV,'Value',init,'SliderStep',[0.01 0.1],'Callback',cb);
end

% Helper: misc

function out = ternary(cond,a,b)
% TERNARY - Conditional helper: returns a if cond is true, else b
if cond, out=a; else out=b; end
end
