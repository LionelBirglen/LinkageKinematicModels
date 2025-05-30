function fourbar_spherical_gui()
% FOURBAR_SPHERICAL_GUI - Interactive GUI for a Spherical Fourbar Linkage
%
% OBJECTIVE:
%   Launch a MATLAB graphical user interface to analyze and visualize
%   the kinematics of a spherical fourbar linkage.
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
%   >> fourbar_spherical_gui
%   Opens the GUI with default link lengths and joint values.
%
% BY:
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Last Update: 2025/05/30
% Contact: lionel.birglen@polymtl.ca
%
% Code provided under GNU Affero General Public License v3.0


% ---------------------------------------------------------------------
% INITIALIZATION
% ---------------------------------------------------------------------

% Create the main figure
f = figure('Name', 'Fourbar Spherical Linkage', 'NumberTitle', 'off', ...
    'Position', [100, 100, 1100, 750]);

% Default joint angles
theta = 90 ;
alpha = 0 ;

% State variables for current configuration and mode
configState     = -1;
modeState       = 1;
animating       = false;            % Animation on/off flag

% Create axes for plotting
ax = axes('Parent', f, 'Position', [0.4 0.2 0.55 0.75]);
axis_lim = 1.2;
axis(ax, [-axis_lim axis_lim -axis_lim axis_lim -axis_lim axis_lim]);
axis(ax, 'equal'); grid(ax, 'on'); view(ax, 45, 30);
xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
title(ax, 'Fourbar Spherical Serial Linkage'); hold(ax, 'on');

% ---------------------------------------------------------------------
% GUI COMPONENTS
% ---------------------------------------------------------------------

geometry=[45 60 45 90];

% Input boxes for arc angles
arcLabels = {'Output η1 (red)', 'Coupler η2 (green)', 'Input η3 (blue)', 'Base η4'};
for i = 1:4
    uicontrol('Style', 'text', 'Parent', f, ...
        'Position', [20, 750-30*i, 100, 20], 'String', arcLabels{i}, ...
        'HorizontalAlignment', 'left');
    edits(i) = uicontrol('Style', 'edit', 'Parent', f, ...
        'Position', [120, 755-30*i, 60, 20], 'String', num2str(geometry(i)), ...
        'Callback', @(~,~) updatePlot());
end

% Colors for arcs and cylinders
colours = {'r', 'g', 'b' , 'k'};

% Mode & Configuration Controls
uicontrol('Style','text','Position',[10 600 100 20],'String','Mode');
mode_menu = uicontrol('Style','popupmenu', 'String', {'Direct','Inverse'}, ...
    'Position',[110 600 100 20], 'Callback', @modeChanged);

toggle_btn = uicontrol('Style','togglebutton','String','Elbow: Up', ...
    'Position',[10 570 200 25],'Callback',@toggleConfig);

show_both_checkbox = uicontrol('Style','checkbox','String','Show both configs', ...
    'Position',[10 540 200 25],'Value',0,'Callback',@updatePlot);

% Create sliders and labels for joint angles
sliders(1) = makeSlider('θ (deg)',10,490,-180,180,theta,@updatePlot);
values(1)    = uicontrol('Style','text','Position',[220 490 60 20], ...
    'String',sprintf('%.1f°',theta));

% End-Effector Sliders (Inverse Mode)
sliders(2)  = makeSlider('α (deg)',10,430,-180,180,alpha,@updatePlot);
values(2)     = uicontrol('Style','text','Position',[220 430 60 20], ...
    'String',sprintf('%.1f°',alpha));

set(sliders(2),'Enable','off'); set(values(2),'Enable','off');

% Animate toggle button under sliders
hAnimate = uicontrol('Style', 'togglebutton', 'Parent', f, ...
    'Position', [20, 370, 100, 30], 'String', 'Animate', ...
    'Callback', @toggleAnimation);

% Text display for EE info
info_text = uicontrol('Style','text','Position',[10 50 350 150], ...
    'HorizontalAlignment','left','FontSize',11,'FontName','Courier New');


% Initial plot
updatePlot();

% ---------------------------------------------------------------------
% Nested callback functions and helpers
% ---------------------------------------------------------------------

    function toggleConfig(~,~)
        % TOGGLECONFIG - flip between elbow-down and elbow-up configurations
        configState = -configState;
        % Update button label
        set(toggle_btn,'String',sprintf('Elbow: %s', ternary(configState==1,'Up','Down')));
        updatePlot();
    end

    function modeChanged(src, ~)
        % Handle switching between direct (1) and inverse (2) modes
        modeState = get(src, 'Value');
        if modeState == 1
            set(sliders(1),'Enable','on'); set(values(1),'Enable','on');
            set(sliders(2),'Enable','off'); set(values(2),'Enable','off');
            set(sliders(1),'Value',theta); set(values(1),'String',sprintf('%.1f°',theta)); 
        else
            set(sliders(1),'Enable','off'); set(values(1),'Enable','off');
            set(sliders(2),'Enable','on'); set(values(2),'Enable','on');
            set(sliders(2),'Value',alpha); set(values(2),'String',sprintf('%.1f°',alpha));
        end
        updatePlot();
    end
    
    function toggleAnimation(~,~)
        % TOGGLEANIMATION - Toggle animation on/off
        animating = ~animating;
        if animating
            set(hAnimate, 'String', 'Stop');
            runAnimation();
        else
            set(hAnimate, 'String', 'Animate');
        end
    end

    function updatePlot(~,~)
        % Redraws linkage based on current UI values and mode
        cla(ax); hold(ax,'on');
        geometry = arrayfun(@(e) str2double(get(e,'String')),edits);
        if modeState==1
            % Forward kinematics
            theta=get(sliders(1),'Value'); set(values(1),'String',sprintf('%.1f°',theta));
            thetatemp=[theta;NaN];
            [alphatemp,ok,P1,P2]=fourbar_spherical_direct_kinematics(geometry,theta);
            alpha=alphatemp(configState*0.5+1.5);
            if ~ok, text(0,0,'Unreachable','Parent',ax,'Color','r','FontSize',14); return; end
            if configState==-1; drawLinkage(P1); else drawLinkage(P2); end
            if get(show_both_checkbox,'Value')
                if configState==-1; drawLinkage(P2,'--'); else drawLinkage(P1,'--'); end
            end
        else
            % Inverse kinematics
            alpha=get(sliders(2),'Value'); set(values(2),'String',sprintf('%.1f°',alpha));
            alphatemp=[alpha;NaN];
            [thetatemp,ok,P1,P2]=fourbar_spherical_inverse_kinematics(geometry,alpha);
            theta=thetatemp(configState*0.5+1.5);
            if ~ok, text(0,0,'Unreachable','Parent',ax,'Color','r','FontSize',14); return; end
            if configState==-1; drawLinkage(P1); else drawLinkage(P2); end
            if get(show_both_checkbox,'Value')
               if configState==-1; drawLinkage(P2,'--'); else drawLinkage(P1,'--'); end
            end
        end
        if modeState==2, alpha=get(sliders(2),'Value'); end;
        info_str = sprintf([...
                 'θ1=%.1f°, θ2=%.1f°\n', ...
                 'α1=%.1f°, α2=%.1f°'], ...
                 thetatemp(1),thetatemp(2),alphatemp(1),alphatemp(2));
        set(info_text,'String',info_str);
    end

    function drawLinkage(P,style)
        % Plots links and joints
        if nargin==1, style='-'; axis(ax,[-axis_lim axis_lim -axis_lim axis_lim -axis_lim axis_lim]); %view(ax,45,30);
            [sX,sY,sZ]=sphere(20); surf(ax,sX,sY,sZ,'FaceColor',[0.8 0.8 0.9],'FaceAlpha',0.1,'EdgeColor','none'); mesh(ax,sX,sY,sZ,'EdgeColor',[0.6 0.6 0.6],'FaceColor','none','LineWidth',0.5);
        end

        for k=1:3, p0=P(:,k); p1=P(:,k+1); ang=acos(dot(p0,p1)/(norm(p0)*norm(p1))); t=linspace(0,1,50);
            arc=(sin((1-t)*ang).*p0+sin(t*ang).*p1)/sin(ang);
            plot3(ax,arc(1,:),arc(2,:),arc(3,:),'Color',colours{k},'LineWidth',2,'LineStyle',style);
        end
        for k=1:4, drawCylinder(ax,P(:,k),0.03,colours{k}); end
        
        drawnow;
    end

    function runAnimation(~,~)
        % Continuously animate linkage
        geometry = arrayfun(@(e) str2double(get(e,'String')),edits);

        range = computeFeasibleAngleRange(geometry, modeState);
        if isempty(range)
            return;  % No valid motion if empty range
        end

        angles = linspace(range(1), range(2), 200);  % Smooth animation steps

        while animating && ishandle(f)
            for val = angles
                if ~animating || ~ishandle(f)
                    break;
                end
                if modeState==1
                set(values(1), 'String', num2str(val));
                set(sliders(1), 'Value', val);
                else
                set(values(2), 'String', num2str(val));
                set(sliders(2), 'Value', val);
                end
                updatePlot();
                drawnow;
                pause(0.02);
            end
        end
    end

    function drawCylinder(ax,v,radius,colour)
        %Draw cylinders at joint location
        v_unit=v/norm(v); startF=0.9; endF=1.1; Lc=endF-startF;
        [Xc,Yc,Zc]=cylinder(radius,20); Zc=Zc*Lc+startF;
        z_axis=[0;0;1]; axis_rot=cross(z_axis,v_unit);
        if norm(axis_rot)<1e-6, R=eye(3);
        else axis_rot=axis_rot/norm(axis_rot);
            ang=acos(dot(z_axis,v_unit)); K=[0 -axis_rot(3) axis_rot(2); axis_rot(3) 0 -axis_rot(1); -axis_rot(2) axis_rot(1) 0];
            R=eye(3)+sin(ang)*K+(1-cos(ang))*(K^2);
        end
        [nr,nc]=size(Xc); XYZ=[Xc(:)';Yc(:)';Zc(:)']; pts=R*XYZ;
        Xct=reshape(pts(1,:),nr,nc); Yct=reshape(pts(2,:),nr,nc); Zct=reshape(pts(3,:),nr,nc);
        surf(ax,Xct,Yct,Zct,'FaceColor',colour,'EdgeColor','none');
    end
end

% Helper: draw slider control

function s=makeSlider(label,x,y,minV,maxV,init,cb)
% MAKE SLIDER - Creates a labeled slider control
uicontrol('Style','text','Position',[x y+20 100 20],'String',label);
s=uicontrol('Style','slider','Position',[x y 200 20],'Min',minV,'Max',maxV,'Value',init,'SliderStep',[0.01 0.1],'Callback',cb);
end


% Helper: compute workspace of the linkage for direct and inverse modes

function range = computeFeasibleAngleRange(geometry, mode)
% COMPUTEFEASIBLERANGER - Determine valid motion range of input/output
angle_vals = linspace(-pi, pi, 361);
valid = false(size(angle_vals));
for i = 1:length(angle_vals)
    try
        if mode == 1
            fourbar_spherical_direct_kinematics(geometry, angle_vals(i));
        else
            fourbar_spherical_inverse_kinematics(geometry, angle_vals(i));
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

function out=ternary(cond,a,b)
% TERNARY - Conditional helper: returns a if cond is true, else b
if cond, out=a; else out=b; end
end
