function rrr_spherical_gui()
% % RRR_SPHERICAL_GUI - Interactive GUI for a Spherical RRR Serial Linkage
% %
% % OBJECTIVE:
% %   Launch a MATLAB graphical user interface to analyze and visualize
% %   the kinematics of a spherical 3R serial robot (RRR manipulator).
% %   Supports both forward (direct) and inverse kinematics, elbow-up/down
% %   configuration switching, and animated motion.
% %
% % INPUTS:
% %   None. All parameters are set via UI controls within the GUI.
% %
% % OUTPUTS:
% %   Visual feedback on the figure window. No MATLAB outputs.
% %
% % USAGE EXAMPLE:
% %   >> rrr_spherical_gui
% %   Opens the GUI with default link lengths and joint values.
% %
% % BY:
% % Prof. Lionel Birglen
% % Polytechnique Montreal, 2025
% % Last Update: 2025/05/23
% % Contact: lionel.birglen@polymtl.ca
% %
% % Code provided under GNU Affero General Public License v3.0


% ---------------------------------------------------------------------
% INITIALIZATION
% ---------------------------------------------------------------------

% Create the main figure
f = figure('Name', '3R Spherical Linkage', 'NumberTitle', 'off', ...
    'Position', [100, 100, 1100, 750]);

% Default joint angles
theta = [0, 0, 0];
eul = [0, 0, 0];

% State variables for current configuration and mode
configState     = -1;
modeState       = 1;

% Create axes for plotting
ax = axes('Parent', f, 'Position', [0.4 0.2 0.55 0.75]);
axis_lim = 1.2;
axis(ax, [-axis_lim axis_lim -axis_lim axis_lim -axis_lim axis_lim]);
axis(ax, 'equal'); grid(ax, 'on'); view(ax, 45, 30);
xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
title(ax, '3R Spherical Serial Linkage'); hold(ax, 'on');

% ---------------------------------------------------------------------
% GUI COMPONENTS
% ---------------------------------------------------------------------

% Input boxes for arc angles
arcLabels = {'\alpha_1', '\alpha_2', '\alpha_3'};
for i = 1:3
    uicontrol('Style', 'text', 'Parent', f, ...
        'Position', [20, 750-30*i, 60, 20], 'String', arcLabels{i}, ...
        'HorizontalAlignment', 'left');
    edits(i) = uicontrol('Style', 'edit', 'Parent', f, ...
        'Position', [90, 755-30*i, 60, 20], 'String', '45', ...
        'Callback', @(~,~) updatePlot());
end

% Colors for arcs and cylinders
colours = {'r', 'g', 'b'};

% Mode & Configuration Controls
uicontrol('Style','text','Position',[10 630 100 20],'String','Mode');
mode_menu = uicontrol('Style','popupmenu', 'String', {'Direct','Inverse'}, ...
    'Position',[110 630 100 20], 'Callback', @modeChanged);

toggle_btn = uicontrol('Style','togglebutton','String','Elbow: Up', ...
    'Position',[10 600 200 25],'Callback',@toggleConfig);

show_both_checkbox = uicontrol('Style','checkbox','String','Show both configs', ...
    'Position',[10 570 200 25],'Value',0,'Callback',@updatePlot);

set(show_both_checkbox,'Enable','off');
set(toggle_btn,'Enable','off');

% Create sliders and labels for joint angles
sliders(1) = makeSlider('θ₁ (deg)',10,530,-180,180,theta(1),@updatePlot);
values(1)    = uicontrol('Style','text','Position',[220 530 60 20], ...
    'String',sprintf('%.1f°',theta(1)));
sliders(2) = makeSlider('θ₂ (deg)',10,480,-180,180,theta(2),@updatePlot);
values(2)    = uicontrol('Style','text','Position',[220 480 60 20], ...
    'String',sprintf('%.1f°',theta(2)));
sliders(3) = makeSlider('θ₃ (deg)',10,430,-180,180,theta(3),@updatePlot);
values(3)    = uicontrol('Style','text','Position',[220 430 60 20], ...
    'String',sprintf('%.1f°',theta(3)));

% End-Effector Sliders (Inverse Mode)
sliders(4)  = makeSlider('Yaw (deg)',10,380,-180,180,eul(1),@updatePlot);
values(4)     = uicontrol('Style','text','Position',[220 380 60 20], ...
    'String',sprintf('%.1f°',eul(1)));
sliders(5)  = makeSlider('Pitch (deg)',10,330,-180,180,eul(2),@updatePlot);
values(5)     = uicontrol('Style','text','Position',[220 330 60 20], ...
    'String',sprintf('%.1f°',eul(2)));
sliders(6)  = makeSlider('Roll (deg)',10,280,-180,180,eul(3),@updatePlot);
values(6)    = uicontrol('Style','text','Position',[220 280 60 20], ...
    'String',sprintf('%.1f°',eul(3)));

% Animate toggle button under sliders
hAnimate = uicontrol('Style', 'togglebutton', 'Parent', f, ...
    'Position', [20, 230, 100, 30], 'String', 'Animate', ...
    'Callback', @(~,~) animateFcn());

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
            set(sliders(1:3),'Enable','on'); set(values(1:3),'Enable','on');
            set(sliders(4:6),'Enable','off'); set(values(4:6),'Enable','off');
            set(show_both_checkbox,'Enable','off'); set(toggle_btn,'Enable','off');
            for k=1:3, set(sliders(k),'Value',theta(k)); set(values(k),'String',sprintf('%.1f°',theta(k))); end
        else
            set(sliders(1:3),'Enable','off'); set(values(1:3),'Enable','off');
            set(sliders(4:6),'Enable','on'); set(values(4:6),'Enable','on');
            set(show_both_checkbox,'Enable','on'); set(toggle_btn,'Enable','on');
            for k=1:3, set(sliders(k+3),'Value',eul(k)); set(values(k+3),'String',sprintf('%.1f°',eul(k))); end
        end
        updatePlot();
    end

    function updatePlot(~,~)
        % Redraws linkage based on current UI values and mode
        cla(ax); hold(ax,'on');
        geometry = arrayfun(@(e) str2double(get(e,'String')),edits);
        if modeState==1
            % Forward kinematics
            for k=1:3, theta(k)=get(sliders(k),'Value'); set(values(k),'String',sprintf('%.1f°',theta(k))); end
            [P,R_full,eul]=rrr_spherical_direct_kinematics(geometry,theta);
            drawLinkage(P);
        else
            % Inverse kinematics
            for k=1:3, eul(k)=get(sliders(k+3),'Value'); set(values(k+3),'String',sprintf('%.1f°',eul(k))); end
            [thetatemp,ok]=rrr_spherical_inverse_kinematics(geometry,eul);
            theta=thetatemp(configState*0.5+1.5,1:3);
            if ~ok, text(0,0,'Unreachable','Parent',ax,'Color','r','FontSize',14); return; end
            [P,R_full,~]=rrr_spherical_direct_kinematics(geometry,theta);
            drawLinkage(P);
            if get(show_both_checkbox,'Value')
                theta2=thetatemp(-configState*0.5+1.5,1:3);
                [P2,~,~]=rrr_spherical_direct_kinematics(geometry,theta2);
                drawLinkage(P2,'--');
            end
        end
        if modeState==2, for k=1:3, eul(k)=get(sliders(k+3),'Value'); end; end
        R=R_full;
        info_str = sprintf([...
            'X=%.2f, Y=%.2f, Z=%.2f\n', ...
            'θ1=%.1f°, θ2=%.1f°, θ3=%.1f°\n', ...
            'Yaw=%.1f°, Pitch=%.1f°, Roll=%.1f°\n', ...
            'R = [%.3f %.3f %.3f\n', ...
            '     %.3f %.3f %.3f\n', ...
            '     %.3f %.3f %.3f]'], ...
            P(1,4),P(2,4),P(3,4),theta(1),theta(2),theta(3),eul(1),eul(2),eul(3),...
            R(1,1),R(1,2),R(1,3),R(2,1),R(2,2),R(2,3),R(3,1),R(3,2),R(3,3));
        set(info_text,'String',info_str);
    end

    function drawLinkage(P,style)
        % Plots links and joints
        if nargin==1, style='-'; axis(ax,[-axis_lim axis_lim -axis_lim axis_lim -axis_lim axis_lim]); view(ax,45,30);
            [sX,sY,sZ]=sphere(20); surf(ax,sX,sY,sZ,'FaceColor',[0.8 0.8 0.9],'FaceAlpha',0.1,'EdgeColor','none'); mesh(ax,sX,sY,sZ,'EdgeColor',[0.6 0.6 0.6],'FaceColor','none','LineWidth',0.5);
        end
        for k=1:3, p0=P(:,k); p1=P(:,k+1); ang=acos(dot(p0,p1)/(norm(p0)*norm(p1))); t=linspace(0,1,50);
            arc=(sin((1-t)*ang).*p0+sin(t*ang).*p1)/sin(ang);
            plot3(ax,arc(1,:),arc(2,:),arc(3,:),'Color',colours{k},'LineWidth',2,'LineStyle',style);
        end
        for k=1:3, drawCylinder(ax,P(:,k),0.03,colours{k}); end
        if nargin==1, [Xs,Ys,Zs]=sphere(20); surf(ax,0.05*Xs+P(1,4),0.05*Ys+P(2,4),0.05*Zs+P(3,4),'FaceColor','k','EdgeColor','none'); end
        drawnow;
    end

    function animateFcn(~,~)
        % Continuously animate linkage
        if get(hAnimate,'Value')==1
            set(hAnimate,'String','Stop');
            % Direct bounce rates
            v=[1,2,3];
            % step size per frame (deg)
            step=5;
            % capture start Euler
            yaw0=eul(1); pitch0=eul(2); roll0=eul(3);
            % build paths
            yawPath=[linspace(yaw0,180,ceil((180-yaw0)/step)), linspace(180,-180,ceil(360/step)), linspace(-180,yaw0,ceil((yaw0+180)/step))];
            pitchPath=[linspace(pitch0,180,ceil((180-pitch0)/step)), linspace(180,-180,ceil(360/step)), linspace(-180,pitch0,ceil((pitch0+180)/step))];
            rollPath=[linspace(roll0,180,ceil((180-roll0)/step)), linspace(180,-180,ceil(360/step)), linspace(-180,roll0,ceil((roll0+180)/step))];
            axisIdx=1;
            while get(hAnimate,'Value')==1
                modeState=get(mode_menu,'Value');
                if modeState==1
                    % Direct ping-pong
                    for k=1:3
                        theta(k)=theta(k)+v(k);
                        if theta(k)>=180, theta(k)=180; v(k)=-v(k);
                        elseif theta(k)<=-180, theta(k)=-180; v(k)=-v(k); end
                        set(sliders(k),'Value',theta(k)); set(values(k),'String',sprintf('%.1f°',theta(k)));
                    end
                else
                    % Inverse sequential full-range scan
                    switch axisIdx
                        case 1, path=yawPath; idxOffset=0;
                        case 2, path=pitchPath; idxOffset=0;
                        case 3, path=rollPath; idxOffset=0;
                    end
                    for val=path
                        if get(hAnimate,'Value')==0, break; end
                        switch axisIdx
                            case 1, eul(1)=val;
                            case 2, eul(2)=val;
                            case 3, eul(3)=val;
                        end
                        set(sliders(axisIdx+3),'Value',eul(axisIdx));
                        set(values(axisIdx+3),'String',sprintf('%.1f°',eul(axisIdx)));
                        updatePlot(); pause(0.02);
                    end
                    if get(hAnimate,'Value')==0, break; end
                    axisIdx=mod(axisIdx,3)+1;
                end
                updatePlot(); pause(0.02);
            end
            set(hAnimate,'String','Animate');
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

% Helper: misc

function out=ternary(cond,a,b)
% TERNARY - Conditional helper: returns a if cond is true, else b
if cond, out=a; else out=b; end
end
