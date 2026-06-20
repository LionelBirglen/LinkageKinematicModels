function stephensonIII_gui
% STEPHENSONIII_GUI  Interactive GUI for Stephenson III six-bar linkage
%
%   This GUI lets you:
%     • Enter geometry ([OA, Bx, By, OC, CD, DA, BE, EM, MP, eta])
%     • Choose Direct or Inverse mode
%     • In Direct mode: Slide θ_O (input crank angle) and see the linkage update
%     • In Inverse mode: Type θB (angle of BE to x-axis), click “Solve,” and see all solutions
%   Requirements: stephensonIII_direct_kinematics.m and stephensonIII_inverse_kinematics.m
%   must be on the MATLAB path.
%   To run: save as stephensonIII_gui.m and type >> stephensonIII_gui

% --- Create main figure
hFig = figure('Name','Stephenson III Linkage GUI', ...
    'NumberTitle','off', 'Position',[200 200 900 620]);

% --------- Geometry Panel --------------
geoLabels = {'OA','Bx','By','OC','CD','DA','BE','EM','DM','MP','eta','delta'};
geoUnits = {'','', '', '', '', '', '', '', '', '', 'deg', 'deg'};
geoDefaults = {'40','70','30','50','20','50','30','30','-30','20','30','60'};

uipanel('Title','Geometry','FontSize',10,'Position',[0.02 0.71 0.32 0.29]);
geoEd = gobjects(1,12);
for k = 1:6
    uicontrol('Style','text','Units','normalized','Position',[0.03 0.96-0.04*k 0.03 0.035],...
        'String',geoLabels{k},'HorizontalAlignment','right');
    geoEd(k) = uicontrol('Style','edit','Units','normalized','Position',[0.07 0.97-0.04*k 0.06 0.035],...
        'String',geoDefaults{k},'Callback',@updatePlot);
end
for k = 7:12
    uicontrol('Style','text','Units','normalized','Position',[0.19 0.96-0.04*(k-6) 0.03 0.035],...
        'String',geoLabels{k},'HorizontalAlignment','right');
    geoEd(k) = uicontrol('Style','edit','Units','normalized','Position',[0.23 0.97-0.04*(k-6) 0.06 0.035],...
        'String',geoDefaults{k},'Callback',@updatePlot);
end

% --------- Mode Selection --------------
uipanel('Title','Mode','FontSize',10,'Position',[0.02 0.6 0.32 0.1]);
modeBtn = uibuttongroup('Units','normalized','Position',[0.02 0.61 0.32 0.06],'SelectionChangedFcn',@modeChanged);
rad1 = uicontrol(modeBtn,'Style','radiobutton','String','Direct','Units','normalized','Position',[0.05 0.1 0.4 0.8]);
rad2 = uicontrol(modeBtn,'Style','radiobutton','String','Inverse','Units','normalized','Position',[0.50 0.1 0.4 0.8]);
modeBtn.SelectedObject = rad1;

% --------- Direct Kinematics Controls --------------
uipanel('Title','Direct Kinematics','FontSize',10,'Position',[0.02 0.52 0.32 0.08]);
uicontrol('Style','text','Units','normalized','Position',[0.03 0.525 0.02 0.035],...
    'String','θO','HorizontalAlignment','right');
thetaO_slider = uicontrol('Style','slider','Units','normalized','Position',[0.06 0.53 0.22 0.035],...
    'Min',-180,'Max',180,'Value',90,'SliderStep',[1/360 0.1],'Callback',@updatePlot);
thetaO_edit = uicontrol('Style','edit','Units','normalized','Position',[0.29 0.53 0.04 0.035],...
    'String','30','Callback',@thetaO_edit_callback);

% --------- Inverse Kinematics Controls --------------
uipanel('Title','Inverse Kinematics','FontSize',10,'Position',[0.02 0.44 0.32 0.08]);
uicontrol('Style','text','Units','normalized','Position',[0.03 0.445 0.02 0.035],...
    'String','θB','HorizontalAlignment','right');
thetaB_slider = uicontrol('Style','slider','Units','normalized','Position',[0.06 0.45 0.22 0.035],...
    'Min',-180,'Max',180,'Value',69,'SliderStep',[1/360 0.1],'Callback',@updatePlot);
thetaB_edit = uicontrol('Style','edit','Units','normalized','Position',[0.29 0.45 0.04 0.035],...
    'String','30','Callback',@thetaB_edit_callback);

% Checkbox:
solsPanel = uipanel('Title','Display solutions:','FontSize',10, ...
    'Position',[0.02 0.34 0.32 0.10], ...
    'Visible','on');
for i=1:4
    sols_checkbox(i) = uicontrol('Parent',solsPanel,'Units','normalized','Style','checkbox','Position',[0.2*i 0.5 0.7 0.6], ...
        'String',num2str(i),'Value',0,'Callback',@updatePlot);
end
for i=5:8
    sols_checkbox(i) = uicontrol('Parent',solsPanel,'Units','normalized','Style','checkbox','Position',[0.2*(i-4) 0.01 0.7 0.6], ...
        'String',num2str(i),'Value',0,'Callback',@updatePlot);
end
set(sols_checkbox(2),'Value',1);

% Animate button
animate_btn = uicontrol('Style','pushbutton','String','Animate', ...
    'Position',[19 175 289 30], 'Callback',@toggleAnimation);

% --------- Plot Axes and Solution Info --------------
axesPanel = uipanel('Title','Linkage Plot','FontSize',10,'Position',[0.36 0.09 0.62 0.87]);
ax = axes('Parent',axesPanel,'Units','normalized','Position',[0.06 0.09 0.88 0.85]);
solTxt = uicontrol('Style','text','Units','normalized','Position',[0.02 0.07 0.33 0.2],...
    'FontSize',10,'HorizontalAlignment','left','String','');
xlim([-70 70]*1.2);
ylim([-70 70]*1.2);

% --------- Main logic callbacks --------------

    function geo = getGeometry()
        geo = zeros(1,12);
        for i=1:12
            geo(i) = str2double(get(geoEd(i),'String'));
        end
    end

    function updatePlot(~,~)
        info_str='';
        geo = getGeometry();
        modeStr = modeBtn.SelectedObject.String;
        if strcmp(modeStr,'Direct')
            thetaO = get(thetaO_slider,'Value');
            set(thetaO_edit,'String',num2str(thetaO,'%.1f'));
            sols = stephensonIII_direct_kinematics(geo, thetaO);
        else
            thetaB = get(thetaB_slider,'Value');
            set(thetaB_edit,'String',num2str(thetaB,'%.1f'));
            sols = stephensonIII_inverse_kinematics(geo, thetaB);
        end
        cla(ax);
        hold(ax,'on');
        plotSolutions(ax, sols);
        xlim([-70 70]*1.2);
        ylim([-70 70]*1.2);
        if ~isempty(sols) 
    
            thetaO_disp = get(thetaO_slider,'Value');
            thetaB_disp = get(thetaB_slider,'Value');

            for i=1:numel(sols)
                info_str = append(info_str, ' Sol ',num2str(i),sprintf([ ' X=%.2f, Y=%.2f, θO=%.1f°, θB=%.1f°\n'],sols(i).Positions.P(1),sols(i).Positions.P(2),sols(i).Angles.thetaO,sols(i).Angles.thetaB));
            end
            set(solTxt,'String',info_str);
        else
            solTxt.String = 'No valid solution.';
        end
        hold(ax,'off');
    end

    function s = localnumstr(x)
        if isnan(x)
            s = '--';
        else
            s = num2str(x,'%.1f');
        end
    end

    function thetaO_edit_callback(~,~)
        val = str2double(get(thetaO_edit,'String'));
        val = max(min(val,180),-180);
        set(thetaO_slider,'Value',val);
        updatePlot();
    end

    function thetaB_edit_callback(~,~)
        val = str2double(get(thetaB_edit,'String'));
        val = max(min(val,180),-180);
        set(thetaB_slider,'Value',val);
        updatePlot();
    end

    function modeChanged(~,~)
        if strcmp(modeBtn.SelectedObject.String,'Direct')
            set([thetaO_slider thetaO_edit],'Enable','on');
            set([thetaB_slider thetaB_edit],'Enable','off');
        else
            set([thetaO_slider thetaO_edit],'Enable','off');
            set([thetaB_slider thetaB_edit],'Enable','on');
        end
        updatePlot();
    end

% --- Draw linkage only for solutions whose checkbox is checked ---
    function plotSolutions(ax, sols)
        if isempty(sols) 
            text(ax,0,0,'No valid solution','FontSize',12,'HorizontalAlignment','center');
            return
        end
        % Assign distinct colors using lines(7)
        colors = lines(8); % [1]=OAB, [2]=CDM, [3]=EMP, [4]=OC, [5]=AD, [6]=BE, [7]=extra
        % color_OAB = colors(1,:);
        % color_CDM = colors(2,:);
        % color_EMP = colors(3,:);
        % color_OC  = colors(4,:);
        % color_AD  = colors(5,:);
        % color_BE  = colors(6,:);

        numToPlot = min(length(sols),8); % up to 8 checkboxes
        for k = 1:numToPlot

            if get(sols_checkbox(k), 'Value')
                pos = sols(k).Positions;
                plotBodyOAB(ax, pos, [0 0 1], 1);
                plotBodyCDM(ax, pos, colors(k,:), 1);
                plotBodyEMP(ax, pos, colors(k,:), 1);
                plotExtraLinks(ax, pos, colors(k,:), colors(k,:), colors(k,:), 1);
            end
        end
    end

    function plotBodyOAB(ax, pos, color, isMain)
        % Fill triangle OAB
        pts = [pos.O pos.A pos.B];
        fill(ax, pts(1,:), pts(2,:), color, 'FaceAlpha',0.5, 'EdgeColor','none');
        lw = 3*(isMain) + 1.5*(~isMain);
        mksz = 9;
        % Draw OA, AB, BO
        plot(ax,[pos.O(1) pos.A(1)], [pos.O(2) pos.A(2)], '-', 'Color',color, 'LineWidth',lw);
        plot(ax,[pos.A(1) pos.B(1)], [pos.A(2) pos.B(2)], '-', 'Color',color, 'LineWidth',lw);
        plot(ax,[pos.B(1) pos.O(1)], [pos.B(2) pos.O(2)], '-', 'Color',color, 'LineWidth',lw);
        % Joints
        plot(ax,pos.O(1),pos.O(2),'ko','MarkerFaceColor',color,'MarkerSize',mksz);
        plot(ax,pos.A(1),pos.A(2),'ko','MarkerFaceColor',color,'MarkerSize',mksz);
        plot(ax,pos.B(1),pos.B(2),'ko','MarkerFaceColor',color,'MarkerSize',mksz);
    end

    function plotBodyCDM(ax, pos, color, isMain)
        % Fill triangle CDM
        pts = [pos.C pos.D pos.M];
        fill(ax, pts(1,:), pts(2,:), color, 'FaceAlpha',0.5, 'EdgeColor','none');
        lw = 3*(isMain) + 1.5*(~isMain);
        mksz = 9;
        % Draw CD, DM, MC (CM)
        plot(ax,[pos.C(1) pos.D(1)], [pos.C(2) pos.D(2)], '-', 'Color',color, 'LineWidth',lw);
        plot(ax,[pos.D(1) pos.M(1)], [pos.D(2) pos.M(2)], '-', 'Color',color, 'LineWidth',lw);
        plot(ax,[pos.M(1) pos.C(1)], [pos.M(2) pos.C(2)], '-', 'Color',color, 'LineWidth',lw);
        % Joints
        plot(ax,pos.C(1),pos.C(2),'ko','MarkerFaceColor',color,'MarkerSize',mksz);
        plot(ax,pos.D(1),pos.D(2),'ko','MarkerFaceColor',color,'MarkerSize',mksz);
        plot(ax,pos.M(1),pos.M(2),'ko','MarkerFaceColor',color,'MarkerSize',mksz);
    end

    function plotBodyEMP(ax, pos, color, isMain)
        % Fill triangle EMP
        pts = [pos.E pos.M pos.P];
        fill(ax, pts(1,:), pts(2,:), color, 'FaceAlpha',0.5, 'EdgeColor','none');
        lw = 3*(isMain) + 1.5*(~isMain);
        mksz = 9;
        % Draw EM, MP, PE
        plot(ax,[pos.E(1) pos.M(1)], [pos.E(2) pos.M(2)], '-', 'Color',color, 'LineWidth',lw);
        plot(ax,[pos.M(1) pos.P(1)], [pos.M(2) pos.P(2)], '-', 'Color',color, 'LineWidth',lw);
        plot(ax,[pos.P(1) pos.E(1)], [pos.P(2) pos.E(2)], '-', 'Color',color, 'LineWidth',lw);
        % Joints
        plot(ax,pos.E(1),pos.E(2),'ko','MarkerFaceColor',color,'MarkerSize',mksz);
        plot(ax,pos.M(1),pos.M(2),'ko','MarkerFaceColor',color,'MarkerSize',mksz);
        plot(ax,pos.P(1),pos.P(2),'kx','MarkerFaceColor',color,'MarkerSize',mksz, 'LineWidth',2);
    end

    function plotExtraLinks(ax, pos, color_OC, color_AD, color_BE, isMain)
        % OC, AD, BE (not part of triangle faces)
        lw = 3*(isMain) + 1.5*(~isMain);
        % OC (O to C)
        plot(ax, [pos.O(1) pos.C(1)], [pos.O(2) pos.C(2)], '-', 'Color', color_OC, 'LineWidth', lw);
        % AD (A to D)
        plot(ax, [pos.A(1) pos.D(1)], [pos.A(2) pos.D(2)], '-', 'Color', color_AD, 'LineWidth', lw);
        % BE (B to E)
        plot(ax, [pos.B(1) pos.E(1)], [pos.B(2) pos.E(2)], '-', 'Color', color_BE, 'LineWidth', lw);
        % Label all points only once (when main solution)
        if isMain
            labelpts = {'O','A','B','C','D','E','M','P'};
            for lp = 1:numel(labelpts)
                p = pos.(labelpts{lp});
                text(ax,p(1),p(2),['  ',labelpts{lp}],'FontWeight','bold','FontSize',11);
            end
            axis(ax,'equal');
            grid(ax,'on');
            xlabel(ax,'x');
            ylabel(ax,'y');
        end
    end

modeChanged();

end
