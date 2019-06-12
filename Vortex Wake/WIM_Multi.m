%function [Vtin,Wtin] = WIM(blade,flow,oper,gamma,alpha)
%Wake Interference Model V.5
%Calculate the induced velocity caused by rotor/wake/body interference in a
%rotorcraft/UAV. Edited for BEMT code.

%plotting:      'on' or 'off'
%wake.type:     'helix' or 'ring'

% qvr_pts: points on all rotors in which velocities are induced
% vi: velocities induced by single rotor wake structure on all qvr_pts

% COORDINATE SYSTEM:
% Coordinate system is wrt to centre of vehicle (symmetric axis of rotors).
% X is stern to bow (roll axis, positive forward towards bow i.e. forward flight direction), 
% Y is port to starboard (pitch axis, positive towards starboard i.e. to the right )

% Methodology:
% 1. Calculates skew angle, element spacing, circulation
% 2. Call custom function to generate Qpoints and vortex segment structure (depending on formulation chosen)
% 3. Calculates induced velocity at Qpoints using Biot Savart law
% 4. Plotting = [x1 x2 x3 x4 x5]. 1x5 vector of char 'on'/'off'

% To replace/add new wake structure, create new "case" with custom wake function
% (i.e. deformed helix, etc)

function [vi_WIM,qvr_pts] = WIM_Multi(flow,oper,rotor,wake,perf,plotting)
%% [1] Calculating wake paramters, handle data inputs
%----------------------------------------------------
        
for i = 1:rotor.num_rotors

        A(i)           =       pi.*(perf(i).R.^2);                                 % Area of Rotor
        gamma(i)       =       -perf(i).T./(perf(i).B*flow.rho*pi*(oper.rpm/60)*(perf(i).R^2));    % Circulation, gamma. Negative sign because WIMs are plotted CCW (right hand rule).

    % Vehicle orientation, rotor spacing
        if (strcmp(rotor.orientation,'square') == 1 && rem(rotor.num_rotors,2) == 0) || ((strcmp(rotor.orientation,'diamond') == 1 && rem(rotor.num_rotors,2) == 1)) % if (even # of rotors & square) or (odd # and diamond), rotor "1" is rotated CCW by 2*pi/rotor.num_rotors
            armAZIMUTH   = pi/rotor.num_rotors;
        else
            armAZIMUTH   = 0;
        end

    % Skew angle from initial thrust loading (assuming all rotors have same thrust loading. This can be changed easily though)
        chi(i)         =       atan((flow.V*cosd(flow.inflow_angle))/(flow.V*sind(flow.inflow_angle)+sqrt(perf(i).T/(2*pi*flow.rho*(perf(i).R^2))))); %calculate skew angle of wakes for influencing rotor

    % Vertical offset between ring wake elements    
        z(i)           =       (1/(perf(i).B*oper.rpm/60)).*(sqrt(perf(i).T/(2*flow.rho*pi*perf(i).R^2))+flow.V*sind(flow.inflow_angle));
       
    % Horizontal offset between ring wake elements   
        h(i)           =       tan(chi(i))*z(i);
end      
        
%---------------------------------------
%% [2] Create wake structure and Qpoints
%---------------------------------------

    % Create wake structure (vorticity line segments, rotor position, query points) depending on which wake formulation is chosen.
    switch lower(wake.type)
        case 'ring'
                    [WIMrotor,WIMwake] = RingWake_Multi(wake,rotor,perf(1).R,perf(1).mid_span,armAZIMUTH,perf(1).azimuth,z,h);
        case 'helix'
                    for i = 1:length(azimuth)
                    % Need iteration loop to twist WIMs. Can still use helix for
                    % adjacent wakes tho, when untwisted
                        [WIMrotor,WIMwake] = HelixWake(wake,rotor,R,mid_span,armAZIMUTH,azimuth(i),z,h);
                    end     
    end
    
% Calculate the induced velocity at each query point. Call Biot Savart Function
    % Wake structure same for every query point/azimuth for ring formulation
for i = 1:rotor.num_rotors

        q12_P(:,:,i) = Biot_Savart(vertcat(WIMrotor(1:rotor.num_rotors).Qpoints),...
                      vertcat(WIMwake(i).s1),...
                      vertcat(WIMwake(i).s2),...
                      gamma(i));       

    % Set velocities for self induced velocities to zero (adjacent only)
         Qpoints_count = length(WIMrotor(1).Qpoints);
         q12_P((i-1)*Qpoints_count+1:i*Qpoints_count,1:3,i) = 0;
    
    % Induced velocity
        vi_WIM(:,:,i) = [q12_P(:,1,i),q12_P(:,2,i),q12_P(:,3,i)];              
end
    
% Concatenate Qpoints into one variable for quiver plotting positions
    qvr_pts = vertcat(WIMrotor(1:rotor.num_rotors).Qpoints);

%% [3] Plotting preamble
%-----------------------    
% This section is designed to plot the rotor configuration at its AoA in
% the global coordinate sytem, in the vehicles coordinate system, contour
% plots of the configuration in the vehicle coordinate system and vi/v plots
% in the rotor's local coordinate system. Transformation matrices are introduced to
% transform the position of the rotor, vi vectors to their local coordinate
% system
%% Plot(1): Configuration and trailing wakes wrt VEHICLE coordinate system
if strcmpi(plotting(1),'on')
    
    figure(1)
    hold on
    for i = 1:rotor.num_rotors
        plot3([WIMrotor(i).centres(:,1),0],[WIMrotor(i).centres(:,2),0],[WIMrotor(i).centres(:,3),0],'-ok','LineWidth',2);
        
        % Rotor
        X1 = [WIMrotor(i).s1(:,1),WIMrotor(i).s2(:,1)];
        Y1 = [WIMrotor(i).s1(:,2),WIMrotor(i).s2(:,2)];
        Z1 = [WIMrotor(i).s1(:,3),WIMrotor(i).s2(:,3)];
        
        % Plot query points
        %plot3(WIMrotor(i).Qpoints(:,1),WIMrotor(i).Qpoints(:,2),WIMrotor(i).Qpoints(:,3))
        
        % Ring elements
        X2 = [WIMwake(i).s1(:,1),WIMwake(i).s2(:,1)];
        Y2 = [WIMwake(i).s1(:,2),WIMwake(i).s2(:,2)];
        Z2 = [WIMwake(i).s1(:,3),WIMwake(i).s2(:,3)];
        
        
        fill3(X1,Y1,Z1,'b');alpha(.5);
        % Plot rotor markers
        text(WIMrotor(i).centres(:,1),WIMrotor(i).centres(:,2),WIMrotor(i).centres(:,3), num2str(i),'Color','w','FontSize',16,'FontWeight','bold');
        
        % Plot wake elements
        for k = 1:wake.num_seg*wake.num_elements
            plot3(X2(k,:),Y2(k,:),Z2(k,:),'-k');
        end
    end
    
    xlabel('X-axis');
    ylabel('Y-axis');
    zlabel('Z-axis');
    daspect([max(daspect)*[1 1] 1]) % Set xy plane axis to be the same
    grid on;
    grid minor;
    zlim([-(wake.num_elements+1)*z sin(AoA_rad)*R*2]);
    arrow3([cos(AoA_rad) 0 sin(AoA_rad)],[0 0 0],'b2',1) % Freestream velocity
    axis equal
    hold off
end
%% Plot(2): Configuration and trailing wakes wrt GLOBAL coordinate system
if strcmpi(plotting(2),'on')
    
    % Convention for assigning rotor positioning:
    % 1. Assume rotor is in psi = 0 position (rear, along x axis)
    % 2. Apply rotor reference frame transformation matrices
    % 3. Rotate rotor(s) to desired position
    % 4. Create wake structure from orientation of rotor and skew angle
    % 5. Apply global rotation matrices for AoA and Roll to all points (including wake)
    
    
    
    %  Entire vehicle orientation rotation matrices
    % (Needs to be done AFTER completing UAV rotor config i.e. rotate hub, twist and cant rotations)
    % Rotation matrices for vehicle AOA (about y, CCW) and vehicle roll (about x, CCW)
    rotateROLL          =   [1 0 0; 0 cosd(rotor.roll) -sind(rotor.roll); 0 sind(rotor.roll) cosd(rotor.roll)]; % reversed rotation since x axis is "reversed" from overhead. Forward = +x
    rotateALPHA         =   [cosd(flow.inflow_angle) 0 -sind(flow.inflow_angle); 0 1 0; sind(flow.inflow_angle) 0 cosd(flow.inflow_angle)];
    
    
    
    figure(2)
    hold on
    for i = 1:rotor.num_rotors
        
        % Global transformations
        WIMwakeGLOBAL(i).s1       =   (WIMwake(i).s1)*rotateALPHA*rotateROLL;
        WIMwakeGLOBAL(i).s2       =   (WIMwake(i).s2)*rotateALPHA*rotateROLL;
        
        WIMrotorGLOBAL(i).centres =   WIMrotor(i).centres*rotateALPHA*rotateROLL;
        WIMrotorGLOBAL(i).s1      =   WIMrotor(i).s1*rotateALPHA*rotateROLL;
        WIMrotorGLOBAL(i).s2      =   WIMrotor(i).s2*rotateALPHA*rotateROLL;
        WIMrotorGLOBAL(i).Qpoints =   WIMrotor(i).Qpoints*rotateALPHA*rotateROLL;
        
        % Rotor
        X1 = [WIMrotorGLOBAL(i).s1(:,1),WIMrotorGLOBAL(i).s2(:,1)];
        Y1 = [WIMrotorGLOBAL(i).s1(:,2),WIMrotorGLOBAL(i).s2(:,2)];
        Z1 = [WIMrotorGLOBAL(i).s1(:,3),WIMrotorGLOBAL(i).s2(:,3)];
        
        % Ring elements
        X2 = [WIMwakeGLOBAL(i).s1(:,1),WIMwakeGLOBAL(i).s2(:,1)];
        Y2 = [WIMwakeGLOBAL(i).s1(:,2),WIMwakeGLOBAL(i).s2(:,2)];
        Z2 = [WIMwakeGLOBAL(i).s1(:,3),WIMwakeGLOBAL(i).s2(:,3)];
        
        % Plot rotors arms
        plot3([WIMrotorGLOBAL(i).centres(:,1),0],[WIMrotorGLOBAL(i).centres(:,2),0],[WIMrotorGLOBAL(i).centres(:,3),0],'-ok','LineWidth',2);
        
        % Plot rotor plane shading
        fill3(X1,Y1,Z1,'b');alpha(.5);
        
        % Plot rotor text markers
        text(WIMrotorGLOBAL(i).centres(:,1),WIMrotorGLOBAL(i).centres(:,2),WIMrotorGLOBAL(i).centres(:,3), num2str(i),'Color','w','FontSize',16,'FontWeight','bold');
        
        % Plot wake elements
        for k = 1:wake.num_seg*wake.num_elements
            plot3(X2(k,:),Y2(k,:),Z2(k,:),'-k');
        end
    end
    quiver3(0.5,0,0,-0.5,0,0,'LineWidth',3) % Freestream velocity
    
    % Plot query points
    %    plot3(WIMrotor(i).Qpoints(:,1),WIMrotor(i).Qpoints(:,2),WIMrotor(i).Qpoints(:,3))
    xlabel('X-axis');
    ylabel('Y-axis');
    zlabel('Z-axis');
    daspect([max(daspect)*[1 1] 1]) % Set xy plane axis to be the same
    grid on;
    grid minor;
    %zlim([-(wake.num_elements+1)*z sin(AoA_rad)*R*2]);
    arrow3([cos(AoA_rad) 0 sin(AoA_rad)],[0 0 0],'b2',1) % Freestream velocity
    axis equal
    hold off
    
end
%% Plot(3): Quiver plot of induced velocity vectors @ azimuth stations wrt VEHICLE coordinate system
if strcmpi(plotting(3),'on')
    figure(3)
    hold on
    %arrow3([cos(AoA_rad) 0 sin(AoA_rad)],[0 0 0],'b2',1) % Freestream velocity arrow
    for i = 1:rotor.num_rotors
        % Rotor
        X1 = [WIMrotor(i).s1(:,1),WIMrotor(i).s2(:,1)];
        Y1 = [WIMrotor(i).s1(:,2),WIMrotor(i).s2(:,2)];
        Z1 = [WIMrotor(i).s1(:,3),WIMrotor(i).s2(:,3)];
        
        % Plot rotor edge
        for ii = 1:wake.num_seg
            plot3(X1(ii,:),Y1(ii,:),Z1(ii,:),'-k','LineWidth',2);
        end
    end
    
    for i = 1:rotor.num_rotors
        quiver3(qvr_pts(:,1),qvr_pts(:,2),qvr_pts(:,3),q12_P(:,1,i),q12_P(:,2,i),q12_P(:,3,i));
    end
    
%             azm         =       oper.azimuth_num;
%             azimuth     =       linspace(0, 2*pi-(2*pi/azm),azm);
%             
%             xx = reshape(qvr_pts(1:length(qvr_pts)/4,1),[],1);
%             yy = reshape(qvr_pts(1:length(qvr_pts)/4,2),[],1);
%             zz = reshape([q12_P(1:length(qvr_pts)/4,2)],[],1);
% 
%             [xi,yi] = meshgrid(-1.1:0.001:1.1); 
%             zi = griddata(xx,yy,zz,yi,xi); % Flip and rotate axes to follow TPP convention
% 
%     contour3(xi,yi,zi)

    
    hold off
%     grid on
%     grid minor
%     axis equal
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');
end
%% Plot(4): Contour plot of normalized induced velocity on xz or xy planes in VEHICLE coordinate system
if strcmpi(plotting(4),'on')
    
    % Determine the length of contour slice plane based on rotor's and their arm lengths 
    plot_window =  rotor.armLENGTH + (R); % 1 Rotor diameter on each side spacing
    
    % Determine vi magnitudes
    if strcmpi(wake.contour_plane,'xz') % "Side view"
        [XX,ZZ] = meshgrid(linspace(plot_window,-plot_window,wake.mesh_dim)');
        CP = [];
        for j = 1:1:length(XX)
            for jj = 1:1:length(ZZ)
                CP = [CP;XX(jj,j) 0 ZZ(jj,j)]; % Use same index since in meshgrid format!
            end
        end
        [c_peaks] = Biot_Savart(CP,vertcat(WIMwake(1:rotor.num_rotors).s1),vertcat(WIMwake(1:rotor.num_rotors).s2),gamma);
        % Magnitude is the vertical induced velocity component
        Contour_magnitude = reshape(c_peaks(:,3),[wake.mesh_dim,wake.mesh_dim]);
    elseif strcmpi(wake.contour_plane,'xy') % "Top view"
        [XX,YY] = meshgrid(linspace(plot_window,-plot_window,wake.mesh_dim)');
        CP = [];
        for j = 1:1:length(XX)
            for jj = 1:1:length(YY)
                CP = [CP;XX(jj,j) YY(jj,j) 0]; % Use same index since in meshgrid format!
            end
        end
        [c_peaks] = Biot_Savart(CP,vertcat(WIMwake(1:rotor.num_rotors).s1),vertcat(WIMwake(1:rotor.num_rotors).s2),gamma);
        % Magnitude is the vertical induced velocity component!
        Contour_magnitude = reshape(c_peaks(:,3),[wake.mesh_dim,wake.mesh_dim]);
    end
    
    figure(4)
    hold on
    if strcmpi(wake.contour_plane,'xz') % "Side view"
        for i = 1:rotor.num_rotors
            X1 = [WIMrotor(i).s1(:,1),WIMrotor(i).s2(:,1)];
            Y1 = [WIMrotor(i).s1(:,2),WIMrotor(i).s2(:,2)];
            Z1 = [WIMrotor(i).s1(:,3),WIMrotor(i).s2(:,3)];
            fill3(X1,Y1,Z1,[.9 .9 .9]);alpha(.5);
        end
        
        
        LevelList = [1.9;1.8;1.6;1.4;1.2;1;.8;.6;.4;.2;.1;.06;.04;.02;.01;-.2;-.1;-.06;-.04;-.02;-.01]; % Contours in Castles&Deleeuw
        %contour(-XX/R,ZZ/R,(Contour_magnitude/Contour_magnitude(wake.mesh_dim/2,wake.mesh_dim/2)),'LevelList',LevelList,'ShowText','on')
        contour(-XX/R,ZZ/R,(Contour_magnitude),'LevelList',LevelList,'ShowText','on')
        %contour(-XX/R,ZZ/R,Contour_magnitude,'ShowText','on')
        axis equal
        xlabel('X-axis')
        ylabel('Z-axis')
        
    elseif strcmpi(wake.contour_plane,'xy') % "Top view"
        for i = 1:rotor.num_rotors
            X1 = [WIMrotor(i).s1(:,1),WIMrotor(i).s2(:,1)];
            Y1 = [WIMrotor(i).s1(:,2),WIMrotor(i).s2(:,2)];
            Z1 = [WIMrotor(i).s1(:,3),WIMrotor(i).s2(:,3)];
            fill3(X1,Y1,Z1,[.9 .9 .9]);alpha(.5);
        end
        
        % Contour
        LevelList = [1.9;1.8;1.6;1.4;1.2;1;.8;.6;.4;.2;.1;.06;.04;.02;.01;-.2;-.1;-.06;-.04;-.02;-.01]; % Contours in Castles&Deleeuw
        contour(XX/R,YY/R,(Contour_magnitude/Contour_magnitude(wake.mesh_dim/2,wake.mesh_dim/2)))
        axis equal
        xlabel('X-axis')
        ylabel('Y-axis')
    end
    hold off
end
%% Plot(5): Surf plot of normalized induced velocity on xz or xy planes in VEHICLE coordinate system
if strcmpi(plotting(5),'on')
    
    % Determine the length of contour slice plane based on rotor's and their arm lengths 
    plot_window =  rotor.armLENGTH + (perf(1).R*2); % 1 Rotor diameter on each side spacing
    
        [XX,YY] = meshgrid(linspace(plot_window,-plot_window,wake.mesh_dim)');
        CP = [];
        for j = 1:1:length(XX)
            for jj = 1:1:length(YY)
                CP = [CP;XX(jj,j) YY(jj,j) 0]; % Use same index since in meshgrid format!
            end
        end
        
        for i = 1:rotor.num_rotors
            c_peaks(:,:,i) = Biot_Savart(CP,vertcat(WIMwake(i).s1),vertcat(WIMwake(i).s2),gamma(i));
            xv =[];
            yv =[];
            
            % define circle query zones for just adjacent rotors
            for k = 1:rotor.num_rotors
                if k == i
                else
                    L = linspace(0,2.*pi,360)';
                    xv = [xv; (perf(k).R.*cos(L)+ WIMrotor(k).centres(1)); NaN];
                    yv = [yv; (perf(k).R.*sin(L)+ WIMrotor(k).centres(2)); NaN];
                end
            end
            
                [in,on] = inpolygon(CP(:,1),CP(:,2),xv,yv);
                c_peaks(~in,1,i) = 0; c_peaks(on,1,i) = 0;
                c_peaks(~in,2,i) = 0; c_peaks(on,2,i) = 0;
                c_peaks(~in,3,i) = 0; c_peaks(on,3,i) = 0; 
        end
        
        c_peaks_sum = sum(c_peaks(:,:,:),3);
        c_peaks_sum(c_peaks_sum==0) = NaN;
        
        value_to_plot = c_peaks_sum(:,3)./(sqrt(perf(i).T./(2*flow.rho.*A(i)))); %1 = x-component, 2 = y-component, 3 = z-component, 

        % Magnitude is the vertical induced velocity component!
        Contour_magnitude_normalized = reshape(value_to_plot,[wake.mesh_dim,wake.mesh_dim]);

        Contour_magnitude_x = reshape(c_peaks_sum(:,1),[wake.mesh_dim,wake.mesh_dim]);
        Contour_magnitude_y = reshape(c_peaks_sum(:,2),[wake.mesh_dim,wake.mesh_dim]);

        Contour_magnitude_normalized_x = Contour_magnitude_x./(sqrt(perf(i).T./(2*flow.rho.*A(i))));
        Contour_magnitude_normalized_y = Contour_magnitude_y./(sqrt(perf(i).T./(2*flow.rho.*A(i))));

        
        % Normalize control points and query points
        CP_normalized = CP./perf(i).R;
        XX_normalized = XX./perf(i).R;
        YY_normalized = YY./perf(i).R;
                
       
% Refine grid mesh
    dx=0.002;
    dy=0.002;
    xxx= reshape(CP_normalized(:,1),[],1);
    yyy= reshape(CP_normalized(:,2),[],1);
    zzz= value_to_plot;
    x_edge=[floor(min(xxx)):dx:ceil(max(xxx))];
    y_edge=[floor(min(yyy)):dy:ceil(max(yyy))];
    [X,Y]=meshgrid(x_edge,y_edge);
    Z=griddata(xxx,yyy,zzz,X,Y);

    rotor_surf(XX_normalized, YY_normalized, Contour_magnitude_normalized, X,Y,Z, WIMrotor,perf,rotor)
    
    % Z 
    rotor_plane_plots(XX_normalized,YY_normalized,Contour_magnitude_normalized,WIMrotor,perf,rotor);

    % X
    rotor_plane_plots(XX_normalized,YY_normalized,Contour_magnitude_normalized_x,WIMrotor,perf,rotor);
    
    % Y
    rotor_plane_plots(XX_normalized,YY_normalized,Contour_magnitude_normalized_y,WIMrotor,perf,rotor);
    
end

end


function [] = rotor_surf(xdata1, ydata1, zdata1, xdata2, ydata2, zdata2, WIMrotor,perf,rotor)

% Create figure
figure1 = figure('InvertHardcopy','off','PaperSize',[5 5],'Color',[1 1 1]);
colormap(jet);

% Create axes
axes1 = axes('Parent',figure1,'Position',[0.23 0.28 0.581249999999999 0.61125]);
hold(axes1,'on');

% Create multiple lines using matrix input to plot3
    for i = 1:rotor.num_rotors
        X1 = [WIMrotor(i).s1(:,1),WIMrotor(i).s2(:,1)]./perf(i).R;
        Y1 = [WIMrotor(i).s1(:,2),WIMrotor(i).s2(:,2)]./perf(i).R;
        Z1 = [WIMrotor(i).s1(:,3),WIMrotor(i).s2(:,3)]./perf(i).R;
        plot3(X1,Y1,Z1,'Parent',axes1,'Color',[0 0 0]);
    end


% Create contour
contour(xdata1,ydata1,zdata1,'LineColor',[0 0 0],'LevelStep',0.025);

% Create surf
surf(xdata2,ydata2,zdata2,'Parent',axes1,'FaceLighting','gouraud','FaceAlpha',0.9,...
    'SpecularStrength',0,...
    'DiffuseStrength',0.8,...
    'EdgeColor','none');

% Create light
%light('Parent',axes1);

% Create xlabel
xlabel('X/R','FontWeight','bold','FontSize',13.2,'FontName','Arial');

% Create zlabel
zlabel('v_{i} / v','EdgeColor',[1 1 1],'FontWeight','bold','FontAngle','italic','FontSize',13.2,...
    'FontName','Arial',...
    'Rotation',90);

% Create ylabel
ylabel('Y/R','FontWeight','bold','FontSize',13.2,'FontName','Arial');

xlim(axes1,[-3.5 3.5]);
ylim(axes1,[-3.5 3.5]);
zlim(axes1,[-0.2 0.2]);

view(axes1,[-30 35]);
box(axes1,'on');

grid(axes1,'on');
% Set the remaining axes properties


set(axes1,'CLim',[-0.05 0.25],'CameraViewAngle',9.85767154166459,'DataAspectRatio',[8 8 1],...
    'FontSize',12,'PlotBoxAspectRatio',[1.75 1.75 1],'XTick',[-3 -2 -1 0 1 2 3],'YTick',...
    [-3 -2 -1 0 1 2 3],'ZTick',[-0.2 -0.1 0 0.1 0.2 0.3]);


% Create colorbar
colorbar('peer',axes1,'Position',[0.369943502824858 0.176912611198325 0.299999999999999 0.04],...
    'FontSize',10.8);


% Plot arrow
arrow3([3.5 0 0] ,[2.8 0 0],['r' '1.5'],1.5,1.5)

hold(axes1,'off');
hold off;
end


function [] = rotor_plane_plots(xdata1, ydata1, zdata1,WIMrotor,perf,rotor)

% Create figure
figure2 = figure('InvertHardcopy','off','PaperSize',[2.75 2.75],'Color',[1 1 1],...
    'Renderer','zbuffer');

% Create axes
axes2 = axes('Parent',figure2,'Units','inches','Position',[4.05 2.15 2.75 2.75]);
hold(axes2,'on');

%view(axes2,[90 90])

% Create contour
[c2,h2] = contour(xdata1,ydata1,zdata1,'LineColor',[0 0 0],'LevelStep',0.02);
clabel(c2,h2,'FontSize',7,'Rotation',90);

    for i = 1:rotor.num_rotors
        X1 = [WIMrotor(i).s1(:,1),WIMrotor(i).s2(:,1)]./perf(i).R;
        Y1 = [WIMrotor(i).s1(:,2),WIMrotor(i).s2(:,2)]./perf(i).R;
        Z1 = [WIMrotor(i).s1(:,3),WIMrotor(i).s2(:,3)]./perf(i).R;
        plot3(X1,Y1,Z1,'Parent',axes2,'Color',[0 0 0]);
    end


% Create xlabel
xlabel('X/R','FontWeight','bold','FontSize',13.2,'FontName','Arial');

% Create zlabel
zlabel('v_{i} / v','EdgeColor',[1 1 1],'FontWeight','bold','FontAngle','italic','FontSize',13.2,...
    'FontName','Arial',...
    'Rotation',90);

% Create ylabel
ylabel('Y/R','FontWeight','bold','FontSize',13.2,'FontName','Arial');


xlim(axes2,[-3.5 3.5]);
ylim(axes2,[-3.5 3.5]);
zlim(axes2,[-0.2 0.2]);

box(axes2,'on');
grid(axes2,'on');



% Set the remaining axes properties
set(axes2,'CLim',[0 0.1],'CameraViewAngle',9.85767154166459,'DataAspectRatio',[8 8 1],...
    'FontSize',12,'PlotBoxAspectRatio',[2.1875 2.1875 1],'XTick',[-3 -2 -1 0 1 2 3],'YTick',...
    [-3 -2 -1 0 1 2 3],'ZTick',[-0.2 -0.1 0 0.1 0.2]);

arrow3([3.5 0 0] ,[2.8 0 0],['r' '1.5'],1.5,1.5)


hold(axes2,'off');
hold off;

end


