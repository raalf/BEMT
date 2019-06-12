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

function [vi,qvr_pts] = WIM(flow,oper,rotor,wake,T,B,mid_span,azimuth,R,AoA_rad,plotting)
%% [1] Calculating wake paramters, handle data inputs
%----------------------------------------------------
        
        A           =       pi.*(R.^2);                                 % Area of Rotor
        gamma       =       -T./(B*flow.rho*pi*(oper.rpm/60)*(R^2));    % Circulation, gamma. Negative sign because WIMs are plotted CCW (right hand rule).

    % Vehicle orientation, rotor spacing
        if (strcmp(rotor.orientation,'square') == 1 && rem(rotor.num_rotors,2) == 0) || ((strcmp(rotor.orientation,'diamond') == 1 && rem(rotor.num_rotors,2) == 1)) % if (even # of rotors & square) or (odd # and diamond), rotor "1" is rotated CCW by 2*pi/rotor.num_rotors
            armAZIMUTH   = pi/rotor.num_rotors;
        else
            armAZIMUTH   = 0;
        end

    % Skew angle from initial thrust loading (assuming all rotors have same thrust loading. This can be changed easily though)
        chi         =       atan((flow.V*cos(AoA_rad))/(flow.V*sin(AoA_rad)+sqrt(T/(2*pi*flow.rho*(R^2))))); %calculate skew angle of wakes for influencing rotor

    % Vertical offset between ring wake elements    
        z           =       (1/(B*oper.rpm/60)).*(sqrt(T/(2*flow.rho*pi*R^2))+flow.V*sin(AoA_rad));
       
    % Horizontal offset between ring wake elements   
        h           =       tan(chi)*z;
%---------------------------------------
%% [2] Create wake structure and Qpoints
%---------------------------------------

    % Create wake structure (vorticity line segments, rotor position, query points) depending on which wake formulation is chosen.
    switch lower(wake.type)
        case 'ring'
                    [WIMrotor,WIMwake] = RingWake(wake,rotor,R,mid_span,armAZIMUTH,azimuth,z,h);
        case 'helix'
                    for i = 1:length(azimuth)
                    % Need iteration loop to twist WIMs. Can still use helix for
                    % adjacent wakes tho, when untwisted
                        [WIMrotor,WIMwake] = HelixWake(wake,rotor,R,mid_span,armAZIMUTH,azimuth(i),z,h);
                    end     
    end
    
% Calculate the induced velocity at each query point. Call Biot Savart Function
    % Wake structure same for every query point/azimuth for ring formulation

% To do all in one go  
%             q12_P(i) = Biot_Savart(vertcat(WIMrotor(1:rotor.num_rotors).Qpoints),...
%                       vertcat(WIMwake(1:rotor.num_rotors).s1),...
%                       vertcat(WIMwake(1:rotor.num_rotors).s2),...
%                       gamma); 
% Induced velocity. Sign convention in BEMT is down is positive!
%    vi() = -[q12_P(:,1),q12_P(:,2),q12_P(:,3)];
    
    for i = 1:rotor.num_rotors
        % q12_P is the induced velocity at Qpoints (starting with rotor 1, azimuth position azm = 0)
        q12_P(i,:) = Biot_Savart(vertcat(WIMrotor(i).Qpoints),...
                      vertcat(WIMwake(i).s1),...
                      vertcat(WIMwake(i).s2),...
                      gamma); 
    
    end
  
% Induced velocity. Sign convention in BEMT is down is positive!
    vi = -[q12_P(:,1),q12_P(:,2),q12_P(:,3)];
    
% Concatenate Qpoints into one variable for quiver plotting positions
    qvr_pts = vertcat(WIMrotor(1:rotor.num_rotors).Qpoints);
    
%-----------------------
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
    rotateALPHA         =   [cos(AoA_rad) 0 -sin(AoA_rad); 0 1 0; sin(AoA_rad) 0 cos(AoA_rad)];
    
    
    
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
    
    quiver3(qvr_pts(:,1),qvr_pts(:,2),qvr_pts(:,3),q12_P(:,1),q12_P(:,2),q12_P(:,3));
       
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
    plot_window =  rotor.armLENGTH + (R*3); % 1 Rotor diameter on each side spacing
    
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
    
    figure(5)
    hold on
    if strcmpi(wake.contour_plane,'xz') % "Side view"
%         for i = 1:rotor.num_rotors
%             X1 = [WIMrotor(i).s1(:,1),WIMrotor(i).s2(:,1)];
%             Y1 = [WIMrotor(i).s1(:,2),WIMrotor(i).s2(:,2)];
%             Z1 = [WIMrotor(i).s1(:,3),WIMrotor(i).s2(:,3)];
%             fill3(X1,Y1,Z1,[.9 .9 .9]);alpha(.5);
%         end
        
        
        LevelList = [1.9;1.8;1.6;1.4;1.2;1;.8;.6;.4;.2;.1;.06;.04;.02;.01;-.2;-.1;-.06;-.04;-.02;-.01]; % Contours in Castles&Deleeuw
        contour(-XX/R,ZZ/R,(Contour_magnitude/Contour_magnitude(wake.mesh_dim/2,wake.mesh_dim/2)),'LevelList',LevelList,'ShowText','on')
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

        Contour_magnitude(isnan(Contour_magnitude)) = 0;
        Contour_magnitude(Contour_magnitude>5) = NaN;
        Contour_magnitude(Contour_magnitude<-5) = NaN;

 surf(XX,YY,Contour_magnitude,'EdgeColor','none')


    % Refine grid mesh
        dx=0.005;
        dy=0.005;
        xxx= reshape(CP(:,1),[],1);
        yyy= reshape(CP(:,2),[],1);
        zzz= reshape(c_peaks(:,3),[],1);
        x_edge=[floor(min(xxx)):dx:ceil(max(xxx))];
        y_edge=[floor(min(yyy)):dy:ceil(max(yyy))];
        [X,Y]=meshgrid(x_edge,y_edge);
        Z=griddata(xxx,yyy,zzz,X,Y);

        Z(isnan(Z)) = 0;
        Z(Z>500) = NaN;
        Z(Z<-500) = NaN;

figure(55)
hold on
        for i = 1:rotor.num_rotors
            X1 = [WIMrotor(i).s1(:,1),WIMrotor(i).s2(:,1)];
            Y1 = [WIMrotor(i).s1(:,2),WIMrotor(i).s2(:,2)];
            Z1 = [WIMrotor(i).s1(:,3),WIMrotor(i).s2(:,3)];
            fill3(X1,Y1,Z1,[.9 .9 .9]);alpha(.5);
        end
surf(X,Y,Z,'EdgeColor','none')

        xlabel('X-axis')
        ylabel('Y-axis')
    end
    hold off
end
%% Plot(6): Vi on x-axis plot: LOCAL ROTOR coordinate system.
if strcmpi(plotting(6),'on')
    
    % Normalized points to plot along x axis
    x_axis = [3.2;2;1.6;1.2;1.1;0.9;0.8;0.4;0;-0.4;-0.8;-0.9;-1.1;-1.2;-1.6;-2;-3.2]; %Deleeuw spacing
    x_axis = [3.2 3	2.8	2.6	2.4	2.2	2 1.8 1.6 1.4 1.2 0.8 0.6 0.4 0.2 0	-0.2 -0.4 -0.6 -0.8 -1.2 -1.4 -1.6 -1.8 -2 -2.2 -2.4 -2.6 -2.8 -3 -3.2]';
   
    x_axis= x_axis*R; % Position wrt to rotor plane's (dimensionalize to radius)
    x_axis(:,2) = 0; x_axis(:,3) = 0;

    %x_axis = [linspace(4*R,-4*R,wake.mesh_dim)']; x_axis(:,2) = 0; x_axis(:,3) = 0;
    [v_i] = Biot_Savart(x_axis,vertcat(WIMwake(1:rotor.num_rotors).s1),vertcat(WIMwake(1:rotor.num_rotors).s2),gamma);
    Vi_v = v_i(:,3)/v_i(ceil(end/2),3);
    x_R = x_axis(:,1)./R;
     
figure(6)
    hold on
    plot(x_R,Vi_v);
    hold off
    grid on
    grid minor
    xlabel('x-axis');
    ylabel('y-axis');
    
end
%% Plot(7): (IN WORKS) Volumetric slice on 3d wim plot

if strcmpi(plotting(6),'on')
    
    
    % Determine the length of contour slice based on rotor's and their arm lengths 
    plot_window =  rotor.armLENGTH + (R*2); % 1 Rotor diameter on each side spacing
    meshdim = linspace(plot_window,-plot_window,wake.mesh_dim);
    
    [XX,YY,ZZ] = meshgrid(meshdim,meshdim,meshdim);
    CP = [];
    
 %for i = 1:rotor.num_rotors    
    for j = 1:1:length(ZZ)
        parfor jj = 1:1:length(YY)
            CP = [XX(:,jj,j) YY(:,jj,j) ZZ(:,jj,j)]; % Use same index since in meshgrid format!           
            [v_i] = Biot_Savart(CP,vertcat(WIMwake(1:rotor.num_rotors).s1),vertcat(WIMwake(1:rotor.num_rotors).s2),gamma);
            viz(:,jj,j) = v_i(:,3);
            Viz_vz(:,jj,j) = v_i(:,3)/v_i(ceil(end/2),3);
        end
    end
    
   % [c_peaks] = Biot_Savart(CP,vertcat(WIMwake(1:rotor.num_rotors).s1),vertcat(WIMwake(1:rotor.num_rotors).s2),gamma);
    % Magnitude is the vertical induced velocity component
 
    figure(6)
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
    
        
    xslice = []; 
    yslice = []; 
    zslice = [0.1];
    
    
    slice(XX,YY,ZZ,Viz_vz,xslice,yslice,zslice);
    
    xlabel('X-axis');
    ylabel('Y-axis');
    zlabel('Z-axis');
    arrow3([cos(AoA_rad) 0 sin(AoA_rad)],[0 0 0],'b2',1) % Freestream velocity
    axis equal
    hold off

end
