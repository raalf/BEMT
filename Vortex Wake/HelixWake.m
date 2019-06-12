%---------------------------
% Vortex Helix Wake Function
%---------------------------

% Creates a helix wake structure for X number of rotors and Y number
% of ring vortices with Z number of elements. Function outputs are
% endpoints s1 and s2
%
% INPUTS:
% -------
% wake:         wake structure: wake.num_seg (number of segments),
%               wake.num_rings (number of ring elements).
% rotor:        rotor structure: num_rotors,
%               orientation,armLENGTH,roll,rp_twist.
% R:            Radius
% alpha:        Angle of attack
% r_mid:        Positions of station midpoints along blade
% armAZIMUTH:   Angle between arms of vehicle
% chi:          Skew angle
% z:            Spacing of segments
% plotting:      'on' or 'off'


function [WIMrotor,WIMwake] = HelixWake(wake,rotor,R,AoA_rad,mid_span,armAZIMUTH,azimuth,z,h,plotting)

% Generate query point grid [x,y,z] based on azimuth positions and radial stations
    query_points        =   [reshape(bsxfun(@times,mid_span,-cos(azimuth)),[size(mid_span,1)*size(azimuth,2),1]),...
                            reshape(bsxfun(@times,mid_span,-sin(azimuth)),[size(mid_span,1)*size(azimuth,2),1]),...
                            zeros(size(mid_span,1)*size(azimuth,2),1)];

% Get single set of 3D points, s1 and s2, representing the rotor's edge. (for plotting)
    [s1,s2]             =   VortRing(wake.num_seg,R,[0,0,0],'off');

% Rotation matrix for arm twist and cant. About x-axis for "unpositioned" rotor from VortRing
    rotateTWIST         =   [1 0 0; 0 cosd(rotor.rp_twist) sind(rotor.rp_twist); 0 -sind(rotor.rp_twist) cosd(rotor.rp_twist)];
    rotateCANT          =   [cos(rotor.rp_cant) 0 -sin(rotor.rp_cant); 0 1 0; sin(rotor.rp_cant) 0 cos(rotor.rp_cant)];
    
% Apply twist rotation matrix to "unpositioned" rotor from VortRing and
% Qpoints. Do BEFORE positions around UAV body, since rotation axis is not
% about a global reference frame cartesian axis @ location of "hypothetical" rotor at psi(azm) = 0 position.
    WIMrotor(1).centres =   [-rotor.armLENGTH,0,0]*rotateCANT; % twist doesn't affect the center point
    WIMrotor(1).s1      =   bsxfun(@plus,s1,WIMrotor(1).centres)*rotateTWIST*rotateCANT; % For plotting
    WIMrotor(1).s2      =   bsxfun(@plus,s2,WIMrotor(1).centres)*rotateTWIST*rotateCANT; % For plotting
    WIMrotor(1).Qpoints =   bsxfun(@plus,query_points,WIMrotor(1).centres)*rotateTWIST*rotateCANT;

    extend_wake_elements=   ceil([1:((wake.num_elements)*(wake.num_seg))]./(wake.num_seg))';  
              
    
% Translate/duplicate ring to center point of each rotor hub. Apply rotation matrix to centre points as well
% Duplicate rotors and wake by rotation about UAV center (z axis) to each position. Skew angle is not applied yet
    for i = 1:rotor.num_rotors
        % Create wake structure by duplicating existing ring element in wake direction using cross product. 
            % WIMrotor fields are the rotor segments and qpoints, 
            % WIMwake fields are ring wake elements.
           
            % Rotation matrix for positioning, about z axis CCW.
            if i == 1 
                rot_angle           =   armAZIMUTH;
            else 
                rot_angle           = 	2*pi*(i-1)/rotor.num_rotors;
            end
            rotateHUB_POSITION  =   [cos(rot_angle) sin(rot_angle) 0; -sin(rot_angle) cos(rot_angle) 0; 0 0 1];
                
            % Location of rotor plane edge. (Used mostly for plotting)
                WIMrotor(i).s1      =   WIMrotor(1).s1*rotateHUB_POSITION;
                WIMrotor(i).s2      =   WIMrotor(1).s2*rotateHUB_POSITION;
                WIMrotor(i).Qpoints =   WIMrotor(1).Qpoints*rotateHUB_POSITION;
                WIMrotor(i).centres =   WIMrotor(1).centres*rotateHUB_POSITION;
            
            % Wake direction based on rotor plane orientation, wake and
            % skew angle
                WIMwake(i).s1       =   bsxfun(@plus,repmat(WIMrotor(i).s1,[wake.num_elements,1]),(extend_wake_elements*[-h,0,-z]));
                WIMwake(i).s2       =   bsxfun(@plus,repmat(WIMrotor(i).s2,[wake.num_elements,1]),(extend_wake_elements*[-h,0,-z]));   

                
                
    end 
    
%  Entire vehicle orientation rotation matrices
% (Needs to be done AFTER completing UAV rotor config i.e. rotate hub, twist and cant rotations)
% Rotation matrices for vehicle AOA (about y, CCW) and vehicle roll (about x, CCW)
    rotateROLL          =   [1 0 0; 0 cosd(rotor.roll) -sind(rotor.roll); 0 sind(rotor.roll) cosd(rotor.roll)]; % reversed rotation since x axis is "reversed" from overhead. Forward = +x
    rotateALPHA         =   [cos(AoA_rad) 0 -sin(AoA_rad); 0 1 0; sin(AoA_rad) 0 cos(AoA_rad)];

  for i = 1:rotor.num_rotors
    WIMwake(i).s1       =   (WIMwake(i).s1)*rotateALPHA*rotateROLL;
    WIMwake(i).s2       =   (WIMwake(i).s2)*rotateALPHA*rotateROLL;
    
    WIMrotor(i).centres =   WIMrotor(i).centres*rotateALPHA*rotateROLL;
    WIMrotor(i).s1      =   WIMrotor(i).s2*rotateALPHA*rotateROLL;
    WIMrotor(i).s2      =   WIMrotor(i).s2*rotateALPHA*rotateROLL;
    WIMrotor(i).Qpoints =   WIMrotor(i).Qpoints*rotateALPHA*rotateROLL;
    
    %% Plotting  
    if strcmpi(plotting,'on')
            % Rotor
            X1 = [WIMrotor(i).s1(:,1),WIMrotor(i).s2(:,1)];
            Y1 = [WIMrotor(i).s1(:,2),WIMrotor(i).s2(:,2)];
            Z1 = [WIMrotor(i).s1(:,3),WIMrotor(i).s2(:,3)];

            % Ring elements
            X2 = [WIMwake(i).s1(:,1),WIMwake(i).s2(:,1)];
            Y2 = [WIMwake(i).s1(:,2),WIMwake(i).s2(:,2)];
            Z2 = [WIMwake(i).s1(:,3),WIMwake(i).s2(:,3)];

            figure(1)
            hold on
            plot3([WIMrotor(i).centres(:,1),0],[WIMrotor(i).centres(:,2),0],[WIMrotor(i).centres(:,3),0],'-ok','LineWidth',2);
            % Rotor plane shading
            fill3(X1,Y1,Z1,'b');alpha(.5);
            % Plot rotor markers
            text(WIMrotor(i).centres(:,1),WIMrotor(i).centres(:,2),WIMrotor(i).centres(:,3), num2str(i),'Color','w','FontSize',16,'FontWeight','bold');

            % Plot rotor
            for ii = 1:wake.num_seg
                plot3(X1(ii,:),Y1(ii,:),Z1(ii,:),'b','LineWidth',3);
            end

            % Plot wake elements
            for k = 1:wake.num_seg*wake.num_elements
                plot3(X2(k,:),Y2(k,:),Z2(k,:),'-k');
            end

            % Plot query points
            %    plot3(WIMrotor(i).Qpoints(:,1),WIMrotor(i).Qpoints(:,2),WIMrotor(i).Qpoints(:,3))
            xlabel('X-axis');
            ylabel('Y-axis');
            zlabel('Z-axis');
            hold off
            daspect([max(daspect)*[1 1] 1]) % Set xy plane axis to be the same
            grid on;
            grid minor;
            axis equal
    end
    %% Method 2
    % % Method 2 of generating query points on rotor plane    
    % % Query points @ blade stations/azimuth position for rotor (independent of position). azm = 0 @ rear of rotor
    % %     WIMrotor(i).Qpoints = [reshape(bsxfun(@times,r_mid,sin(azimuth)) + WIMrotor(i).centres(1),[size(r_mid,1)*size(azimuth,2),1]),...
    % %                            reshape(bsxfun(@times,r_mid,cos(azimuth)) + WIMrotor(i).centres(2),[size(r_mid,1)*size(azimuth,2),1]),zeros(size(r_mid,1)*size(azimuth,2),1)];
    % 
    % 
    % % Method 2 of arm axis rotation
    % % Rotation matrix around uav arms axis' (defined as unit vector).
    % %     ux  = cos(armAZIMUTH(i));
    % %     uy  = sin(armAZIMUTH(i));
    % %     uz  = 0;
    % %     
    % %     rotateTWIST = [cos(rotor.rp_twist)+ux^2*(1-cos(rotor.rp_twist)), ux*uy*(1-cos(rotor.rp_twist))-uz*sin(rotor.rp_twist),ux*uz*(1-cos(rotor.rp_twist))+uy*sin(rotor.rp_twist);...
    % %                     uy*ux*(1-cos(rotor.rp_twist))+uz*sin(rotor.rp_twist), cos(rotor.rp_twist)+uy^2*(1-cos(rotor.rp_twist)),uy*uz*(1-cos(rotor.rp_twist))-ux*sin(rotor.rp_twist);...
    % %                     uz*ux*(1-cos(rotor.rp_twist))-uy*sin(rotor.rp_twist), uz*uy*(1-cos(rotor.rp_twist))+ux*sin(rotor.rp_twist), cos(rotor.rp_twist)+uz^2*(1-cos(rotor.rp_twist))];
    end
end
