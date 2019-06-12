%--------------------------
% Vortex Ring Wake Function
%--------------------------

% Creates a ring wake structure for X number of rotors and Y number
% of ring vortices with Z number of elements. Function outputs are
% endpoints s1 and s2, query points of rotor and wake elements.
%
% INPUTS:
% -------
% wake:         wake structure: wake.num_seg (number of segments),
%               wake.num_elementss (number of ring elements).
% rotor:        rotor structure: num_rotors,
%               orientation,armLENGTH,roll,rp_twist.
% R:            Radius
% alpha:        Angle of attack
% r_mid:        Positions of station midpoints along blade
% armAZIMUTH:   Angle between arms of vehicle
% chi:          Skew angle
% z:            Spacing of segments
% plotting:      'on' or 'off'


%WIMrotor Vs WIMwake
% Cant and twist are not functional since this changes the angle of attack
% for each rotor then
% WIMrotor fields are the rotor segments and qpoints, 
% WIMwake fields are ring wake elements.

function [WIMrotor,WIMwake] = RingWake_Multi(wake,rotor,R,mid_span,armAZIMUTH,azimuth,z,h)

% Generate query point grid based on azimuth positions and stations, starting at azimuth = 0 position
    query_points        =   [reshape(bsxfun(@times,mid_span,-cos(azimuth)),[size(mid_span,1)*size(azimuth,2),1]),...
                            reshape(bsxfun(@times,mid_span,-sin(azimuth)),[size(mid_span,1)*size(azimuth,2),1]),...
                            zeros(size(mid_span,1)*size(azimuth,2),1)];

% Get single set of 3D points, s1 and s2, representing the rotor's edge.
    [s1,s2]             =   VortRing(wake.num_seg,R,[0,0,0],'off');

% Rotation matrix for arm twist and cant. About x-axis for "unpositioned" rotor from VortRing
    rotateTWIST         =   [1 0 0; 0 cosd(rotor.rp_twist) sind(rotor.rp_twist); 0 -sind(rotor.rp_twist) cosd(rotor.rp_twist)];
    rotateCANT          =   [cos(rotor.rp_cant) 0 -sin(rotor.rp_cant); 0 1 0; sin(rotor.rp_cant) 0 cos(rotor.rp_cant)];
    
% Apply twist rotation matrix to "unpositioned" rotor from VortRing and
% Qpoints. Do BEFORE positions around UAV body, since rotation axis is not
% about a global reference frame cartesian axis @ location of "hypothetical" rotor at psi(azm) = 0 position.
    WIMrotor(1).centres =   [-rotor.armLENGTH,0,0]*rotateCANT; % twist doesn't affect the center point
    WIMrotor(1).s1      =   bsxfun(@plus,s1,WIMrotor(1).centres)*rotateTWIST*rotateCANT;
    WIMrotor(1).s2      =   bsxfun(@plus,s2,WIMrotor(1).centres)*rotateTWIST*rotateCANT;
    WIMrotor(1).Qpoints =   bsxfun(@plus,query_points,WIMrotor(1).centres)*rotateTWIST*rotateCANT;
    
                
        extend_wake_elements=   ceil([1:((wake.num_elements)*(wake.num_seg))]./(wake.num_seg))'; % excludes rotor plane as ring    
        extend_wake_elements=   extend_wake_elements-1; % include rotor plane as ring UNLESS, inflow is determined for this rotor using other means
            
    
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

        % Wake direction based on rotor plane orientation, wake and skew angle
            WIMwake(i).s1       =   bsxfun(@plus,repmat(WIMrotor(i).s1,[wake.num_elements,1]),(extend_wake_elements*[-h(i),0,-z(i)]));
            WIMwake(i).s2       =   bsxfun(@plus,repmat(WIMrotor(i).s2,[wake.num_elements,1]),(extend_wake_elements*[-h(i),0,-z(i)]));
    end
end

