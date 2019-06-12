%----------------------
% Vortex Helix Function
%----------------------
% Calculates and returns points of vortex helix(s) corresponding to the
% number of azimuth stations.

% number_of_segments:   Number of segments
% number_of_elements:   Number_of_elements
% R:                    Rotor radius [m]
% CenterPoint:          Center point [x,y,z]


% Sign convention:
% 0 and 2pi is back of rotor plane
% UAV tilts about y-axis, tilt down is + angle up to 90 (axial). z axis is
% height

function [s1,s2] = VortHelix(number_of_segments,number_of_elements,R,vertical_spacing,chi,AoA_rad,azimuth)

% Vector defining the direction of the skew angle wake spacing
    extend_wake_elements=  linspace(0,number_of_elements,number_of_segments*number_of_elements)';   
    v =  [0*extend_wake_elements,bsxfun(@times,(vertical_spacing*cos((AoA_rad+chi)-(pi/2))),extend_wake_elements)];    

% Helix formation
    azimuth_helix = bsxfun(@times,azimuth,ones(number_of_segments*number_of_elements,1));

% Apply logarithmic segment spacing to wake elements within a radius of the
% rotor plane. Determine how many elements are in the "catchment area"

    number_of_helices_to_equal_R = R/vertical_spacing;

% Parametric equations defining helix
    t = cat(1,logspace()',linspace(0,2*pi*number_of_elements,number_of_segments*number_of_elements)');
    azimuth_t = bsxfun(@plus,azimuth_helix,t);
    x = bsxfun(@plus,(R*sin(t_azimuth)),v(:,1));
    y = bsxfun(@plus,(R*cos(t_azimuth)),v(:,2));
    z = ((-vertical_spacing.*t_azimuth)./(2*pi));


% watch outputting s1 and s2 since they become tensors since s1 =
% [x,y,z] of single numbers. Maybe output x y z coordinates and deal with
% when each helix is called?

%% Plotting
    if strcmpi(plotting_option,'on')
        X = [s1(:,1),s2(:,1)];
        Y = [s1(:,2),s2(:,2)];
        Z = [s1(:,3),s2(:,3)];

        for ii = 1:number_of_segments
            hold on
            plot3(X(ii,:),Y(ii,:),Z(ii,:))
            xlabel('X-axis');
            ylabel('Y-axis');
            zlabel('Z-axis');
            hold off
        end
    end
end
