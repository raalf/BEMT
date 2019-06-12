%---------------------
% Vortex Ring Function
%---------------------
% Calculates and returns points of vortex ring corresponding to the radius
% of the rotor plane at a center point

% number_of_segments:   Number of segments
% R:                    Rotor radius [m]
% CenterPoint:          Center point [x,y,z]
% plotting_option:      'on' or 'off' (Case insensitive). 3d plot of ring

% Sign convention:
% 0 [deg] is back of rotor plane (normalized coordinates of [-1,0])
% UAV tilts about x-axis, tilt down is + angle up to 90 (axial)


function [s1,s2] = VortRing(number_of_segments,R,CenterPoint,plotting_option)

% Calculate cartesian angles of endpoints s1 and s2. CCW, from back of rotor plane [-1,0].
anglesS1 = linspace(0,(2*pi)-(2*pi/number_of_segments),number_of_segments);
anglesS2 = anglesS1 + (2*pi/number_of_segments);
s1 = R*[-cos(anglesS1)',-sin(anglesS1)',CenterPoint(3)*ones(number_of_segments,1)];
s2 = R*[-cos(anglesS2)',-sin(anglesS2)',CenterPoint(3)*ones(number_of_segments,1)];

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

