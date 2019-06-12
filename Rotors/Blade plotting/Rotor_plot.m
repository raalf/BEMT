

function Rotor_plot(blade)

% Extract rotor geometry and rotor parameters
    stations        =   height(blade.geometry);                 % Number of stations through height of table
    B               =   blade.geometry{1,'Blades'};             % Number of blades 
    R               =   blade.geometry{stations,'Radius'};      % Rotor diameter
    chord_R         =   blade.geometry{:,'c_R'};                % c/R Normalized chord distribution [c/R]
    beta_deg        =   blade.geometry{:,'Beta'};               % Pitch Angle distribution [Beta/R]

    Chord           =   blade.geometry{:,'Chord'};
    chord_twisted   =   cosd(beta_deg).*chord_R*R;
    sweep_axis      =   0.25;                                   % Quarter chord                     
    Section_Radius  =   blade.geometry.Radius';
    
    % Coordinates of leading and trailing edge based on sweep axis
    chord_twisted_leading = chord_twisted*(sweep_axis);
    chord_twisted_trailing = chord_twisted*-(1-sweep_axis);
   
    % "Shift" position of blades so that they meet up at hub
    % (otherwise they're offset by sweep axis)
    chord_twisted_leading = chord_twisted_leading + (.5 - sweep_axis)*chord_twisted_leading(1);
    chord_twisted_trailing = chord_twisted_trailing + chord_twisted_leading(1);
    
figure()
hold on
    for loc = 0:360/B:360-360/B % Loop for radial position of blade [0-360]

            RM = [cosd(loc), sind(loc), 0;
                  -sind(loc), cosd(loc) , 0;
                      0    ,      0    , 1]; % Rotation matrix
          
        for i = 1:stations-1
            X = [Section_Radius(i); Section_Radius(i); Section_Radius(i+1); Section_Radius(i+1)];
            Y = [chord_twisted_trailing(i); chord_twisted_leading(i); chord_twisted_leading(i+1); chord_twisted_trailing(i+1)];
            Z = [tand(beta_deg(i))*chord_twisted_trailing(i); tand(beta_deg(i))*chord_twisted_leading(i); tand(beta_deg(i+1))*chord_twisted_leading(i+1); tand(beta_deg(i+1))*chord_twisted_trailing(i+1)];
            
            XYZ_RM = RM*[X';Y';Z'];

            C = [1;1;1;1];
            fill3(XYZ_RM(1,:),XYZ_RM(2,:),XYZ_RM(3,:),C,'FaceColor','interp')
        end
    end 

    camlight('headlight')     
    title('Rotor plot');
    xlabel('x (Radius)'); % x-axis label
    ylabel('y (Chord)'); % y-axis label
    axis ([-R R -R R -R/2 R/2]);   
    hold off 

end