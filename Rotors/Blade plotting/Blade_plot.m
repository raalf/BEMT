% Plotting function for blade (fill)
% Includes plotting options for both twisted and untwisted blades

function Blade_plot(blade)

% Extract rotor geometry and rotor parameters
    stations        =   height(blade.geometry);           % Number of stations through height of table
    R               =   blade.geometry{stations,'Radius'};         % Rotor diameter
    chord_R         =   blade.geometry{:,'c_R'};                   % c/R Normalized chord distribution [c/R]
    beta_deg        =   blade.geometry{:,'Beta'};                  % Pitch Angle distribution [Beta/R]

    Chord           =   blade.geometry{:,'Chord'};
    chord_twisted   =   cosd(beta_deg).*chord_R*R;
    sweep_axis      =   0.25;                             
    Section_Radius   =   blade.geometry.Radius';

    % Transform chords to points for blade
    chord_twisted_leading = chord_twisted*(sweep_axis);
    chord_twisted_trailing = chord_twisted*-(1-sweep_axis);
    %chord_upper = chord*sweep; %Untwisted plot - do not delete yet
    %chord_lower = chord*-(1-sweep); %Untwisted plot
      
 figure()
 
    hold on
    for i = 1:stations-1
    
        X = [Section_Radius(i); Section_Radius(i); Section_Radius(i+1); Section_Radius(i+1)];
        Y = [chord_twisted_trailing(i); chord_twisted_leading(i); chord_twisted_leading(i+1); chord_twisted_trailing(i+1)];
        Z = [tand(beta_deg(i))*chord_twisted_trailing(i); tand(beta_deg(i))*chord_twisted_leading(i); tand(beta_deg(i+1))*chord_twisted_leading(i+1); tand(beta_deg(i+1))*chord_twisted_trailing(i+1)];
        C = [1;1;1;1];
        fill3(X,Y,Z,C,'FaceColor','interp')
    end

    camlight('headlight')     
    title('Blade plot');
    xlabel('x (Radius)'); % x-axis label
    ylabel('y (Chord)'); % y-axis label
    axis ([(R/stations) R -R/2 R/2 -R/2 R/2]);   
    hold off  
end