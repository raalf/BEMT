% Plotting function for blade planform
% Includes plotting options for both twisted and untwisted blades

function Blade_planform_plot_rotor(blade,rotor)

    DV = size(rotor,2);                                     % Number of Design Variables
    pop = size(rotor,1);                                    % Size of population

% Extract rotor geometry and rotor parameters
    stations        =   (size(rotor,2)-1)/2;           % Number of stations through height of table
    R = blade.R;%R               =   blade.geometry{stations,'Radius'};         % Rotor diameter
    chord_R         =   rotor(:,1:((DV-1)/2));                  % c/R normalized chord [c/R]
    beta_deg        =   rotor(:,((DV-1)/2)+1:(DV-1));           % Pitch Angle distribution [Beta/R]
    
    
    chord_twisted   =   cosd(beta_deg).*chord_R*R;
    sweep_axis      =   0.25;                             % Sweep. (Constant 1/4 chord in this case)
    Stations_plot   =   (R/stations):(R/stations):R;      % Define Stations as an Array

    
    % Transform chords to points for blade
    chord_twisted_leading = chord_twisted*(sweep_axis);
    chord_twisted_trailing = chord_twisted*-(1-sweep_axis);
    %chord_upper = chord*sweep; %Untwisted plot - do not delete yet
    %chord_lower = chord*-(1-sweep); %Untwisted plot
    

 figure()
    hold on
    plot(Stations_plot,chord_twisted_leading,Stations_plot,chord_twisted_trailing,'b')
    hold off
    
    title('Blade plot');
    xlabel('x (Radius)'); % x-axis label
    ylabel('y (Chord)'); % y-axis label
    axis ([(R/stations) R -R/2 R/2]);    
    
%     str = {['\bf RPM: ' num2str(final.rpm)], ['\bf Diameter [m]: ' num2str(final.diameter)],...
%         ['\bf Power [W]: ' num2str(final.Power)]};
%     annotation('textbox', [0.59 0.125 0.3 0.15],...
%             'BackgroundColor','white', 'String', str)
%     
end
    
    