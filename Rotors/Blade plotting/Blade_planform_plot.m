% Plotting function for blade planform
% Includes plotting options for both twisted and untwisted blades

function Blade_planform_plot(blade)

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
    plot(Section_Radius,[chord_twisted_leading,chord_twisted_trailing],'b')
    hold off
    
    title('Blade planform');
    xlabel('x (Radius)'); % x-axis label
    ylabel('y (Chord)'); % y-axis label
    axis ([(R/stations) R -R/2 R/2]);     
    
    
    
%     str = {['\bf RPM: ' num2str(final.rpm)], ['\bf Diameter [m]: ' num2str(final.diameter)],...
%         ['\bf Power [W]: ' num2str(final.Power)]};
%     annotation('textbox', [0.59 0.125 0.3 0.15],...
%             'BackgroundColor','white', 'String', str)
    
end
    
    