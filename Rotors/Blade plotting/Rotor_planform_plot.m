% Plotting function for blade planform
% Includes plotting options for both twisted and untwisted blades
%


function Rotor_planform_plot(blade)

% Extract rotor geometry and rotor parameters
    stations        =   height(blade.geometry);                 % Number of stations through height of table
    B               =   blade.geometry{1,'Blades'};             % Number of blades 
    R               =   blade.geometry{stations,'Radius'};      % Rotor diameter
    chord_R         =   blade.geometry{:,'c_R'};                % c/R Normalized chord distribution [c/R]
    beta_deg        =   blade.geometry{:,'Beta'};               % Pitch Angle distribution [Beta/R]
    airfoil         =   blade.geometry{:,'Airfoil'};            % Airfoils at each station

    Chord           =   blade.geometry{:,'Chord'};
    chord_twisted   =   cosd(beta_deg).*Chord;
    blade_locations =   360/B;
    sweep_axis      =   0.25;                                   % Sweep. (Constant 1/4 chord in this case)
    Stations_plot   =   (R*2/stations):(R*2/stations):R*2;      % Define Stations as an Array
    
figure()
hold on
    for loc = 0:blade_locations:360-blade_locations % Loop for radial position of blade [0-360]

            RM = [cosd(loc),-sind(loc);sind(loc),cosd(loc)]; % Rotation matrix

            % Transform chords to points for first blade
            chord_twisted_leading = chord_twisted*(1+sweep_axis)-(sweep_axis*chord_twisted);
            chord_twisted_trailing = chord_twisted*-(1-sweep_axis)-(sweep_axis*chord_twisted);
            %chord_upper = chord*sweep; %Untwisted plot - do not delete yet
            %chord_lower = chord*-(1-sweep); %Untwisted plot

            %Use rotation matrix for other blades
            Rdata  = RM*[Stations_plot; chord_twisted_leading'];
            Rdata2 = RM*[Stations_plot; chord_twisted_trailing'];
            %Rdata3 = RM*[Stations_plot;chord_upper'];
            %Rdata4 = RM*[Stations_plot;chord_lower'];

            plot(Rdata(1,:) , Rdata(2,:),'b'); %Twisted blade Upper
            %plot(Rdata3(1,:), Rdata3(2,:),'r'); %Untwisted blade Upper
            plot(Rdata2(1,:), Rdata2(2,:),'b');% Twisted blade Lower
            %plot(Rdata4(1,:), Rdata4(2,:),'r'); %Untwisted blade Lower
            % In this order so that the legend is clean
    end 
    
    %legend ('Twisted Blade','Untwisted Blade');
    
    viscircles([0,0],R*2,'LineStyle','--','EdgeColor','b','LineWidth',.5); %Plot hub diameter  
    %plot([0,final.diameter], [0,0],'-g') % Plot Sweep line (linear for now)
    
    hold off  
    axis equal
    title('Blade plot');
    xlabel('x (Span)'); % x-axis label
    ylabel('y (Chord)'); % y-axis label
    
%     str = {['\bf RPM: ' num2str(final.rpm)], ['\bf Diameter [m]: ' num2str(final.diameter)],...
%         ['\bf Power [W]: ' num2str(final.Power)]};
%     annotation('textbox', [0.59 0.125 0.3 0.15],...
%             'BackgroundColor','white', 'String', str)
%     
end
    
    