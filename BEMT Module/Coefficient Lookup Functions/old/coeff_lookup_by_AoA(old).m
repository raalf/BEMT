% COEFFICIENT LOOKUP FUNCTION BY ANGLE OF ATTACK (VECTORIZED)
% Calculates lift and drag coefficients from direct look-up scheme and checks if stalled
% Outputs operating condition, stall or normal working state as binary output.
% Stall model(s):
% 1) From airfoil data (if provided).
% 2) Airfoils operating past cl_max are penalized 80% (McCormick) 
%    and then linearly approach cl = 0 @ 90 degress AoA.
%
% Works for 2D stations on wings, rotors, control surfaces, etc.
%
% INPUTS:
%     a_0       = lift curve slope (normally = 2*pi)
%     alpha     = station angle of attack
%     Re        = Reynolds number
%     airfoil   = airfoil name (input as a string cell)
%
% OUTPUTS:
%     c_l       = Coefficient of lift
%     c_d       = Coefficient of drag
%     stall_condition: 1 = stalled, 0 = not stalled.
%
% COUPLED WITH:
%     Other aerodynamic coefficient functions in folder. See naming
%     convention. i.e. drag_coeff_lookup.m
%
% NOTES:
%     Rows are stations, columns are azimuths


function [c_l,stall_condition] = coeff_lookup_by_AoA(alpha,Re,station_airfoil)
    
% Collection Routine Start:    
    
        % Identify unique airfoils stored in var airfoil (multiple rows for
        % multiple rotors). airfoil_list is only used so that data from
        % each airfoil is collected only ONCE. Do not use below collection
        % sequence.
        airfoil_list = unique(station_airfoil);           
        
     % Load airfoil polars    
    for j = 1:length(airfoil_list)  
        airfoil_table.(airfoil_list{j})    =       load(strjoin(strcat('Aero Module\Airfoil Database\',airfoil_list(j),'.mat')));    
        % Find # of Reynolds number data sets for station airfoil
        Re_curves_count.(airfoil_list{j})  =       unique(airfoil_table.(airfoil_list{j}).(airfoil_list{j}){:,'Re'});
    end 

% Calculation/Interpolation Routine Start:    
    % Determine size of alpha variable to loop through (assuming each Re has an
    % airfoil associated with it)
    [m,n] = size(alpha);

    % Loop for each Re element to find 2D cl according to station airfoil.
    for k = 1:n %Looping azimuths
        for l = 1:m % Looping stations
       
            
            
     % Interpolate over all alpha and Re data.
     % IS THIS VALID????
     
     Re_interp = airfoil_table.(station_airfoil{m,n}).(station_airfoil{m,n}).Re;
     alpha_interp = airfoil_table.(station_airfoil{m,n}).(station_airfoil{m,n}).alpha;
     c_l_interp = airfoil_table.(station_airfoil{m,n}).(station_airfoil{m,n}).CL;
     
     
     
     c_l = interp2(meshgrid(Re_interp),meshgrid(alpha_interp),meshgrid(c_l_interp),Re(m,n),alpha(m,n));
     
     
     
     interp2(Re_interp,alpha_interp,c_l_interp,Re(m,n),alpha(m,n))
     
        % Within Re Range 
                % c_l max/min @ current Re for current airfoil
                c_l_max   =   interp1(Re_curves_count.(station_airfoil{l}),CL_count_max.(station_airfoil{l}),Re(l,k),'pchip');
                c_l_min   =   interp1(Re_curves_count.(station_airfoil{l}),CL_count_min.(station_airfoil{l}),Re(l,k),'pchip');

            % check if c_l from a_0    
                if c_l(l,k) > c_l_max
                    stall_condition(l,k) = 1; % positive "stalled"                
                    c_l(l,k) = (-c_l_max*.85)/((pi/2)-(c_l_max/(2*pi)))*((c_l(l,k)/(2*pi))-(pi/2)); % (-c_l_max*SF) where SF = stalled scaling factor. Rest of equation is linear trend to 90deg flat plate 0 lift 
                    %c_l(k,l) = c_l_max *.8;
                    
                    if c_l(l,k) < 0
                        c_l(l,k) = 0;
                    end
                elseif c_l(l,k) < c_l_min  % negative "stalled"
                    stall_condition(l,k) = -1;
                    c_l(l,k) = c_l_min * .8;
                else
                    stall_condition(l,k) = 0; % not stalled
                end 
        end
    end
end