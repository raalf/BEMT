% MOMENT COEFFICIENT LOOKUP FUNCTION
%     Called after lift_coeff_lookup function. Uses c_l to query for c_m
% INPUTS:
%     c_l: Coefficient of lift
%     alpha: amgle of attack
%     Re: Reynolds number
%     airfoil: airfoil type by calling name
%     stall_condition: 1 = yes, 0 = no
%  OUTPUTS:
%     c_m: Coefficient of drag
%
%  NOTES:
%     Remember tables query by column whereas rotors are represented by
%     rows


function [c_m] = moment_coeff_lookup(alpha,Re,station_airfoil,stall_condition)

% Collection Routine Start:

        % Identify unique airfoils stored in var airfoil (multiple rows for
        % multiple rotors). airfoil_list is only used so that data from
        % each airfoil is collected only ONCE. Do not use below collection
        % sequence.
    airfoil_list = unique(station_airfoil);             % Identify unique airfoils stored in var airfoil


for j = 1:length(airfoil_list)
  % Load airfoil polars
        %airfoil_table.(airfoil_list{j})    =       load(strjoin(strcat('Rotor Analysis Code\BEMT Module\Airfoil Database\',airfoil_list(j),'.mat')));
        airfoil_table.(airfoil_list{j})     =       load(airfoil_list{j});
        
        % Find # of Reynolds number data sets
        Re_curves_count.(airfoil_list{j})  =       unique(airfoil_table.(airfoil_list{j}).(airfoil_list{j}){:,'Re'});


        % Loop through Re data sets to get alpha_min/max for each curve
        % Taken from c_l look up regime. To find endpoints of data set by
        % quering c_l
        for i=1:length(Re_curves_count.(airfoil_list{j}))
            % Table lookup for CL_max & CL_min @ Re list (capital because
            % Xfoil output header for cl is "CL")
            CL_max_count.(airfoil_list{j})(i)  =       max(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).CL(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).Re==Re_curves_count.(airfoil_list{j})(i)));
            CL_min_count.(airfoil_list{j})(i)  =       min(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).CL(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).Re==Re_curves_count.(airfoil_list{j})(i)));
        
            % alpha @ cl max/min and in [deg]
                row_max = airfoil_table.(airfoil_list{j}).(airfoil_list{j}).CL==CL_max_count.(airfoil_list{j})(i) & airfoil_table.(airfoil_list{j}).(airfoil_list{j}).Re==Re_curves_count.(airfoil_list{j})(i);
                row_min = airfoil_table.(airfoil_list{j}).(airfoil_list{j}).CL==CL_min_count.(airfoil_list{j})(i) & airfoil_table.(airfoil_list{j}).(airfoil_list{j}).Re==Re_curves_count.(airfoil_list{j})(i);
            alpha_at_c_l_max_count.(airfoil_list{j})(i)  = airfoil_table.(airfoil_list{j}).(airfoil_list{j}).alpha(row_max);
            alpha_at_c_l_min_count.(airfoil_list{j})(i)  = airfoil_table.(airfoil_list{j}).(airfoil_list{j}).alpha(row_min);
            
        end
end

% Calculation/Interpolation Routine Start:
    % Switch to station_airfoil dynamic field reference over airfoil_table
    
    
    % Determine size of Re variable to loop through (considering each Re has an
    % airfoil associated with it)
    [m,n] = size(Re);

    for k = 1:n %Looping azimuths
        for l = 1:m %Looping stations


% NOT Stalled
        if stall_condition(l,k) == 0    

            % Re is exactly one of the data points
                if any(Re(l,k)==Re_curves_count.(station_airfoil{l})) == 1;
                
                   c_m(l,k) = interp1(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).alpha(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re(l,k)),...
                        airfoil_table.(station_airfoil{l}).(station_airfoil{l}).CM(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re(l,k)),...
                        rad2deg(alpha(l,k)),'pchip');
                    
            % Re is lower than Re_min.
                elseif Re(l,k) <= min(Re_curves_count.(station_airfoil{l}))
                    
                    c_m(l,k)  = interp1(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).alpha(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==min(Re_curves_count.(station_airfoil{l}))),...
                                airfoil_table.(station_airfoil{l}).(station_airfoil{l}).CM(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==min(Re_curves_count.(station_airfoil{l}))),...
                                rad2deg(alpha(l,k)),'pchip');
                    
            % Re is higher than Re_max.
                elseif Re(l,k)>= max(Re_curves_count.(station_airfoil{l}))
                    
                    c_m(l,k)  = interp1(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).alpha(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==max(Re_curves_count.(station_airfoil{l}))),...
                                airfoil_table.(station_airfoil{l}).(station_airfoil{l}).CM(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==max(Re_curves_count.(station_airfoil{l}))),...
                                rad2deg(alpha(l,k)),'pchip');
                    
            % Re within Re_min and Re_max (non inclusive). Use bounding Re's.
                else
                    % From Closest bounding Re's
                    Re_upper  = Re_curves_count.(station_airfoil{l})(Re_curves_count.(station_airfoil{l})>=Re(l,k));
                    Re_lower  = Re_curves_count.(station_airfoil{l})(Re_curves_count.(station_airfoil{l})<=Re(l,k));
                    
                    c_m_upper = interp1(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).alpha(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re_upper(1)),...
                                airfoil_table.(station_airfoil{l}).(station_airfoil{l}).CM(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re_upper(1)),...
                                rad2deg(alpha(l,k)),'pchip');
                    
                    c_m_lower = interp1(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).alpha(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re_lower(end)),...
                                airfoil_table.(station_airfoil{l}).(station_airfoil{l}).CM(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re_lower(end)),...
                                rad2deg(alpha(l,k)),'pchip');
                    
                    c_m(l,k)  = interp1([Re_upper(1), Re_lower(end)],[c_m_upper, c_m_lower], Re(l,k));
                end
    
            
% Stalled
        else
                
        % Lindenburg stall model
           % alpha_min/max [deg] @ Re for current airfoil. (sequence from cl look up function) [deg]
            alpha_at_c_l_max   =   interp1(Re_curves_count.(station_airfoil{l}),alpha_at_c_l_max_count.(station_airfoil{l}),Re(l,k),'pchip');
            alpha_at_c_l_min   =   interp1(Re_curves_count.(station_airfoil{l}),alpha_at_c_l_min_count.(station_airfoil{l}),Re(l,k),'pchip');
            
       
            alpha_high_S = deg2rad(alpha_at_c_l_max);
            alpha_low_S  = deg2rad(alpha_at_c_l_min);
            
            c_d_90 = 1.98; % Flat plate 2D
           
            % Shift alpha to be within 0 -> 180 range. Call it "alpha_adjusted"       
            % Use also to identify stall condition
                if alpha(l,k) < alpha_low_S
                    alpha_adjusted = alpha(l,k) + pi;
                else
                    alpha_adjusted = pi - alpha(l,k);
                end

                alpha_adjusted = abs(alpha(l,k));
                
            %c_m
            c_n = c_d_90*(sin(alpha_adjusted))./(0.56+0.44*sin(alpha_adjusted));
            c_m(l,k) = -c_n*(0.25 - 0.175*(1-(2*alpha_adjusted)./pi));

            % correct for when moment coefficient is negative
            if alpha(l,k)<0
            c_m = -c_m;
            end
            
        end
    end
end