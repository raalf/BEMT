% LIFT COEFFICIENT LOOKUP FUNCTION
% Calculates lift coefficient from AoA and checks if stalled
% Outputs operating condition, stall or normal working state as binary output.
% Stall model: Airfoils operating past cl_max are penalized 80%
% (McCormick) and then linearly approach cl = 0 @ 90 degress AoA.
%
% Works for 2D stations on wings, rotors, control surfaces, etc.
%
% ASSUMPTIONS:
%     Data supplied for each Re curve MUST AT LEAST INCLUDE cl min AND cl max
%
% INPUTS:
%     a_0       = lift curve slope (normally = 2*pi)
%     alpha     = station angle of attack [rad]
%     alpha_zero= zero lift angle of attack for cambered airfoils [rad] (positive  clockwise. wrt to zero lift axis)
%     Re        = Reynolds number
%     airfoil   = airfoil name (input as a string cell)
%
% OUTPUTS:
%     c_l       = Coefficient of lift
%     stall_condition(l,k): 1 = stalled, 0 = not stalled.
%
% COUPLED WITH:
%     Other aerodynamic coefficient functions in folder. See naming
%     convention. i.e. drag_coeff_lookup.m
%
% NOTES:
%     Rows are stations, columns are azimuths


function [c_l,stall_condition] = lift_coeff_lookup(alpha,Re,station_airfoil)
    
% Data collection and working state data identification routine:    
        % Identify unique airfoils stored in var airfoil (multiple rows for
        % multiple rotors). airfoil_list is only used so that data from
        % each airfoil is collected only ONCE. Do not use below collection
        % sequence.
        
        airfoil_list = unique(station_airfoil);           
        
    for j = 1:length(airfoil_list) % Loop through airfoils to collect AoA vs Cl max/min point @ each Re for each airfoil 
        
        % Load airfoil polars
        %airfoil_table.(airfoil_list{j})    =       load(strjoin(strcat('Rotor Analysis Code\BEMT Module\Airfoil Database\',airfoil_list(j),'.mat')));
        airfoil_table.(airfoil_list{j})    =       load(airfoil_list{j});
        
        
        % Find # of Reynolds number data sets
        Re_curves_count.(airfoil_list{j})  =       unique(airfoil_table.(airfoil_list{j}).(airfoil_list{j}){:,'Re'});

 
        % Loop through Re data sets to get AoA vs Cl max/min for each Re set
        % Also find range of supplied data, either unstalled or post stall
        for i=1:length(Re_curves_count.(airfoil_list{j}))
            % Table lookup for CL_max & CL_min @ Re list (capital because Xfoil output header for cl is "CL")
            CL_max_count.(airfoil_list{j})(i)  =       max(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).CL(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).Re==Re_curves_count.(airfoil_list{j})(i)));
            CL_min_count.(airfoil_list{j})(i)  =       min(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).CL(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).Re==Re_curves_count.(airfoil_list{j})(i)));
        
            % alpha @ cl max/min and in [deg]
            row_max = airfoil_table.(airfoil_list{j}).(airfoil_list{j}).CL==CL_max_count.(airfoil_list{j})(i) & airfoil_table.(airfoil_list{j}).(airfoil_list{j}).Re==Re_curves_count.(airfoil_list{j})(i);
            row_min = airfoil_table.(airfoil_list{j}).(airfoil_list{j}).CL==CL_min_count.(airfoil_list{j})(i) & airfoil_table.(airfoil_list{j}).(airfoil_list{j}).Re==Re_curves_count.(airfoil_list{j})(i);
            alpha_at_c_l_max_count.(airfoil_list{j})(i)  = airfoil_table.(airfoil_list{j}).(airfoil_list{j}).alpha(row_max);
            alpha_at_c_l_min_count.(airfoil_list{j})(i)  = airfoil_table.(airfoil_list{j}).(airfoil_list{j}).alpha(row_min);
            
            % Find range of supplied data (can include post stall data)
            alpha_max_Re_count.(airfoil_list{j})(i)      =       max(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).alpha(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).Re==Re_curves_count.(airfoil_list{j})(i)));
            alpha_min_Re_count.(airfoil_list{j})(i)      =       min(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).alpha(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).Re==Re_curves_count.(airfoil_list{j})(i)));
        
        end
    end 
      
% Calculation/Interpolation routine for each section:    
    
    % Determine size of Re variable to loop through (each Re should have an
    % airfoil associated with it from rotor file)
    [m,n] = size(Re);

    % Loop for each Re element to find 2D c_l according to station airfoil.
    for k = 1:n %Looping azimuths (columns) Note: airfoil_station doesn't change with azimuth
        for l = 1:m % Looping stations (rows)
       
        % Remember, alpha(l,k) is in [rad] while alpha data is [deg]! 
    
       % Find 2 closest Re sets, either bounding or sequential
           Re_sort=sort(abs(Re(l,k)-Re_curves_count.(station_airfoil{l})));
           Re_idx1 = find(abs(Re(l,k)-Re_curves_count.(station_airfoil{l}))==Re_sort(1));
            
           
           % Catch if there are two data sets that are the same difference from
           % the Re(l,k). Do idx2 first since idx1 is over-written
           if size(Re_idx1,1) > 1 
           Re_idx2 = Re_idx1(2); Re_idx1 = Re_idx1(1);
%           Re_idx3 = find(abs(Re(l,k)-Re_curves_count.(station_airfoil{l}))==Re_sort(2));
           else
           Re_idx2 = find(abs(Re(l,k)-Re_curves_count.(station_airfoil{l}))==Re_sort(2));
%           Re_idx3 = find(abs(Re(l,k)-Re_curves_count.(station_airfoil{l}))==Re_sort(3));
           end
           
           Re_1 = Re_curves_count.(station_airfoil{l})(Re_idx1(1));
           Re_2 = Re_curves_count.(station_airfoil{l})(Re_idx2(1));
 %          Re_3 = Re_curves_count.(station_airfoil{l})(Re_idx3(1));
 
% Station alpha is WITHIN range of bounding alpha's for Re_1 and Re_2 data sets
            % If within range, airfoil working state is determined by data
            if alpha(l,k) <= deg2rad(alpha_max_Re_count.(station_airfoil{l})(Re_idx1(1))) && ...
               alpha(l,k) <= deg2rad(alpha_max_Re_count.(station_airfoil{l})(Re_idx2(1))) && ...
               alpha(l,k) >= deg2rad(alpha_min_Re_count.(station_airfoil{l})(Re_idx1(1))) && ...
               alpha(l,k) >= deg2rad(alpha_min_Re_count.(station_airfoil{l})(Re_idx2(1)))
                
               c_l_1 = interp1(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).alpha(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re_1),...
                       airfoil_table.(station_airfoil{l}).(station_airfoil{l}).CL(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re_1),...
                       rad2deg(alpha(l,k)),'pchip');


               c_l_2 = interp1(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).alpha(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re_2),...
                       airfoil_table.(station_airfoil{l}).(station_airfoil{l}).CL(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re_2),...
                       rad2deg(alpha(l,k)),'pchip');


%               c_l_3 = interp1(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).alpha(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re_3),...
%                       airfoil_table.(station_airfoil{l}).(station_airfoil{l}).CL(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re_3),...
%                       rad2deg(alpha(l,k)),'pchip');

             % Interpolate/extrapolate for c_l, from airfoil data
                % uncomment if/else block if c_l for Re<supplied Re's is
                % producing incorrect estimate.
             
%             if Re(l,k)<Re_curves_count.(station_airfoil{l})(1) % If less than Re set, don't interpolate
                 % since this was predicting -cl's from pchip interp when
                 % aoa around 0   
%               c_l(l,k)  = c_l_1;
%               OR
                c_l(l,k)  = interp1([Re_1, Re_2],[c_l_1, c_l_2], Re(l,k),'pchip');
                if c_l(l,k)< 0 && alpha(l,k) >= 0 % If negative c_l with +AoA
                   c_l(l,k) = 0;
                elseif c_l(l,k)< -1 && alpha(l,k) < 0 % If negative c_l is <-1
                   c_l(l,k)  = c_l_1;
                end 
%             else
                c_l(l,k)  = interp1([Re_1, Re_2],[c_l_1, c_l_2], Re(l,k),'pchip');
%               c_l(l,k)  = interp1([Re_1, Re_2, Re_3],[c_l_1, c_l_2, c_l_3], Re(l,k),'pchip'); 
%             end

                if isnan(c_l(l,k)) == 1
                    disp('Lift coefficient is NaN at the following conditions (Re;alpha;row;column) \n');
                    Re(l,k)
                    alpha(l,k)
                    l
                    k
                    disp('check supplied airfoil data');
                    pause
                end
                
                
                % Check if stalled by determining max/min c_l in data
                % alpha_min/max [deg] @ Re for current airfoil. (from cl_max/min) [deg]
                alpha_at_c_l_max   =   interp1(Re_curves_count.(station_airfoil{l}),alpha_at_c_l_max_count.(station_airfoil{l}),Re(l,k),'pchip');
                alpha_at_c_l_min   =   interp1(Re_curves_count.(station_airfoil{l}),alpha_at_c_l_min_count.(station_airfoil{l}),Re(l,k),'pchip');
            
                % Apply stall condition
                if alpha(l,k) > alpha_at_c_l_max
                    stall_condition(l,k) = 1;
                elseif alpha(l,k) < alpha_at_c_l_min
                    stall_condition(l,k) = -1;
                else
                    stall_condition(l,k) = 0;
                end
                
% Station alpha is OUTSIDE range of bounding alpha's for Re_1 and Re_2 data sets
            % Assume that if outside, airfoil is STALLED    
            else

            % Linear Post stall model
                % Positive stall
                % c_l(l,k) = -(c_l_max/((pi/2)-deg2rad(alpha_at_c_l_max)))*(alpha(l,k)-deg2rad(alpha_at_c_l_max))+c_l_max; % (-c_l_max*SF) where SF = stalled scaling factor. Rest of equation is linear trend to 90deg flat plate 0 lift
                % Negative stall
                % c_l(l,k) = c_l_min * .8;
                
            %Lindenburg stall model
                    c_d_0_count = []; % Reset variable for each loop. (Different airfoils have different dimensions)
                for i=1:length(Re_curves_count.(station_airfoil{l}))
                    
                    % CD @ alpha = 0. c_d_0: drag coefficient at zero degrees
                    c_d_0_count(i) = interp1(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).alpha(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re_curves_count.(station_airfoil{l})(i)),...
                                     airfoil_table.(station_airfoil{l}).(station_airfoil{l}).CD(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re_curves_count.(station_airfoil{l})(i)),...
                                     0,'pchip');
                end
                
                alpha_at_c_l_max   =   interp1(Re_curves_count.(station_airfoil{l}),alpha_at_c_l_max_count.(station_airfoil{l}),Re(l,k),'pchip');
                alpha_at_c_l_min   =   interp1(Re_curves_count.(station_airfoil{l}),alpha_at_c_l_min_count.(station_airfoil{l}),Re(l,k),'pchip');
            
                alpha_high_S = deg2rad(alpha_at_c_l_max);
                alpha_low_S  = deg2rad(alpha_at_c_l_min);
                    
                % Shift alpha to be within 0 -> 180 range. Called "alpha_adjusted", so works with Lindenburg stall model       
                % Use also to identify stall condition
                if alpha(l,k) < 0 % alpha_low_S -> was giving errors when close to lower stall angle
                    alpha_adjusted = alpha(l,k) + pi;
                    stall_condition(l,k) = -1; % negative (nose down) stall
                else
                    stall_condition(l,k) = 1; % positive (nose up) stall
                    alpha_adjusted= alpha(l,k);
                end
                    
                % Drag coefficients for 0 and 90 deg positions
                c_d_0 = interp1(Re_curves_count.(station_airfoil{l}),c_d_0_count, Re(l,k),'pchip');                
                c_d_90 = 1.98; % Flat plate 2D
                
                % c_l
                c_n_SF = 0.85; % scale back c_n to reduce c_l(#2) max if too high
                
                c_n = c_d_90*(sin(alpha_adjusted))./(0.56+0.44*sin(alpha_adjusted));
                c_t = 0.5*c_d_0*cos(alpha_adjusted);
                c_l(l,k) = c_n_SF*c_n*cos(alpha_adjusted)-c_t*sin(alpha_adjusted);
                
                % Flatten out c_l curve near stall angle
                % 20 deg used as offset 
               if stall_condition(l,k) == 1 && alpha(l,k)<deg2rad(alpha_at_c_l_max+20) && c_l(l,k)< 0.8 % alpha(l,k)>deg2rad(alpha_at_c_l_max) - taken out because already checked with stall condition
                   c_l(l,k) = 0.8;
               end
% EDITS HERE %%%%%%%%%%%%%%%%%%%%%%%%
                % "Approximately" scale stall model for low Reynolds numbers
                % Equivalent c_lmax for this Reynolds number
                
                CL_max_scale = interp1(Re_curves_count.(station_airfoil{l}), CL_max_count.(station_airfoil{l}), Re(l,k),'pchip');
                
                SF = CL_max_scale/CL_max_count.(station_airfoil{l})(end);
                
                c_l(l,k) = c_l(l,k)*SF;
               
               
            end % End if for inside/outside range statement
            
        end % Looping stations
    end % Looping azimuths
end