% DRAG COEFFICIENT LOOKUP FUNCTION
%     Called after lift_coeff_lookup function
%     Assume drag bucket increases c_d 10 fold
% INPUTS:
%     c_l: Coefficient of lift
%     Re: Reynolds number
%     airfoil: airfoil type by calling name
%     stall_condition: 1 = yes, 0 = no
%  OUTPUTS:
%     c_d: Coefficient of drag
%
%  NOTES:
%     Remember tables query by column whereas rotors are represented by
%     rows


function [c_d] = drag_coeff_lookup(c_l,alpha,Re,station_airfoil,stall_condition)

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


        % Loop through Re data sets to get cl_min/max and alpha_min/max for each curve
        % Taken from c_l look up regime. To find endpoints of data set by
        % quering c_l
        for i=1:length(Re_curves_count.(airfoil_list{j}))
            % Table lookup for CL_max & CL_min @ Re list (capital because
            % Xfoil output header for cl is "CL")
            
            CD_max_count.(airfoil_list{j})(i)  =       max(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).CD(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).Re==Re_curves_count.(airfoil_list{j})(i)));
            CD_min_count.(airfoil_list{j})(i)  =       min(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).CD(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).Re==Re_curves_count.(airfoil_list{j})(i)));
        
            % Find range of supplied data (can include post stall data)
            alpha_max_Re_count.(airfoil_list{j})(i)      =       max(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).alpha(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).Re==Re_curves_count.(airfoil_list{j})(i)));
            alpha_min_Re_count.(airfoil_list{j})(i)      =       min(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).alpha(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).Re==Re_curves_count.(airfoil_list{j})(i)));

        end
end

% Calculation/Interpolation Routine Start:
    % Switch to station_airfoil dynamic field reference over airfoil_table
    
    
    % Determine size of Re variable to loop through (considering each Re has an
    % airfoil associated with it)
    [m,n] = size(Re);

    for k = 1:n %Looping azimuths
        for l = 1:m %Looping stations

        % Find 2 closest Re sets, either bounding or sequential
            Re_sort=sort(abs(Re(l,k)-Re_curves_count.(station_airfoil{l})));
            Re_idx1 = find(abs(Re(l,k)-Re_curves_count.(station_airfoil{l}))==Re_sort(1));
            
           % Catch if there are two data sets that are the same difference from
           % the Re(l,k). Do idx2 first since idx1 is over-written
           if size(Re_idx1,1) > 1 
           Re_idx2 = Re_idx1(2); Re_idx1 = Re_idx1(1);
           else
           Re_idx2 = find(abs(Re(l,k)-Re_curves_count.(station_airfoil{l}))==Re_sort(2));
           end
            
            Re_1 = Re_curves_count.(station_airfoil{l})(Re_idx1(1));
            Re_2 = Re_curves_count.(station_airfoil{l})(Re_idx2(1));
            
            
% Station alpha is WITHIN range of bounding alpha's for Re_1 and Re_2 data sets
            % If within range, use direct look up
            if alpha(l,k) <= deg2rad(alpha_max_Re_count.(airfoil_list{j})(Re_idx1(1))) && ...
               alpha(l,k) <= deg2rad(alpha_max_Re_count.(airfoil_list{j})(Re_idx2(1))) && ...
               alpha(l,k) >= deg2rad(alpha_min_Re_count.(airfoil_list{j})(Re_idx1(1))) && ...
               alpha(l,k) >= deg2rad(alpha_min_Re_count.(airfoil_list{j})(Re_idx2(1)))

           
               c_d_1 = interp1(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).alpha(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re_1),...
                       airfoil_table.(station_airfoil{l}).(station_airfoil{l}).CD(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re_1),...
                       rad2deg(alpha(l,k)),'pchip');

               c_d_2 = interp1(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).alpha(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re_2),...
                       airfoil_table.(station_airfoil{l}).(station_airfoil{l}).CD(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re_2),...
                       rad2deg(alpha(l,k)),'pchip');           

           % Interpolate for c_l, from airfoil data                  
              % c_d(l,k)  = interp1([Re_1, Re_2],[c_d_1, c_d_2], Re(l,k),'pchip');

            if Re(l,k)<Re_curves_count.(station_airfoil{l})(1) % If less than Re set, don't interpolate
                 % since this was predicting too small cd from pchip interp  
                c_d(l,k)  = c_d_1;
             else
                c_d(l,k)  = interp1([Re_1, Re_2],[c_d_1, c_d_2], Re(l,k),'pchip');
             end
   
% Station alpha is OUTSIDE range of bounding alpha's for Re_1 and Re_2 data sets
    % Assume this section is stalled, since automatically past c_l_max apply Lindenburg model stall model
            else
                
                
       % Lindenburg stall model
            % Loop to find c_d_AoA = 0 @ Re
            for i=1:length(Re_curves_count.(station_airfoil{l}))
                c_d_0_count = []; % Reset variable for each loop. (Different airfoils have different dimensions)
                c_d_0_count(i) = interp1(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).alpha(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re_curves_count.(station_airfoil{l})(i)),...
                                         airfoil_table.(station_airfoil{l}).(station_airfoil{l}).CD(airfoil_table.(station_airfoil{l}).(station_airfoil{l}).Re==Re_curves_count.(station_airfoil{l})(i)),...
                                         0,'pchip');
            end
            
            c_d_90 = 1.98; % Flat plate 2D
            c_d_0 = interp1(Re_curves_count.(station_airfoil{l}),c_d_0_count, Re(l,k),'pchip');
            
            % Shift alpha to be within 0 -> 180 range. Call it "alpha_adjusted"
            % Assume that when stall_condition = -1 AND data is out of
            % range, alpha < 0
                if stall_condition(l,k) == -1 && alpha(l,k) < 0
                    alpha_adjusted = alpha(l,k) + pi;
                else
                    alpha_adjusted = alpha(l,k);
                    %alpha_adjusted = pi - alpha(l,k);
                end
                    
            % c_d
            c_n = c_d_90*(sin(alpha_adjusted))./(0.56+0.44*sin(alpha_adjusted));
            c_t = 0.5*c_d_0*cos(alpha_adjusted);
            c_d(l,k) = c_n*sin(abs(alpha(l,k)))+c_t*cos(abs(alpha(l,k)));

            
            % If c_d (stalled, OUTSIDE range) is less than the maximum c_d in closest data set
            % then adjust to match it
            if c_d(l,k) < CD_max_count.(airfoil_list{j})(Re_idx1(1))
                c_d(l,k)  = CD_max_count.(airfoil_list{j})(Re_idx1(1));
            end
            

            
        %% Linear stall model (max c_d at +/- pi)
            %                  if abs(alpha(l,k)) <= pi/2
            %                     c_d(l,k) = c_d(l,k)+((pi/2).*abs(alpha(l,k))); % second part here is flat plate drag ((cd=1)/(pi/2))*alpha in rads)                                     
            %                  else
            %                     c_d90 = 1.9; c_d180 = 0.08; 
            %                     c_d(l,k) = -(c_d90-c_d180)/(pi-(pi/2))*(abs(alpha(l,k))+(-(c_d90-c_d180)/(pi-(pi/2)))*(pi/2))-c_d180;
            %                  end
            %                  
            %                  if c_d(l,k) > 1.9
            %                     c_d(l,k) = 1.9;
            %                  end
end 
            
       % c_d is less than zero     
            if c_d(l,k) < 0
            c_d(l,k) = 0.05;
            end
        end
    end
end