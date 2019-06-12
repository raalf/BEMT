% find_zero_lift_angle_airfoil
% Finds zero lift angle for cambered airfoil given coefficient data
% INPUTS:
%     Re: Reynolds Number
%     airfoil: airfoil name (char)
% OUTPUTS:
%   Beta_zero: Zero lift angle of attack. Negative is tip down, positive is
%   tip up (Positive CW)
% 
% Description:
%     Interpolates c_l = 0 for all Re curves in data
%     Interpolates new array of c_l's across Re's for Beta_zero and input Re
%
% NOTES:
%     Rows are stations, columns are azimuths


function [Beta_zero] = find_zero_lift_angle_airfoil(Re,section_airfoil)


% Collection Routine Start:

        % Identify unique airfoils stored in var airfoil (multiple rows for
        % multiple rotors). airfoil_list is only used so that data from
        % each airfoil is collected only ONCE. Do not use below collection
        % sequence.
        airfoil_list = unique(section_airfoil); 

% Find Beta_zero for each airfoil's Re data sets (# Re sets = # beta zeros)
    for j = 1:length(airfoil_list) % Loop through airfoils to find Beta_0_count for each

        % Load airfoil polars
        %airfoil_table.(airfoil_list{j})    =       load(strjoin(strcat('Rotor Analysis Code\BEMT Module\Airfoil Database\',airfoil_list(j),'.mat')));
        airfoil_table.(airfoil_list{j})     =       load(airfoil_list{j});
        
        % Find # of Re data sets per airfoil
        Re_curves_count.(airfoil_list{j})  =       unique(airfoil_table.(airfoil_list{j}).(airfoil_list{j}){:,'Re'});

        for i=1:length(Re_curves_count.(airfoil_list{j})) % Loop through Re's Beta_0
            
            CL_count            =       airfoil_table.(airfoil_list{j}).(airfoil_list{j}).CL(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).Re==Re_curves_count.(airfoil_list{j})(i));
            alpha_count         =       airfoil_table.(airfoil_list{j}).(airfoil_list{j}).alpha(airfoil_table.(airfoil_list{j}).(airfoil_list{j}).Re==Re_curves_count.(airfoil_list{j})(i));
            
            % Find minimum c_l, find points on either side by index
            [~,idx] = min(abs(CL_count));
            % Statement to make sure that adjacent points are selected
            % incase 0 lift angle on 2d airfoil data is (for some weird reason) an endpoint
            
            if idx == length(CL_count)
                CL_count = [CL_count(idx-2),CL_count(idx-1),CL_count(idx)];
                alpha_count = [alpha_count(idx-2),alpha_count(idx-1),alpha_count(idx)];
            elseif idx == 1
                CL_count = [CL_count(idx),CL_count(idx+1),CL_count(idx+2)];
                alpha_count = [alpha_count(idx),alpha_count(idx+1),alpha_count(idx+2)];
            else
                CL_count = [CL_count(idx-1),CL_count(idx),CL_count(idx+1)];
                alpha_count = [alpha_count(idx-1),alpha_count(idx),alpha_count(idx+1)];
            
            end
            
            Beta_0_count.(airfoil_list{j})(i)   =       deg2rad(interp1(CL_count,alpha_count,0,'pchip')); % CAN DO THIS IN ONE PASS???          
        end        
    end
  
% Interpolation routine for each station Re from database:
    % Switch to section_airfoil dynamic field reference over airfoil_table
    
    [m,n] = size(Re);

    % Loop for each Re element to find Beta_zero for each element according to
    % airfoil

    for k = 1:n %Looping azimuths
        for l = 1:m % Looping stations
            % Find Beta_0 for each station, for each Re (and for each rotor if vectorized)
            % Station airfoil is selected by Beta_0_count.(airfoil{m}) term
            
            if Re(l,k)< min(Re_curves_count.(section_airfoil{l})) % Re is lower than range, set to lowest beta_zero
                Beta_zero(l,k) = - Beta_0_count.(section_airfoil{l})(1);
            elseif Re(l,k)> max(Re_curves_count.(section_airfoil{l})) % Re is higher than range, set to highest beta_zero
                max_num = size(Beta_0_count.(section_airfoil{l}),2);
                Beta_zero(l,k) = - Beta_0_count.(section_airfoil{l})(max_num);
                
            else % Re is inbetween datasets. (has a lower AND higher Re dataset) 
%                 
%                 [index,index] = min(abs(Re(l,k)-Re_curves_count.(section_airfoil{l})));
%                 Re_data_1 = Re_curves_count.(section_airfoil{l})(index);
%                 
%                 if Re_data_1 <= Re(l,k)
%                    Re_data_2 = Re_curves_count.(section_airfoil{l})(index+1);
%                    Beta_zero(l,k) = deg2rad(interp1([Re_data_1,Re_data_2],[Beta_0_count.(section_airfoil{l})(index),Beta_0_count.(section_airfoil{l})(index+1)],Re(l,k), 'pchip'));
%                    
%                 elseif Re_data_1 >= Re(l,k)
%                    Re_data_2 = Re_curves_count.(section_airfoil{l})(index-1);
%                    Beta_zero(l,k) = deg2rad(interp1([Re_data_1,Re_data_2],[Beta_0_count.(section_airfoil{l})(index),Beta_0_count.(section_airfoil{l})(index-1)],Re(l,k), 'pchip'));
%                    
%                 else % if for some reason, above two sequences fail. query all
%                      Beta_zero(l,k) =  deg2rad(interp1(Re_curves_count.(section_airfoil{l}),Beta_0_count.(section_airfoil{l}),Re(l,k), 'pchip'));
%                 end
%                 

                Beta_zero(l,k) =  - deg2rad(interp1(Re_curves_count.(section_airfoil{l}),Beta_0_count.(section_airfoil{l}),Re(l,k), 'pchip'));
                
            end
           
        end
    end
end