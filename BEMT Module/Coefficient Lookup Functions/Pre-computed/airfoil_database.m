
% Perfomed BEFORE BEMT_Carroll, since BEMT_Carroll assumes that airfoil
% data is already compiled and accessible



function airfoil_database(blade,options) 

    % Verify that an airfoil_coeff_database exists
        if exist('airfoil_coeff_database.mat','file') ~= 2
            disp('No airfoil database found, creating new database...');

            % Verify that folder storing airfoil_coeff_database exists also
            if exist('Airfoil Databases','dir') ~= 7
                % Create folder
                mkdir '../BEMT Module' 'Airfoil Databases';
                % Add to path
                addpath('../BEMT Module/Airfoil Databases');
            end

            % Create and save airfoil_coeff_database
            airfoil_coeff_database   = struct([]);
            save('../BEMT Module/Airfoil Databases/airfoil_coeff_database', 'airfoil_coeff_database')
        end

    % Scan database for entries matching interpolation parameters and airfoil
        load('../BEMT Module/Airfoil Databases/airfoil_coeff_database', 'airfoil_coeff_database');
            
    % Load and compile airfoil list from rotor file
        airfoil_list = unique(blade.geometry.Airfoil);
        
    % Loop for each airfoil to check existing entry or generate new one
        for i = 1:1:size(unique(blade.geometry.Airfoil),1)
            airfoil_load  =  load(airfoil_list{i});
            airfoil       =  airfoil_load.(airfoil_list{i});
            logic         =  []; %Reset logic gate

           % Look for existing entries based on condition of 4 options parameters
            % Create matrix of logical conditions that meet requirements
            % (Quick but less elegant method)
            if isempty(airfoil_coeff_database) == 1 % If airfoil_coeff_database is empty
                logic = zeros(1,4);
            else
                logic(:,1) = double(contains({airfoil_coeff_database.airfoil}',airfoil_list(i))); %  airfoil name
                logic(:,2) = double(([airfoil_coeff_database.RErange_max]') == options.RErange_max); % RErange_max
                logic(:,3) = double(([airfoil_coeff_database.REresolution]') == options.REresolution); % REresolution
                logic(:,4) = double(([airfoil_coeff_database.AoAresolution]') == options.AoAresolution); % AoAresolution
            end

            % If entry exists
            if ismember(ones(1,4), logic,'rows') == 1
                % scan database for parameters
                fprintf('Existing entry found for: %s\n',  airfoil_list{i});
            else
                disp('No identical entries found, computing new dataset...');
                airfoil_coeff_database = append_airfoil_coeff_database(options,airfoil,airfoil_list{i},airfoil_coeff_database);
                fprintf('New entry added for: %s\n',  airfoil_list{i});
            end
        end

        disp('Database update complete.');
            
% Can use ismember(,,'rows'), hist(), unique() functions to condition in a single line.         
% C = union(airfoil_list(i) ,{airfoil_coeff_database.airfoil}') ;            
% [dum,ai] = ismember(airfoil_list(i),C) ;
% [dum,bi] = ismember({airfoil_coeff_database.airfoil}',C) ;
% [tf,loc] = ismember(ai,bi,'rows');
% % Alternative way of sorting, all in one conditional line
% if ismember([airfoil_list(i) options.RErange_max options.REresolution options.AoAresolution], airfoil_coeff_database,'rows') == 1          

end