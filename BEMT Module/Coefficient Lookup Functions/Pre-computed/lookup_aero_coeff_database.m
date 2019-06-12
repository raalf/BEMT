% Airfoil coefficient lookup function.
% Uses airfoil database format
% Input:  options: Info about database resolution
%         database_coeffs: aero_coeff_database
%         AoA: angle of attack
%         Re: Reynolds number
%         airfoil: airfoil
% Output: [c_l,c_d,c_m]

function [c_l,c_d,c_m] = lookup_aero_coeff_database(options,airfoil_coeff_database,AoA,Re,airfoil)

% Preallocation
c_l = zeros(size(AoA,1),size(AoA,2));
c_d = zeros(size(AoA,1),size(AoA,2));
c_m = zeros(size(AoA,1),size(AoA,2));

    % Round AoA to the nearest AoAresolution from options, for database
    AoA = round(rad2deg(AoA)./options.AoAresolution)*options.AoAresolution;

    % Round Reynolds number to the nearest REresolution from options, for database
    Re = round(Re./options.REresolution)*options.REresolution;
    
    % Find row from options.REresolution, options.AoAresolution and options.airfoil
    for i = 1:size(AoA,2)
        for j = 1:size(AoA,1)
            row_index_in_database = find([airfoil_coeff_database.REresolution]' == options.REresolution &...
                [airfoil_coeff_database.AoAresolution]' == options.AoAresolution &...
                strcmp({airfoil_coeff_database.airfoil}', airfoil(j)));

            row_in_coeff_array = find(airfoil_coeff_database(row_index_in_database).interp_coeffs.Re == Re(j,i) &...
                abs(airfoil_coeff_database(row_index_in_database).interp_coeffs.alpha - AoA(j,i))<10^-6);
            
            c_l(j,i) = airfoil_coeff_database(row_index_in_database).interp_coeffs.CL(row_in_coeff_array);
            c_d(j,i) = airfoil_coeff_database(row_index_in_database).interp_coeffs.CD(row_in_coeff_array);
            c_m(j,i) = airfoil_coeff_database(row_index_in_database).interp_coeffs.CM(row_in_coeff_array);
            
            
        end
    end
end