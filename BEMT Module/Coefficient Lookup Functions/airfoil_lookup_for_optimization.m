% Function used to speed up optimization look up
% Airfoil coefficient data must be precomputed for all AoAs and Re's 
% and organized properly. File name is: airfoil in optimization folde 


function [c_l,c_d,c_m] = airfoil_lookup_for_optimization(AoA,Re,airfoil,coeff_data)

    % Preallocation
    c_l = zeros(size(AoA,1),size(AoA,2));
    c_d = zeros(size(AoA,1),size(AoA,2));
    c_m = zeros(size(AoA,1),size(AoA,2));

    % Round AoA to the nearest AoAresolution from options, for database
    AoA = round(rad2deg(AoA)./0.5)*0.5;

    % Round Reynolds number to the nearest REresolution from options, for database
    Re = round(Re./10000)*10000;
    
    % Find row from options.REresolution, options.AoAresolution and options.airfoil
    for i = 1:size(AoA,2)
        for j = 1:size(AoA,1)
            row = find(coeff_data.(airfoil{j})(:,1) == Re(j,i) & abs(coeff_data.(airfoil{j})(:,2) - AoA(j,i))<10^-6);
            c_l(j,i) = coeff_data.(airfoil{j})(row,3);
            c_d(j,i) = coeff_data.(airfoil{j})(row,4);
            c_m(j,i) = coeff_data.(airfoil{j})(row,5);
        end
    end
end