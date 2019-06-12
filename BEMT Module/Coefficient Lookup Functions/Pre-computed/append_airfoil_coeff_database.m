% Aerodynamic coefficient database append

% Appends database entries to the 'master' database that contains all
% generated airfoil data and metadata

% Airfoil data requirements
    % at least 2 coefficients (assumed to be + and - stall limits)
    % single cl max and cl min
    
% '<airfoilname>_<AoAresolution>'

function airfoil_coeff_database = append_airfoil_coeff_database(options,airfoil,airfoil_name,airfoil_coeff_database)
    sorted_coeffs =     []; % Initialize
    Re_count      =     unique(airfoil.Re(airfoil.Re<=options.RErange_max));
    Re_curve      =     unique(Re_count); % Number of Reynolds number curves

% Loop to generate data for each Reynolds number set. These sets are what
% is interpolated, using the methods below.
    for j = 1:size(Re_count,1)
        %%% Lindenburg stall model
            % Alpha @ CL_max and CL_max
                CL_max = max(airfoil.CL(airfoil.Re==Re_curve(j)));
                CL_min = min(airfoil.CL(airfoil.Re==Re_curve(j)));

                alpha_at_CL_max = deg2rad(airfoil.alpha(airfoil.CL == CL_max & airfoil.Re == Re_curve(j)));
                alpha_at_CL_min = deg2rad(airfoil.alpha(airfoil.CL == CL_min & airfoil.Re == Re_curve(j)));

            % Resolution and range of stalled performance data needed. Current
            % resolution is per 1 degree
                alpha_max_rad = deg2rad(max(airfoil.alpha(airfoil.Re==Re_curve(j))));
                alpha_min_rad = deg2rad(min(airfoil.alpha(airfoil.Re==Re_curve(j))));

                AoA_stalled = [-pi:deg2rad(options.AoAresolution):(alpha_min_rad-deg2rad(options.AoAresolution)), alpha_max_rad+deg2rad(options.AoAresolution):deg2rad(options.AoAresolution):pi]';

            % Adjusted (shifted) angle of attack for Lindenburg stall model: 0<alpha<180
            % so -AoA = AoA + pi;
                AoA_adjusted = AoA_stalled;
                AoA_adjusted(AoA_adjusted<0)=AoA_adjusted(AoA_adjusted<0)+pi;

            % CD @ alpha = 0    
                CD_0 = interp1(airfoil.alpha(airfoil.Re==Re_curve(j)),...
                              airfoil.CD(airfoil.Re==Re_curve(j)),...
                              0,'pchip'); % Interpolation

            % Other coefficients, for +/- 180 deg
                CN_SF = 0.90; % Normal force coefficient scaling factor (for "second" cl max)            
                CD_90 = 1.98; % Drag coefficient @ alpha = 90 [deg], Flat plate 2D 
                CN = CD_90.*(sin(AoA_adjusted))./(0.56+0.44.*sin(AoA_adjusted));
                CT = 0.5.*CD_0.*cos(AoA_adjusted);

                CL_stalled = CN_SF.*CN.*cos(AoA_adjusted)-CT.*sin(AoA_adjusted); % Lift
                CD_stalled = CN.*sin(abs(AoA_stalled))+CT.*cos(abs(AoA_stalled)); % Drag
                CM_stalled = -CN.*(0.25 - 0.175.*(1-(2.*AoA_adjusted)./pi)); % Moment

                % Since using AoA_adjusted, adjust CM for negative AoAs and symmetry
                idx = find(AoA_stalled<0);
                CM_stalled(idx) = -CM_stalled(idx);
                CM_stalled(idx) = flipud(CM_stalled(idx));
                
            % Scale stalled performance wrt to CL max @ Reynolds set. So that coeffs reduce to zero as Re -> 0    
            % (Not part of Lindenburg model)
                SF = CL_max/max(airfoil.CL(airfoil.alpha<20 & airfoil.alpha>0));
                CL_stalled = CL_stalled*SF;

            % Flatten out CL curve for some of the post stall angles
                idx_s_start = find(AoA_stalled>0,1); %Indices of first positive "stalled" AoA
                idx_s_end = find(rad2deg(AoA_stalled)>40,1); %Indices of AoA @ 40 deg, approximate location of second CLmax)
                % Greasy way of replacing entries in CL_stalled, with constant CL
                % in immediate post stall region, for smoother transition
                CL_stalled_temp = CL_stalled(idx_s_start:idx_s_end);
                CL_stalled_temp(CL_stalled_temp<0.8) = 0.8 ;
                CL_stalled(idx_s_start:idx_s_end) = CL_stalled_temp;
    
    %-% Organize data pre-interpolation
    % Append existing data to include full range of angles and coefficients
        v_CL = [CL_stalled; airfoil.CL(airfoil.Re==Re_curve(j))];
        v_CD = [CD_stalled; airfoil.CD(airfoil.Re==Re_curve(j))];
        v_CM = [CM_stalled; airfoil.CM(airfoil.Re==Re_curve(j))];
        v_alpha = [rad2deg(AoA_stalled); airfoil.alpha(airfoil.Re==Re_curve(j))];
        v_Re = ones(size(v_alpha,1),1).*Re_curve(j);

    % Sorted coefficients for airfoil, including stalled angles of attack.
    % Same format as airfoil structure
    % sorted_coeffs = [Reynolds number, Angle of attack, CL, CD, CM]
        sorted_coeffs = [sorted_coeffs; sortrows([ v_Re v_alpha  v_CL  v_CD  v_CM],2)];
    end
    
% For lift and moment, append hypothetical Re=0 set (all zeros) for coarse low Re estimates   
    lth = length(sorted_coeffs(sorted_coeffs(:,1)==Re_curve(1))); % Size of smallest Re set
    sorted_coeffs = [[zeros(lth,1) sorted_coeffs(1:lth,2) zeros(lth,1) sorted_coeffs(1:lth,4) zeros(lth,1)];sorted_coeffs];
      
% Create grid of data on which to interpolate over
    [REmesh,AoAmesh] = meshgrid(0:options.REresolution:options.RErange_max,-180:options.AoAresolution:180);
    
% scatteredInterpolant method (extrapolation included)
    FCL = scatteredInterpolant(sorted_coeffs(:,1),sorted_coeffs(:,2),sorted_coeffs(:,3),'linear','nearest');
    CLmesh = FCL(REmesh,AoAmesh);
    
    FCD = scatteredInterpolant(sorted_coeffs(:,1),sorted_coeffs(:,2),sorted_coeffs(:,4),'linear','nearest');
    CDmesh = FCD(REmesh,AoAmesh);
    
    FCM = scatteredInterpolant(sorted_coeffs(:,1),sorted_coeffs(:,2),sorted_coeffs(:,5),'linear','nearest');
    CMmesh = FCM(REmesh,AoAmesh);
    
% griddata method
%      CLmesh = griddata(sorted_coeffs(:,1),sorted_coeffs(:,2),sorted_coeffs(:,3),REmesh,AoAmesh,'linear');
%      CDmesh = griddata(sorted_coeffs(:,1),sorted_coeffs(:,2),sorted_coeffs(:,4),REmesh,AoAmesh,'linear');
%      CMmesh = griddata(sorted_coeffs(:,1),sorted_coeffs(:,2),sorted_coeffs(:,5),REmesh,AoAmesh,'linear');
%     
% Format & append data to master database with metadata
    rows = length(airfoil_coeff_database);
    airfoil_coeff_database(rows+1).raw_airfoil_data = array2table(sorted_coeffs,'VariableNames',{'Re' 'alpha' 'CL' 'CD' 'CM'});
    airfoil_coeff_database(rows+1).interp_coeffs    = array2table([reshape(REmesh,[],1) reshape(AoAmesh,[],1) reshape(CLmesh,[],1) reshape(CDmesh,[],1) reshape(CMmesh,[],1)],'VariableNames',{'Re' 'alpha' 'CL' 'CD' 'CM'});
    airfoil_coeff_database(rows+1).airfoil          = airfoil_name;
    airfoil_coeff_database(rows+1).RErange_max      = options.RErange_max;
    airfoil_coeff_database(rows+1).REresolution     = options.REresolution;
    airfoil_coeff_database(rows+1).AoAresolution    = options.AoAresolution;
    airfoil_coeff_database(rows+1).date             = datestr(datetime('now'));

% Save new entry in airfoil_coeff_database
save('../BEMT Module/Airfoil Databases/airfoil_coeff_database', 'airfoil_coeff_database')

    if strcmpi(options.toggle_coeff_plot,'on')
      figure();hold on;
        surf(REmesh,AoAmesh,CLmesh);
        plot3(airfoil.Re,deg2rad(airfoil.alpha),airfoil.CL,'b.');
        xlabel('Reynolds number');
        ylabel('Angle of attack [rads]');
        zlabel('Lift coefficient');
        hold off;

      figure();hold on;
        surf(REmesh,AoAmesh,CDmesh);
        plot3(airfoil.Re,deg2rad(airfoil.alpha),airfoil.CD,'b.');
        xlabel('Reynolds number');
        ylabel('Angle of attack [rads]');
        zlabel('Lift coefficient');
        hold off;

      figure();hold on;
        surf(REmesh,AoAmesh,CMmesh);
        plot3(airfoil.Re,deg2rad(airfoil.alpha),airfoil.CM,'b.');
        xlabel('Reynolds number');
        ylabel('Angle of attack [rads]');
        zlabel('Lift coefficient');
        hold off;
    end
end
