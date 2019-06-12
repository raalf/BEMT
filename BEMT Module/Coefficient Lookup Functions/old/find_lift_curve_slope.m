% LIFT CURVE SLOPE CALCULATE FUNCTION
% INPUTS:
%     Re: Reynolds number
%     airfoil: airfoil type by calling name
%  OUTPUTS:
%     a_0: Lift Curve Slope

function [a_0]  =   find_lift_curve_slope(Re,airfoil)

    load(strjoin(strcat('Aero Module\Airfoil Database\',airfoil,'.mat')));
    airfoil_table       =       eval(char(airfoil));

    Re_curves_count     =       unique(airfoil_table{:,'Re'});
    
     for i=1:length(Re_curves_count)
                      %data_length            =       length(airfoil_table{airfoil_table.Re == Re_curves_count(i),{'Re'}});

         
            % Take Cl and Cd at 1/3 and 2/3 sections
            %c_l_high(i)            =       airfoil_table{airfoil_table.Re == Re_curves_count(i),data_length*(1/3),{'CL'}};
            
            c_l_max(i)             =       max(airfoil_table{airfoil_table.Re == Re_curves_count(i),{'CL'}});
            alpha_at_CL_max(i)     =       (airfoil_table{airfoil_table.Re == Re_curves_count(i) & airfoil_table.CL == c_l_max(i),{'alpha'}});

            c_l_min(i)             =       min(airfoil_table{airfoil_table.Re == Re_curves_count(i),{'CL'}});
            alpha_at_CL_min(i)     =       (airfoil_table{airfoil_table.Re == Re_curves_count(i) & airfoil_table.CL == c_l_min(i),{'alpha'}});

            if i>1 && Re_curves_count(i-1) > Re  %Exit after Re+1 is passed to save computation time
                Re_curves_count = Re_curves_count(1:i);
                break
            end 
        
     end
     
        c_l_max_interp         =       interp1(Re_curves_count,c_l_max,Re, 'pchip', 'extrap');
        alpha_at_clmax         =       interp1(Re_curves_count,alpha_at_CL_max,Re, 'pchip', 'extrap');
        
        c_l_min_interp         =       interp1(Re_curves_count,c_l_min,Re, 'pchip', 'extrap');
        alpha_at_clmin         =       interp1(Re_curves_count,alpha_at_CL_min,Re, 'pchip', 'extrap');
        

        a_0                    =       (c_l_max_interp - c_l_min_interp)/deg2rad((alpha_at_clmax - alpha_at_clmin));
        
end