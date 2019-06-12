% Inflow_small_angles_no_swirl
% Script to find radial inflow distribution along blade based on BET and
% momentum equivalence in hover and axial flight conditions, resulting in a
% quadratic equation for radial inflow. Swirl is not taken into account.
% Solution iterates for tiploss and rootloss factors, F, which reduce the
% effective thrust loading at the tip and root.

% General form of the inflow equation:
    % lambda = (Vrp_a + vi)./(omega.*R);

% Since for axial flight or hover, all dimensions of vars will be [n x 1]
% to reduce redundancy
    
F = ones(stations-1,1); checkF = 1; countF = 1; % Initializations for iterative conditions
lambda = (Vrp_a)./(omega.*R); % Initialize to lambda_inf for first pass of Re_mid calculation. Recall: lambda = (Vrp_a + vi)./(omega.*R);
lambda_inf = (Vrp_a)./(omega.*R); % Recall: lambda = (Vrp_a + vi)./(omega.*R);
negative_inflow_stations = ones(stations-1,1);


while checkF == 1 && countF < 50  % Iteration scheme for tiploss factor, F (need to iterate because f affects induced velocity)
        F_old           =       F;

        %lambda_c        =       Vrp_a/(sqrt(Ct/2); % Climb velocity ratio
        % climb velocity ratio, lambda_c, Vc/vh = Vrp_a/(sqrt(Ct/2)
        
        % Estimate zero lift angle based on section Re
        V_R             =       sqrt((Vrp_a+(lambda.*omega.*R)).^2+(omega_mid).^2);
        Re_mid          =       flow.rho.*(bsxfun(@times,V_R,chord_mid))./(flow.mu);  % Reynolds number
        
        if strcmpi(oper.toggle_visc,'on')
            beta_zero   =       find_zero_lift_angle_airfoil(Re_mid,airfoil_mid);
        else
            beta_zero   =       -oper.alpha_zero; 
        end
        
        a_0             =       oper.a_0;                                            % Lift curve slope [1/rad]      
        
        lambda          =       sqrt(((sigma_mid.*(a_0).*r_mid./(8.*F))-(lambda_inf./2)).^2+... % Tsodak approach
                                (sigma_mid.*(a_0).*(beta_mid+beta_zero).*r_mid.^2./(4.*F)))-...         
                                ((sigma_mid.*r_mid.*(a_0)./(8.*F))-(lambda_inf./2));      
                                                    
%        lambda2          =       sqrt(((sigma_mid.*(a_0).*r_mid./(16.*F))-(lambda_inf./2)).^2+... % Leishman approach
%                                 (sigma_mid.*(a_0).*(beta_mid+beta_zero).*r_mid./(8.*F)))-...         
%                                 ((sigma_mid.*r_mid.*(a_0)./(16.*F))-(lambda_inf./2)); 

    % Negative inflow, set lambda = 0 (violates momentum theory). Effect of
    % negative AoA will be considered w/ profile drag in final BEMT
    % summation. (Also see note below)
    
        if (any(lambda < 0) == 1) || (any(isreal(lambda)) == 0)
           [row,column] = find((imag(lambda)~=0)|(lambda<0));
           negative_inflow_stations(row) = 0;
        end
        
        lambda = lambda.*negative_inflow_stations;
        
    % Tiploss function (Prandtl. Iterative since function of inflow ratio)        
            f_t             =       (B/2).*((1-r_mid)./(r_mid.*(lambda./r_mid)));
            f_r             =       (B/2).*((r_mid)./((1-r_mid).*(lambda./r_mid)));
            F               =       (2/pi).*(acos(exp(-f_t)))*(2/pi).*(acos(exp(-f_r)));        

        checkF          =       any(any(abs((F - F_old)./F)>0.005)==1);
        countF          =       countF + 1;
end







%         if any((beta_mid+beta_zero)< 0) ==1
%             % Negative (angle of attack + zero lift aoa) sometimes yields a negative
%             % "((sigma_mid.*(a_0).*r_mid./(8.*F))-(lambda_inf./2)).^2+(sigma_mid.*(a_0).*(beta_mid+beta_zero).*r_mid.^2./(4.*F))"
%             % term in the inflow equation and then an imaginary component from (sqrt(-))
%             % Therefore commands below make second term in sqrt = 0 so that inflow is real.
%             
%             [row,~] = find((beta_mid+beta_zero)<0);
%             beta_mid(row) = abs(beta_zero(row));
%         end        