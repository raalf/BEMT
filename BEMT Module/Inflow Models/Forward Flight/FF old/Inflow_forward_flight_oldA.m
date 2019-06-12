% Script: Case 5
% Inflow_forward_flight  

    mu      =       Vrp_p./(omega.*R); % Also equal to mu_x, tipspeed ratio
    mu_z    =       Vrp_a./(omega.*R);
    
  % BET to get initial Ct guess from BET
    vi = 0; ui = 0; 
    BETscript;       
    Ct_old = Ct;
    lambda_i_old = sqrt(Ct_old/2);

    % Numerical solution to the inflow equation from uniform momentum
    % theory with updating Ct including inflow adjustments.
    % Does not compute swirl component of inflow (ksi).
    
    count = 1; epsilon = 1; epsilonCt = 1;     % Initializations
    while epsilon > 0.0005 && count < 100 && epsilonCt > 0.0005
        
            lambda_old = lambda_i_old + (Vrp_a./(omega.*R));
        
        % Momentum Theory for updated inflow and Ct
            %lambda_i_old = (Ct_old./(2*sqrt(mu.^2+lambda_i_old.^2)));
            
        % N-R iteration method
            f          =   lambda_old - mu.*tan(AoA_rad)-(Ct_old./(2*sqrt(mu.^2+lambda_old.^2)));
            f_prime    =   1 + (Ct_old./2).*(mu.^2+lambda_old.^2).^(-3/2).*lambda_old;
            lambda_new =   lambda_old - (f/f_prime);

        % Fixed Point Method
        % lambda_new = mu.*tan(AoA_rad) + lambda_i;
         
        if isreal(lambda_new) == 0
            pause
            disp('Error in FF iteration scheme')
        end
        
        % Apply linear inflow model here
        lambda_i    =       (Ct_old./(2*sqrt(mu.^2+lambda_new.^2))); % Recalculate the lambda_i
        %lambda_i2    =       sqrt(Ct_old/2);

        chi         =       atan(mu/(mu_z+lambda_i));

        % Pitt and Peters inflow method
        kx          =       (15*pi/23)*tan(chi/2);
        ky          =       0;

        % Drees method
%         kx          =       (4/3).*(1 - cos(chi)-1.8.*(mu.^2))./sin(chi);
%         ky          =       -2.*mu;

        % lambda      =       lambda_i.*(1+kx.*(mid_span.*cos(azimuth))+ky.*(mid_span.*sin(azimuth)));
        lambda      =       lambda_i.*(1+kx.*(bsxfun(@times,mid_span,cos(azimuth)))+ky.*(bsxfun(@times,mid_span,sin(azimuth))));
        ksi         =       0;
        
        % Re-do BET for new Ctn 
        vi         =   (lambda.*omega.*R)-Vrp_a;
        BETscript;
        
        % Convergence criteria
        epsilon = abs((lambda_i - lambda_i_old)./lambda_i);
        epsilonCt = abs((Ct - Ct_old)./Ct);

        % Convergence for lambda_i (centre of rotor) and Ct
        convergence_plot_lambda_i(count,:) = lambda_i;
        convergence_plot_Ct(count,:) = Ct;
        count = count + 1;
        lambda_i_old = lambda_i;
        Ct_old = Ct;
    end

    
    
    
    
%% Hybrid inflow method using small angles for hover/axial flight
% 
%         F = ones(stations-1,1); checkF = 1; countF = 1;
%         while checkF == 1 && countF < 7                                 % Iteration scheme for tiploss factor, F (need to iterate because f affects induced velocity)
%         F_old           =       F;
%   
%         lambda_inf      =       (Vrp_a)./(omega.*R);                    % Hover or Climb. If hover, vrp_a = 0, so reduced so lambda_h 
%         lambda2          =       sqrt(((sigma_mid.*(a_0)./(16.*F))-...
%                                 (lambda_inf./2)).^2+(sigma_mid.*...
%                                 (a_0).*beta_mid.*mid_span./(8.*F)))...
%                                 -((sigma_mid.*(a_0)./(16.*F))...
%                                 -(lambda_inf./2));
% 
%         ksi             =       0;
%         
%         % Tiploss function (Prandtl. Iterative since function of inflow ratio)
%         f_t             =       (B./2)*((1-mid_span)./(mid_span.*(lambda2./mid_span)));
%         f_r             =       (B./2)*((mid_span)./((1-mid_span).*(lambda2./mid_span)));
%         F               =       (2/pi).*(acos(exp(-f_t)))*(2/pi).*(acos(exp(-f_r)));
% 
%         checkF          =       any(any(abs((F - F_old)./F)>0.01)==1);
%         countF          =       countF + 1;
%         end    
% 
% lambda2 = bsxfun(@times,lambda2,ones(1,azm));
% lambda = (lambda + lambda2)./2;