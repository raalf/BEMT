% Script: Case 5
% Inflow_forward_flight  

    mu      =       Vrp_p./(omega.*R); % Also equal to mu_x, tipspeed ratio
    mu_z    =       Vrp_a./(omega.*R);
    
  % BET to get initial Ct guess from BET
    vi = 0; ui = 0; 
    BETscript;       
    dT_old = dT;
    lambda_i_old = sqrt(Ct/2);
    
    count = 1; epsilon = 1; epsilondT = 1;     % Initializations
    while epsilon > 0.0005 && count < 100 && any(any(epsilondT > 0.0005)~= 0)
        
        chi         =       atan(mu/(mu_z+lambda_i_old));
        
        % Pitt and Peters inflow method
        kx          =       (15*pi/23)*tan(chi/2);
        ky          =       0;

        lambda      =       lambda_i_old.*(1+kx.*(bsxfun(@times,mid_span,cos(azimuth)))+ky.*(bsxfun(@times,mid_span,sin(azimuth))));
        ksi         =       0;
        
        % Re-do BET for new Ct
        vi          =       (lambda.*omega.*R)-Vrp_a;
        BETscript;
        lambda_i    =       (Ct./(2*sqrt(mu.^2+lambda_i_old.^2)));
        
        % Convergence criteria
        epsilon = abs((lambda_i - lambda_i_old)./lambda_i);
        epsilondT = abs((dT - dT_old)./dT);

        % Convergence for lambda_i (centre of rotor) and Ct
        convergence_plot_lambda_i(count,:) = lambda_i;
        convergence_plot_Ct(count,:) = Ct;
        count = count + 1;
        lambda_i_old = lambda_i;
        dT_old = dT; 
    end
    
    