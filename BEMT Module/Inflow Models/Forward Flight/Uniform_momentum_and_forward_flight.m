% Uniform_momentum_and_forward_flight  

% Numerical solution to the inflow equation from uniform momentum
% theory with updating Ct including inflow adjustments.
% Uses linear inflow model (Pitt & Peters or Drees) to compute the
% induced velocity at the rotor plane in working state or Coleman model
% for windmilling state.
% Does not compute swirl component of inflow (ksi).
    
% BET with 0 inflow to get initial Ct guess from BET
    vi = 0; ui = 0;
    BETscript;
    CT_old = CT;
    
% Check statement to see if rotor is operating in: regular working state, no work, or brake windmill state   
    if CT > 0
        state = 'working';
        lambda_old = sqrt(CT_old/2);
    elseif CT == 0
        state = 'zero_net_work';
        lambda_old = sqrt(CT_old/2);
    else
        state = 'windmill';
        BETscriptwindmill;
        lambda_old = 0;
    end
 
% Outer loop for convergence on thrust loading
count_CT = 1; epsilonCT = 1; 
while epsilonCT > 0.0005 && count_CT < 25
    
    % If no inflow component, solution reduces to BET. (exact case)
    if strcmpi(state,'zero_net_work') == 1
        lambda = Vrp_a./(omega.*R);
        break
    end
    
    % Inner loop for convergence on uniform inflow, given outer loop's current thrust loading
    count = 1; epsilon_lambda = 1;     % Initializations
    while epsilon_lambda > 0.0005 && count < 25
                
        % Momentum Theory for updated inflow and CT
            %lambda_i_old = (CT_old./(2*sqrt(mu.^2+lambda_i_old.^2)));
       
    % Working State
        if strcmpi(state,'working') == 1
        % N-R iteration method
            f          =   lambda_old - mu_x.*tan(AoA_rad)-(CT_old./(2*sqrt(mu_x.^2+lambda_old.^2)));
            f_prime    =   1 + (CT_old./2).*(mu_x.^2+lambda_old.^2).^(-3/2).*lambda_old;
            lambda_new =   lambda_old - (f/f_prime);

        % Fixed Point Method
            % lambda_new = mu.*tan(AoA_rad) + lambda_i;
    
    % Windmill State
        elseif strcmpi(state,'windmill') == 1
        % N-R iteration method
            f          =   lambda_old - mu_x.*tan(AoA_rad)+(CT_old./(2*sqrt(mu_x.^2+lambda_old.^2)));
            f_prime    =   1 - (CT_old./2).*(mu_x.^2+lambda_old.^2).^(-3/2).*lambda_old;
            lambda_new =   lambda_old - (f/f_prime);
        end 
         
        if isreal(lambda_new) == 0
            disp(['Error in FF iteration scheme at ' num2str(V) ' m/s'])
            disp('Inflow_new guess is NaN. (Inflow_forward_flight: line 50)')
            break
        end
                
        % Convergence criteria
        epsilon_lambda = abs((lambda_new - lambda_old)./lambda_new);
        
        % Convergence for lambda_i (centre of rotor) and CT
        plot_lambda_old(count,:) = lambda_old;
        lambda_old = lambda_new;
        count = count + 1;
    end

    % Induced inflow from uniform momentum theory (at center of rotor)
    % Uniform inflow - freestream inflow ratio
        lambda_0 = lambda_new-(Vrp_a./(omega.*R));
    
    % Apply linear inflow model here
        chi         =       atan(mu_x/(mu_z+lambda_0));

    % Pitt and Peters linear inflow model
        kx          =       (15*pi/23)*tan(chi/2);
        ky          =       0;
 
    % Drees linear inflow model
%         kx          =       (4/3).*(1 - cos(chi)-1.8.*(mu_x.^2))./sin(chi);
%         ky          =       -2.*mu_x;
    
    % If in windmill mode, use Coleman model (Leishman, pg.160 and pg.750)
        if strcmpi(state,'windmill') == 1
           % Coleman model
            kx          =       tan(chi/2) ;
            ky          =       0;
        end
        
        lambda_i      =      lambda_0.*(1+kx.*r_mid.*cos(azimuth)+ky.*r_mid.*sin(azimuth));
        %lambda      =       lambda_new.*(1+kx.*(bsxfun(@times,r_mid,cos(azimuth)))+ky.*(bsxfun(@times,r_mid,sin(azimuth))));
                 
        
    % Induced velocity dependant on working state
        if strcmpi(state,'working') == 1
            vi         =   (lambda_i.*omega.*R);
            %vi         =   (lambda.*omega.*R)-Vrp_a;
            ui = 0;
            
        elseif strcmpi(state,'windmill') == 1
            vi         =  (lambda_i.*omega.*R);
            %vi         =   Vrp_a - (lambda.*omega.*R);
            ui = 0;
        end
        
   % Re-run BET for new CT. Positive vi is down direction.  
        if strcmpi(state,'working') == 1
            BETscript;
        elseif strcmpi(state,'windmill') == 1
            BETscriptwindmill;
        end 
        
        if isreal(phi) == 0
            disp(['Error in FF iteration scheme at ' num2str(V) ' m/s'])
            disp('Induced angle, phi, is NaN. (Inflow_forward_flight: line 71)')
            break
        end
        
     % Update iterative convergence criteria and current CT
        epsilonCT = abs((CT - CT_old)./CT);
        CT_old = CT;
        count_CT = count_CT + 1;
end