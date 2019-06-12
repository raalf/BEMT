
% INFLOW RATIOS

    % Notes:
    % Non valid solution when: -2vi <= Vrp_a <= 0 (upward component of V in
    % descent). (pg. 99)

    % Option applies only to axial and hover


if V == 0 || AoA == 90
%% Axial climb and hover
% Rotor is symmetrically loaded, so only calculate for 1 azimuth to save
% computation time


%% OPTION 0: SMALL ANGLES AND NO SWIRL
    if oper.inflow_option == 0
        F = ones(stations-1,1); checkF = 1; countF = 1;
        while checkF == 1 && countF < 7                                 % Iteration scheme for tiploss factor, F (need to iterate because f affects induced velocity)
        F_old           =       F;
  
        lambda_inf      =       (Vrp_a)./(omega.*R);                    % Hover or Climb. If hover, vrp_a = 0, so reduced so lambda_h 
        lambda          =       sqrt(((sigma_mid.*(a_0)./(16.*F))-...
                                (lambda_inf./2)).^2+(sigma_mid.*...
                                (a_0).*beta_mid.*mid_span./(8.*F)))...
                                -((sigma_mid.*(a_0)./(16.*F))...
                                -(lambda_inf./2));

        ksi             =       zeros(length(lambda),azm);
        
        % Tiploss function (Prandtl. Iterative since function of inflow ratio)
        f_t             =       (B./2)*((1-mid_span)./(mid_span.*(lambda./mid_span)));
        f_r             =       (B./2)*((mid_span)./((1-mid_span).*(lambda./mid_span)));
        F               =       (2/pi).*(acos(exp(-f_t)))*(2/pi).*(acos(exp(-f_r)));

        checkF          =       any(any(abs((F - F_old)./F)>0.01)==1);
        countF          =       countF + 1;
        end    
        
         % Output lambda and ksi in format of # of azimuth columns
         lambda = bsxfun(@times,lambda,ones(1,azm));
        
%% OPTION 1: NO SMALL ANGLES AND SWIRL
    elseif oper.inflow_option == 1
            % Loop for station
            for i = 1:1:(stations-1)                                    % If in hover, rotor is symmetrically loaded so only calc single azimuth

                ETC.Vrp_a = Vrp_a;
                ETC.Vrp_p = Vrp_p;
                ETC.a_0 = a_0;
                ETC.R = R;
                ETC.r_mid = r_mid(i,:);
                ETC.omega = omega;
                ETC.B = B;
                ETC.Vaz_perp =  Vaz_perp(1);
                ETC.airfoil_mid = airfoil_mid(i,:);
                ETC.chord_mid = chord_mid(i,:);
                ETC.beta_mid = beta_mid(i,:);
                ETC.mid_span = mid_span(i,:);
                ETC.sigma_mid = sigma_mid(i,:);
                ETC.i = i;

                % ksi: swirl (tangential) inflow
                % lambda: axial inflow

                % x = [ksi,lambda]
                f = @(x)inflow_equations(x,ETC,flow);

                % Initial guess is the inflow ratio's with no added induced components
                x0 = [1,1];
                
                options = optimset('Display','off','Algorithm','trust-region-reflective');
                [x] = fsolve(f,x0,options);
                ksi(i,1) = x(1);
                lambda(i,1) = x(2);
                
                
                
            end
    end 
    
     % Output lambda and ksi in format of # of azimuth columns
     ksi = bsxfun(@times,ksi,ones(1,azm));
     lambda = bsxfun(@times,lambda,ones(1,azm));
     
elseif V ~= 0 % && AoA ~= 0 
%% Forward Flight  
    %% Tip speed ratio < 0.15
    mu = Vrp_p./(omega.*R);
    
    %if mu <= 0.15
        %viz = WIM_v6(blade,flow,oper,perf,R,AoA,azimuth)

    %% Tip speed ratio > 0.15


    % Numerical solution to the inflow equation from uniform momentum
    % theory with updating Ct including inflow adjustments.
    
    % Does not compute swirl component of inflow (ksi). Just lambda.
    
    count = 1; epsilon = 1; epsilonCt = 1;     % Initializations
    while epsilon > 0.0005 && count < 100 && epsilonCt > 0.0005
        
        % BET to get initial Ct guess from BET
            if count == 1
                vi = 0; ui = 0; 
                BETscript;       
                Ct_old = Ct;
                lambda_old = sqrt(Ct_old/2);
            end
        
        % Look into the induced power factor??
            
        % Momentum Theory for updated inflow and Ct
            mu = Vrp_p./(omega.*R);               % Also equal to mu_x, tipspeed ratio
            lambda_i = (Ct_old./(2*sqrt(mu.^2+lambda_old.^2)));
        
        % N-R iteration method
            f          =   lambda_old - mu.*tan(AoA_rad)-lambda_i;
            f_prime    =   1 + (Ct_old./2).*(mu.^2+lambda_old.^2).^(-3/2).*lambda_old;
            lambda_new =   lambda_old - (f/f_prime);

        % Fixed Point Method
        % lambda_new = mu.*tan(AoA_rad) + lambda_i;
         
        if isreal(lambda_new) == 0
            break
        end
        
        % Re-do BET for new Ct
        vi         =   (lambda_new.*omega.*R)-Vrp_a;
        BETscript;
        
        % Convergence criteria
        epsilon = abs((lambda_new - lambda_old)./lambda_new);
        epsilonCt = abs((Ct - Ct_old)./Ct);
        lambda_old = lambda_new;
        Ct_old = Ct;
        vi = (lambda_new.*omega.*R)-Vrp_a;       % Positive down
        convergence_lambda(count,:) = lambda_new;
        convergence_Ct(count,:) = Ct;
        count = count + 1;
    end
    
    % Linear inflow model
    lambda_i    =       (Ct_old./(2*sqrt(mu.^2+lambda_old.^2))); % Recalculate the lambda_i
    %lambda_i    =       sqrt(Ct_old/2);
    mu_z        =       Vrp_a./(omega.*R);
    
    if isreal(lambda_new) == 0
        lambda_i = 0;
    end
    
    chi         =       atan(mu./(mu_z+lambda_i));
    
    % Pitt and Peters inflow method
%     kx          =       (15.*pi./23).*tan(chi./2);
%     ky          =       0;
    
    % Drees method
    kx          =       (4/3).*(1 - cos(chi)-1.8.*(mu.^2))./sin(chi);
    ky          =       -2.*mu;
    
    % lambda      =       lambda_i.*(1+kx.*(mid_span.*cos(azimuth))+ky.*(mid_span.*sin(azimuth)));
    lambda      =       lambda_i.*(1+kx.*(bsxfun(@times,mid_span,cos(azimuth)))+ky.*(bsxfun(@times,mid_span,sin(azimuth))));
    
    
    ksi         =       0;
  
    
   %%% Hybrid inflow method using small angles for hover/axial flight

        F = ones(stations-1,1); checkF = 1; countF = 1;
        while checkF == 1 && countF < 7                                 % Iteration scheme for tiploss factor, F (need to iterate because f affects induced velocity)
        F_old           =       F;
  
        lambda_inf      =       (Vrp_a)./(omega.*R);                    % Hover or Climb. If hover, vrp_a = 0, so reduced so lambda_h 
        lambda2          =       sqrt(((sigma_mid.*(a_0)./(16.*F))-...
                                (lambda_inf./2)).^2+(sigma_mid.*...
                                (a_0).*beta_mid.*mid_span./(8.*F)))...
                                -((sigma_mid.*(a_0)./(16.*F))...
                                -(lambda_inf./2));

        ksi             =       0;
        
        % Tiploss function (Prandtl. Iterative since function of inflow ratio)
        f_t             =       (B./2)*((1-mid_span)./(mid_span.*(lambda2./mid_span)));
        f_r             =       (B./2)*((mid_span)./((1-mid_span).*(lambda2./mid_span)));
        F               =       (2/pi).*(acos(exp(-f_t)))*(2/pi).*(acos(exp(-f_r)));

        checkF          =       any(any(abs((F - F_old)./F)>0.01)==1);
        countF          =       countF + 1;
        end    

lambda2 = bsxfun(@times,lambda2,ones(1,azm));
lambda = (lambda + lambda2)./2;
    
% figure(1)
% hold on
% plot(mid_span, lambda, '-.*')
% plot(mid_span, lambda2(:,1), '-^')
% hold off
% title('Inflow models, (\alpha = 15°, V = 5 m/s, \rho = 1.17)')
% legend('Linear inflow model \psi = 0','Linear inflow model \psi = 90','Linear inflow model \psi = 180','Linear inflow model \psi = 270',...
%        'Hover model (Uniform loading)')
% xlabel('Normalized span')
% ylabel('Inflow ratio, \lambda')
% 
% figure(2)
% hold on 
% title('Hybrid inflow model, (\alpha = 15°, V = 5 m/s, \rho = 1.17)')
% plot(mid_span, lambda3, '-', 'LineWidth',1.5)
% hold off
% legend('Linear inflow model \psi = 0','Linear inflow model \psi = 90','Linear inflow model \psi = 180','Linear inflow model \psi = 270')
% xlabel('Normalized span')
% ylabel('Inflow ratio, \lambda')
% 
% pause 

else
%%  0 AoA side-slip special case
    % - For this case need to know hover inflow?
    %%% Vrp_a == 0 too, however include the side slip case in FF?
end


% Inflow velocity   

    vi              =        (lambda.*omega.*R)-Vrp_a;       % Positive down

    % notes for debugging: lambda.*omega.*R should be bigger than Vrp in
    % axial case because it is Vinf + vi
    
    if oper.inflow_option == 1 && (V == 0 || AoA == 90)
    ui              =        (bsxfun(@times,omega_r_mid,ones(1,azm)))-(ksi.*omega.*R);
    else
    ui = 0;
    end
    
    
    