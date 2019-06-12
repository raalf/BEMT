% Inflow_large_angles_and_swirl

% ksi: swirl (tangential) inflow
% lambda: axial inflow

% Store variables in INPUT structure for easy function inputting.
    INPUT.Vrp_a = Vrp_a; % Dont need Vrp_p, since this derivation is for axial and hover only so Vrp_p = 0;
    INPUT.R = R;
    INPUT.omega = omega;
    INPUT.B = B;

  % Loop for station
    for i = 1:1:length(r_mid)
        
        % Store variables in INPUT structure for easy function inputting.
            INPUT.r_mid = r_mid(i,:);
            INPUT.airfoil_mid = airfoil_mid(i,:);
            INPUT.chord_mid = chord_mid(i,:);
            INPUT.beta_mid = beta_mid(i,:);
            INPUT.mid_span = mid_span(i,:);
            INPUT.sigma_mid = sigma_mid(i,:);
            INPUT.i = i;

            try
                % Initial guess is [1,1]. DO NOT set initial guess to [0,0],
                % will not find minimum.
                    x0 = [1,1];
                    
                % Inflow equations anonymous function
                    inflow = @(x)inflow_equations(x,INPUT,flow); % x = [ksi,lambda]
                    fsolveoptions = optimset('Display','off','Algorithm','trust-region-reflective');
                    [x] = fsolve(inflow,x0,fsolveoptions);
            catch
                    x = [(INPUT.omega*INPUT.mid_span)/(omega*R) , INPUT.Vrp_a/(omega*R)];
            end

                x_store(i,:) = x;
    end

    ksi = x_store(:,1); lambda = x_store(:,2);
    
     % Output lambda and ksi in format of # of azimuth columns
     ksi = bsxfun(@times,ksi,ones(1,azm));
     lambda = bsxfun(@times,lambda,ones(1,azm));