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
    
checklambda = 1;% Initializations for iterative conditions
F = ones(stations-1,1); checkF = 1; countF = 1;
lambda = (Vrp_a)./(omega.*R); % Initialize to lambda_inf for first pass of Re_mid calculation. Recall: lambda = (Vrp_a + vi)./(omega.*R);
% vi=0;ui=0;
% BETscript;
% lambda = sqrt(Ct/2);

lambda_inf  =  (Vrp_a)./(omega.*R); % Recall: lambda = (Vrp_a + vi)./(omega.*R);
negative_inflow_stations = ones(stations-1,1);
% lambda_c        =       Vrp_a/(sqrt(Ct/2); % Climb velocity ratio
% climb velocity ratio, lambda_c, Vc/vh = Vrp_a/(sqrt(Ct/2)
        

% New initial guess; vi assuming airfoil is operating close to its best L/D

%lambda = (mid_span.*tan(beta_mid-deg2rad(1)))./R;


while (checklambda == 1 || checkF == 1) && (countF < 20)
        
        F_old           =       F;
        lambda_old      =       lambda;
        
    % Estimate zero lift angle based on section Re
        V_R             =       sqrt((lambda.*omega.*R).^2+(omega_mid).^2);
        Re_mid          =       flow.rho.*(V_R.*chord_mid)./(flow.mu);  % Reynolds number
        
        % Induced velocity for positive inflow elements
        phi             =       atan2((lambda.*(omega.*R)),Vaz_perp+omega_mid);
        
        if any(abs(phi)>abs(beta_mid)) == 1 && checkF<10 % Convergence smoothing for small angle case.
           [row,column] = find(abs(phi)>abs(beta_mid));
           phi(row) = beta_mid(row);
        end
        
        alpha           =       beta_mid - phi;
        [c_l,sc]        =       lift_coeff_lookup(alpha,Re_mid,airfoil_mid);
       
        lambda          =       (lambda_inf./2)+sqrt((lambda_inf.^2./4)+(sigma_mid.*c_l.*r_mid.^2)./(4.*F));
        
    % If lambda is not real
        if (any(lambda < 0) == 1) || (any(isreal(lambda)) == 0)
           [row,column] = find((imag(lambda)~=0)|(lambda<0));  
           lambda(row)  =  -((lambda_inf./2)+sqrt((lambda_inf.^2./4)-(sigma_mid(row).*c_l(row).*r_mid(row).^2)./(4.*F(row))));
        end
        
    % Tip/root loss function (Prandtl. Iterative since function of inflow ratio)
%         f_t              =       (B/2).*((1-r_mid)./(r_mid.*(abs(lambda)./r_mid)));
%         f_r              =       (B/2).*((r_mid)./((1-r_mid).*(abs(lambda)./r_mid)));
%         F                =       (2/pi).*(acos(exp(-f_t)))*(2/pi).*(acos(exp(-f_r)));

        checklambda      =       any(any(abs((lambda - lambda_old)./lambda)>0.005)==1);
        checkF           =       any(any(abs((F - F_old)./F)>0.005)==1);
        countF           =       countF + 1;
        
%        figure(8)
%        hold on
%        plot(r_mid,lambda)
%        title('\lambda')
%        
%        figure(2)
%        hold on
%        plot(r_mid,c_l)
%        title('c_{l}')
%        
%        figure(3)
%        hold on
%        plot(r_mid,rad2deg(alpha))
%        title('\alpha')

 end
       
% For cases that dont converge on the inner blade portion, set lambda to intermediate "guess"
if countF >= 20   
    lambda = (lambda +lambda_old)/2;
end

% ^ be careful w/ opt. this may give false inflow if one of the cases isn't real.
% Try to take higher value?

        %If term inside sqrt < 0, then negative inflow is potentially
        %present. Model cannot handle complex distributions of +/- inflow
        %on the same blade. Therefore set c_l = 0, store the location this
        %is happening and equation reduces down to inflow_inf, so set
        %lambda = lambda_inf
        
%         if any(((lambda_inf.^2./4)+(sigma_mid.*c_l.*r_mid.^2)./(4.*F))<0)==1
%             [row,~] = find((((lambda_inf.^2./4)+(sigma_mid.*c_l.*r_mid.^2)./(4.*F)))<0); 
%             negative_inflow_stations(row) = 0;
%         end               
        