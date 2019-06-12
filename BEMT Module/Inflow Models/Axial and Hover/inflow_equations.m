% INFLOW EQUATIONS
% System of non-linear equations for axial and tangentual inflow ratios

% INPUTS:
% 1. x = [ksi,lambda], swirl and inflow.
% 2. INPUT structure consisting of the following:
%     INPUT.Vrp_a
%     INPUT.R
%     INPUT.omega
%     INPUT.B
%     INPUT.r_mid
%     INPUT.airfoil_mid
%     INPUT.chord_mid
%     INPUT.beta_mid
%     INPUT.mid_span
%     INPUT.sigma_mid
%     (Optional) INPUT.i
% 3. flow structure containing:
%     flow.rho
%     flow.mu
%     flow.V
%     flow.inflow_angle

% OUTPUTS:
% F, system of equations corresponding to thrust and power equivalence
% including inflow and swirl.

% COUPLED WITH:
% Inflow_large_angles_and_swirl.m

% NOTES:
% Rows are stations, columns are azimuths

% ksi: swirl (radially tangential to 2D blade section)
% ksi = (Vrp_p - ui)./(omega_mid.*R)

% lambda: axial inflow
% lambda = (Vrp_a + vi)./(omega.*R)

function F = inflow_equations(x,INPUT,flow)

% x = [ksi,lambda]
    ksi = x(:,1);
    lambda = x(:,2);
    
    if isreal(lambda) == 0 || isreal(ksi) == 0
        % warning('Large angle/swirl inflow: lambda or ksi on section is not real', 'on','backtrace')
        return
    else
    
    phi      =       atan2(lambda,ksi);

% Compute tiploss factors, Kt and Kp, for thrust and power, respectively.
    F        =       (2/pi).*(acos(exp((INPUT.B.*(INPUT.r_mid-1))./(2.*INPUT.r_mid.*sin(phi)))));
    Kt       =       (1-(1-F).*cos(phi));
    Kp       =       (1-(1-F).*sin(phi));
    

    vi       =       (lambda.*INPUT.omega.*INPUT.R) - INPUT.Vrp_a;
    ui       =       (ksi   .*INPUT.omega.*INPUT.R) - (INPUT.omega.*INPUT.mid_span); % Vrp_p = 0 always since rotor in axial or hover
    
    V_R      =       sqrt((INPUT.Vrp_a+vi).^2+((INPUT.omega.*INPUT.mid_span)-ui).^2);
    Re_mid   =       flow.rho.*(V_R).*INPUT.chord_mid./(flow.mu); 

    alpha    =       (INPUT.beta_mid)-phi;

    [c_l,sc] =       lift_coeff_lookup(alpha,Re_mid,INPUT.airfoil_mid);
    [c_d]    =       drag_coeff_lookup(c_l,alpha,Re_mid,INPUT.airfoil_mid,sc); 


    F(1)     =       INPUT.sigma_mid.*(sqrt((ksi.^2)+lambda.^2)).*(c_l.*ksi... 
                     - c_d.*lambda) - (4.*Kt.*abs(lambda).*...
                     (lambda-(INPUT.Vrp_a./(INPUT.omega.*INPUT.R))));

    F(2)     =       INPUT.sigma_mid.*(sqrt((ksi.^2)+lambda.^2)).*(c_l.*lambda... 
                     + c_d.*ksi) - (4.*Kp.*abs(lambda).*...
                     (INPUT.r_mid-ksi));
                 
    end             
end