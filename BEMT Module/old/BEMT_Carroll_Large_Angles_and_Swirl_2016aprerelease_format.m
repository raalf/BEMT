% -------------------------------------------------------
% BLADE ELEMENT MOMENTUM THEORY ROTOR FUNCTION: VERSION 2.3 
% BY: Tim Carroll
% Most Recent Revision: March 11 2016
% -------------------------------------------------------

% Blade element momentum theory (BEMT) to calculate blade forces and rotor system
% Includes horizontal inflow and calculates propeller performance for
% various orientations, operating states and inflow directions
    % blade: blade geometry structure.
    % oper: operating conditions and wake parameters structure.
    % flow: flowfield and operating conditions.
    % perf: performance structure output.

function [perf] = BEMT_Carroll_Large_Angles_and_Swirl(blade,flow,oper) 
%% Inputs
    V               =       flow.V;                                 % Flight speed in [m/s].
    AoA             =       flow.inflow_angle;                      % Vehicle Angle of Attack, Inflow angle to rotor plane  [deg].
    stations        =       height(blade(1).geometry);              % Number of stations through height of table.
    B               =       blade.geometry{1,'Blades'};             % Number of blades.
    R               =       blade.geometry{stations,'Radius'};      % Rotor radius.
    c_R             =       blade.geometry{:,'c_R'};                % c/R normalized chord [c/R].
    beta_deg        =       blade.geometry{:,'Beta'};               % Pitch Angle distribution [Beta/R].
    airfoil         =       blade.geometry{:,'Airfoil'};            % Airfoils at each station.
    x_span          =       blade.geometry{:,'r_R'};                % Non-dimensional position (x = r/R), normalized 0 to 1.
    
    %Not needed, take out
    a_0             =       2*pi;                                   % Lift curve slope [1/rad]
    
% Quick modify rotor dimensions    
    beta_deg        =       beta_deg+oper.modify_pitch;
    R               =       R.*oper.scale_radius;
     
% Unit conversions
    beta_rad        =       degtorad(beta_deg);                     % Geometric angle of attack of 2D section. Section pitch (Beta) [rad]
    diameter        =       R*2;                                    % Diameter [m]
    chord           =       c_R*R';                                 % Dimensional chord lengths [m] 
    AoA_rad         =       degtorad(AoA);                          % Angle of attack [rad]
    r               =       x_span.*R;                              % Radial station distance [m]. y in Leishman.
    rps             =       oper.rpm/60;                            % Rotational speed [revs/sec].
    omega           =       rps.*2*pi;                              % Angular frequency, rad/s. 'Rotational frequency' (2pi*rev/s)

% Azimuth setup
    azm             =       oper.azimuth_num;                       % Number of azimuth positions. Increase number for refined calculation.  
    if azm==2
        azimuth     =       [pi/2,3*pi/2];                          % For 2 points, set them to R/A sections
    else
        azimuth     =       linspace(0, 2*pi-(2*pi/azm),azm);       
    end
    
% Midspan setup
    for n = 1:(stations-1)
    mid_span(n,:)   =       (x_span(n)+x_span(n+1))/2;              % "r" in Leishman. Non dimensional radial position.
    beta_mid(n,:)   =       (beta_rad(n)+beta_rad(n+1))/2;          % Mid location twist, [rad].
    chord_mid(n,:)  =       (chord(n)+chord(n+1))/2;                % Mid location chord, [m].
    r_mid(n,:)      =       (r(n)+r(n+1))/2;                        % Mid_span, [m].
    airfoil_mid(n,:)=       airfoil(n+1);
    %sigma_mid(n,:)  =       B.*chord_mid(n)./(pi.*R);               % Section solidity ratio.
    sigma_mid(n,:)  =       B.*chord_mid(n)./(2*pi.*r_mid(n));        % Section solidity ratio.
    delta_mid_span(n,:) =   r(n+1)-r(n);                            % Length of section, end point to end point.
    end 
    
% Initial calculations
    % Leishman convention, 0 deg is back of rotor
    % Blade advances first then retreats: spins CCW 
    J               =       V./(rps*diameter);                       % Advance ratio.
    omega_r_mid     =       r_mid.*omega;                            % Tangential velocity at each section (m/s).
    % Velocity @ rotor plane
    Vrp_a           =       V.*sin(AoA_rad);                         % Axial component towards rotor plane (this is "V" and is constant over the whole blade for all azms). + down
    Vrp_p           =       V.*cos(AoA_rad);                         % Parallel to rotor plane or "forward" component (used for R/A bstall).
    % Velocity @ blade's azimuth position
    Vaz_perp        =       Vrp_p.*sin(azimuth);                     % Inflow component perpendicular to blade (not rotor!) perpendicular to flow at current azimuth angle. Advances first.
    Vaz_tang        =       Vrp_p.*cos(azimuth);                     % Inflow component tangential (span wise along) to blade (not rotor!)
    
%% BEMT (Inflow, incremental force coefficients, sectional lift/drag, etc.)

% Inflow (BET/Momentum Theory Equivalence)
    if oper.toggle_vi == 1  % BEMT
        Inflow_ratio_script     
    else                    % BET
        vi = 0;
        ui = 0;
    end


% V_R (rotation, inflow, induced velocity)
    V_R             =       sqrt((Vrp_a+vi).^2+(bsxfun(@plus,Vaz_perp,bsxfun(@times,omega_r_mid,ones(1,azm)))-ui).^2); % Resultant velocity (Axial Velocity + (Rotational speed + R/A effects))           
    Re_mid          =       flow.rho.*(V_R).*chord_mid./(flow.mu);  % Reynolds number
    phi             =       atan2(Vrp_a+vi,Vaz_perp+omega_r_mid-ui);  % Angle of V_R relative to rotor plane. (Beta - phi = AoA)    
    
%  Determine section AoA and coefficients
    beta_zero       =       find_zero_lift_angle_airfoil(Re_mid,airfoil_mid);% Zero lift angle of attack [rad] (Beta for prop 2D section) 
    alpha_0         =       (beta_mid + beta_zero)-phi;   
    [c_l,sc]        =       lift_coeff_lookup(a_0,alpha_0,beta_zero,Re_mid,airfoil_mid);     
    [c_d]           =       drag_coeff_lookup(c_l,alpha_0,Re_mid,airfoil_mid,sc);
    
% Spanwise Force Coefficients
    dL              =       0.5.*flow.rho.*(V_R.^2).*chord_mid.*c_l;
    dD              =       0.5.*flow.rho.*(V_R.^2).*chord_mid.*c_d;
    dT              =       B.*(dL.*cos(phi)-(dD.*sin(phi)));
    dQ              =       B.*(dL.*sin(phi)+(dD.*cos(phi))).*r_mid;
    dP              =       B.*(dL.*sin(phi)+(dD.*cos(phi))).*omega_r_mid;
    dN              =       B.*(dL.*sin(phi)+(dD.*cos(phi)));      % Normal force, "P-factor"
    
%% Summation of forces and coefficients(with hub and tip sections added)
    perf.T          =       (sum(simps(r_mid,dT)+simps([0;r_mid(1)],[zeros(1,azm);dT(1,:)])+simps([r_mid(stations-1);R],[dT(stations-1,:);zeros(1,azm)])))./azm;
    perf.Q          =       (sum(simps(r_mid,dQ)+simps([0;r_mid(1)],[zeros(1,azm);dQ(1,:)])+simps([r_mid(stations-1);R],[dQ(stations-1,:);zeros(1,azm)])))./azm;
    perf.P          =       (sum(simps(r_mid,dP)+simps([0;r_mid(1)],[zeros(1,azm);dP(1,:)])+simps([r_mid(stations-1);R],[dP(stations-1,:);zeros(1,azm)])))./azm;
    perf.N          =       (sum(simps(r_mid,dN)+simps([0;r_mid(1)],[zeros(1,azm);dN(1,:)])+simps([r_mid(stations-1);R],[dN(stations-1,:);zeros(1,azm)])))./azm;
    perf.Ct         =       perf.T./(flow.rho.*rps^2.*diameter^4);
    perf.Cp         =       perf.P./(flow.rho.*rps^3.*diameter^5);          
    perf.Cn         =       perf.N./(flow.rho.*rps^2.*diameter^4);
    
%% Add temporary vars to perf structure    
perf.dT = dT; perf.dQ = dQ; perf.dP = dP; perf.mid_span = mid_span;perf.r_mid = r_mid; perf.omega = omega; perf.R = R;
perf.azimuth = azimuth; perf.Re_mid = Re_mid; perf.r_mid = r_mid; perf.c_l = c_l; perf.c_d = c_d; perf.J=J; perf.alpha_0 = alpha_0;
perf.V_R = V_R;
end