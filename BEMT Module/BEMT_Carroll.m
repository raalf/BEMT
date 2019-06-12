% -------------------------------------------------------
% BLADE ELEMENT MOMENTUM THEORY ROTOR FUNCTION: VERSION 3 
% BY: Tim Carroll
% Most Recent Modifications: April 10th 2017
% -------------------------------------------------------

% Blade element momentum theory (BEMT) to calculate blade forces and rotor system
% Includes horizontal inflow and calculates rotor/propeller performance for
% various orientations, operating states and inflow directions
    % blade: blade geometry structure.
    % oper: operating conditions and wake parameters structure.
    % flow: flowfield and operating conditions.
    % perf: performance structure output.
    % save_workspace: on/off char toggle if workspace is saved whenever function is called
    
    % Sign conventions:
    % Local rotor plane:
    % - Blade spins CCW
    % - Blade advances first then retreats
    % - Rotor spins in xy plane
    % - From top view of rotor, y is vertical, x is horizontal. Psi = 0, is
    %   back of rotor plane (positive y axis)

   
function [perf] = BEMT_Carroll(blade,flow,oper,rotor,wake,options) 
% Extract variables from blade structure
    V               =       flow.V;                                  % Flight speed in [m/s].
    AoA             =       flow.inflow_angle;                       % Vehicle Angle of Attack, Inflow angle to rotor plane  [deg].
    stations        =       height(blade.geometry);                  % Number of stations through height of table.
    B               =       blade.geometry.Blades(1);                % Number of blades.
    R               =       blade.geometry.Radius(stations);         % Rotor radius.
    c_R             =       blade.geometry.c_R;                      % c/R normalized chord [c/R].
    beta_deg        =       blade.geometry.Beta;                     % Pitch angle of attack of 2D section. Sectional pitch (Beta) [deg].
    airfoil         =       blade.geometry.Airfoil;                  % Airfoils at each station.
    r               =       blade.geometry.r_R;                      % Non-dimensional position, normalized 0 to 1.
        
% Quick modify rotor dimensions
    beta_deg        =       beta_deg+blade.modify_pitch;
    R               =       R.*blade.scale_radius;
    R(R==0)         =       blade.geometry.Radius(stations);         % Fail-safe check if blade.scale_radius = 0 is accidentally entered
    
% Unit conversions
    beta_rad        =       degtorad(beta_deg);                      % Pitch [rad]
    diameter        =       R*2;                                     % Diameter [m]
    chord           =       c_R.*R;                                 % Dimensional chord lengths [m] 
    AoA_rad         =       degtorad(AoA);                           % Angle of attack [rad]
    y_span          =       r.*R;                                    % Radial station distance [m].
    rps             =       oper.rpm./60;                            % Rotational speed [revs/sec].
    omega           =       rps.*2*pi;                               % Angular frequency, rad/s. 'Rotational frequency' (2pi*rev/s)   
    
% Azimuth position setup
    azm             =       oper.azimuth_num;                        % Number of azimuth positions. Increase number for refined calculation.  
    
    if azm==2
        azimuth     =       [pi/2,3*pi/2];                           % For 2 points, set them to retreating/advancing positions
    else
        azimuth     =       linspace(0, 2*pi-(2*pi/azm),azm);       
    end
    
% If rotor is in hover/axial flight, reduce to azm = 1 to speed up computation time. (Only when not using WIM)
%     if (V == 0 || AoA == 90) && strcmpi(oper.toggle_WIM,'off') == 1 && oper.inflow_type < 5
%         azm = 1; azimuth = 0;
%     end
    
% Mid-station setup
    % Pre-allocate for speed
    mid_span = zeros(stations-1,1) ; beta_mid = zeros(stations-1,1); 
    chord_mid = zeros(stations-1,1); r_mid = zeros(stations-1,1); 
    airfoil_mid = cell(stations-1,1); sigma_mid = zeros(stations-1,1);

    for n = 1:(stations-1)
        r_mid(n,:)      =       (r(n)+r(n+1))/2;                     % Non-dimensional radial position: "r" in Leishman
        beta_mid(n,:)   =       (beta_rad(n)+beta_rad(n+1))/2;       % Mid location twist, [rad].
        chord_mid(n,:)  =       (chord(n)+chord(n+1))/2;             % Mid location chord, [m].
        mid_span(n,:)   =       (y_span(n)+y_span(n+1))/2;           % Mid_span, [m].
        airfoil_mid(n,:)=       airfoil(n+1);                        % Airfoil, take airfoil closer to tip
        sigma_mid(n,:)  =       B.*chord_mid(n)./(2*pi.*mid_span(n));% Sectional solidity ratio.
    end 

%% Initial calculations
    % 0 [deg] is back of rotor plane. Spins CCW: blade advances first
    omega_mid       =       mid_span.*omega;                         % Tangential velocity at each section (m/s).
% Velocity component normal to rotor plane
    Vrp_a           =       V.*sin(AoA_rad);                         % Axial component towards rotor plane (this is "V" and is constant over the whole blade for all azms). + down
    Vrp_p           =       V.*cos(AoA_rad);                         % Parallel to rotor plane or "forward" component (used for R/A bstall).
% Velocity component parallel to disk, wrt blade frame
    Vaz_perp        =       Vrp_p.*sin(azimuth);                     % Inflow component perpendicular to blade (not rotor!) perpendicular to flow at current azimuth angle. Advances first.
    Vaz_tang        =       Vrp_p.*cos(azimuth);                     % Inflow component tangential (span wise along) to blade (not rotor!)
% Advance ratios
    mu_x            =       Vrp_p./(omega.*R);                       % Rotor advance ratio parallel to disk (tangential).
    mu_z            =       Vrp_a./(omega.*R);                       % Rotor advance ratio normal to disk (axial).
    mu_freestream   =       V./(omega.*R);                           % Freestream advance ratio
    J               =       V./(rps*diameter);                       % Propeller advance ratio.
    
% Global solidity. Excluding hub area
    global_sigma    =       (B*trapz(y_span,chord))/((pi*R^2)-(pi*y_span(1)^2)); % Sectional solidity ratio.

%% Rotor inflow
% Inflow Models
    if (V ~= 0 && AoA ~= 90) && (oper.inflow_type ~= 1) && (oper.inflow_type ~= 5)% If rotor in FF
        oper.inflow_type = 1; % Switch to FF model if case 1 or 2 is accidentally selected.
        warning('Inflow reverting to uniform momentum with FF inflow model, Vrp_p =/= 0. Rotor had edgewise velocity component');
    end
    
    if strcmpi(oper.toggle_visc,'off') && oper.inflow_type>2 && (flow.V == 90 || flow.inflow_angle)
        warning('Viscous effects toggle is off, reverting to uniform inflow model 2');
        oper.inflow_type = 2;
    end
        
    % Returns: vi and ui in LOCAL ROTOR reference frame.
    try
        switch   lower(oper.toggle_vi)
        % BEMT
            case 'on'
                switch oper.inflow_type
                    case 1
                        Uniform_momentum_and_forward_flight
                        lambda = (Vrp_a+vi)./(omega.*R);
                        ui = 0;
                    case 2
                        Inflow_small_angles_no_swirl
                        ui =  0;
                    case 3
                        Inflow_small_angles_no_swirl_coeff_lookup
                        ui =  0;
                    case 4
                        Inflow_large_angles_and_swirl
                        ui = (bsxfun(@times,omega_mid,ones(1,azm)))-(ksi.*omega.*R);
                    case 5
                        Inflow_WIM_single_rotor % LOCAL COORDINATES TO BLADE POSITION! % Get z (or inflow) component in BLADE coordinate system
                end
        % BET (No induced velocity)
            case 'off'
                lambda  = Vrp_a./(omega.*R);
                ui =  0;
        end
    catch % Catch when the induced velocity is non-real
        warning('Error in inflow calculaiton: Reverting now to BET forumlation','on','backtrace','on','verbose');
        lambda  = Vrp_a./(omega.*R);
    end
    
%% Blade velocities, sectional coefficients and adjacent wake effects

% Update to include wake effects on adjacent velocities at rotor plane.
% Included in freestream velocity components
%     if strcmp(oper.toggle_WIM,'on') == 1
%             Vrp_a    =       Vrp_a.*ones(stations-1,1) + wake.vi_WIM(rotor.WIM_rotor_counter) ;
%             Vrp_p    =       Vrp_p.*ones(stations-1,1) + wake.vi_WIM(rotor.WIM_rotor_counter) ;
%     end

% Inflow angle, (Pitch - inflow angle = Effective AoA)
        phi         =       atan2((lambda.*(omega.*R)),Vaz_perp+omega_mid-ui);
% Resultant velocity at each element
        V_R         =       sqrt((lambda.*(omega.*R)).^2+(Vaz_perp+omega_mid-ui).^2);
% Reynolds number at each element
        Re_mid      =       flow.rho.*(V_R.*chord_mid)./(flow.mu);
% Mach number at each element, assuming perfect gas @ sea level
        Mach_mid    =       abs(V_R)./340;

%  Determine section AoA and coefficients
    % beta_zero       =       find_zero_lift_angle_airfoil(Re_mid,airfoil_mid);% Zero lift angle of attack [rad] (Beta for prop 2D section) 
    alpha_0         =       beta_mid - phi;   

% Coefficient gathering methods
    % Database look-up option, viscous effects on
    if strcmpi(oper.toggle_visc,'on') && strcmpi(options.toggle_precompute,'on')
        [c_l,c_d,c_m] = lookup_aero_coeff_database(options,options.airfoil_coeff_database,alpha_0,Re_mid,airfoil_mid);
    % Single look-up, viscous effects on
    elseif strcmpi(oper.toggle_visc,'on') && strcmpi(options.toggle_precompute,'off')
        [c_l,sc]    =       lift_coeff_lookup(alpha_0,Re_mid,airfoil_mid);     
        [c_d]       =       drag_coeff_lookup(c_l,alpha_0,Re_mid,airfoil_mid,sc);
        [c_m]       =       moment_coeff_lookup(alpha_0,Re_mid,airfoil_mid,sc);
    % Special option for optimization   
    elseif strcmpi(oper.toggle_visc,'on') && strcmpi(options.toggle_precompute,'opt')
        load('opt_coeffs')
        [c_l,c_d,c_m] = airfoil_lookup_for_optimization(alpha_0,Re_mid,airfoil_mid,opt_coeffs);
    % Viscous effects off
    else
        alpha_0     =       alpha_0 - oper.alpha_zero;
        c_l         =       alpha_0.*oper.a_0;
        c_d         =       0;
        c_m         =       0;
    end
    
% Sweep corrected lift and drag. Johnson corrected
        % Lambda_yaw  =       atan((mu_x.*cos(azimuth))./(r_mid+mu_x*sin(azimuth)));
        % [c_l,sc]    =       lift_coeff_lookup(alpha_0.*cos(Lambda_yaw).^2,Re_mid,airfoil_mid);
        % c_l         =       (c_l)./cos(Lambda_yaw).^2;
        % c_d         =       (drag_coeff_lookup(c_l,alpha_0.*cos(Lambda_yaw),Re_mid,airfoil_mid,sc))./cos(Lambda_yaw); 

%% Blade forces             
            
    % Spanwise elemental lift, drag, pitching moment as a function of azimuth position
        dL          =       0.5.*flow.rho.*(V_R.^2).*chord_mid.*c_l;
        dD          =       0.5.*flow.rho.*(V_R.^2).*chord_mid.*c_d;
        dM          =       0.5.*flow.rho.*(V_R.^2).*chord_mid.^2.*c_m;

    % Spanwise elemental blade forces and moments  
        % Thrust (integrated from R(1) to tip)
        % Torque (integrated from rotational axis, R = 0, to tip)
        % Longitudinal force (normal force, "P-factor")
        % Lateral force (side force)
        % Rolling moment
        % Pitching moment
      
        dT          =       B.*(dL.*cos(phi)-(dD.*sin(phi)));
        dQ          =       B.*(dL.*sin(phi)+(dD.*cos(phi))).*mid_span;
        dP          =       B.*(dL.*sin(phi)+(dD.*cos(phi))).*omega_mid;

        dNx         =       B.*-(dL.*sin(phi)+(dD.*cos(phi))).*sin(azimuth);
        dNy         =       B.*(dL.*sin(phi)+(dD.*cos(phi))).*cos(azimuth);
        dMx         =       B.*-(dL.*cos(phi)-(dD.*sin(phi))).*mid_span.*sin(azimuth) + dM.*cos(azimuth);
        dMy         =       B.*(dL.*cos(phi)-(dD.*sin(phi))).*mid_span.*cos(azimuth) - dM.*sin(azimuth);

    % Induced drag and decomposed power sources ( Induced, Parasite)
        dD_i        =       dL.*sin(phi);                  % Induced drag
        dP_i        =       B.*(dL.*sin(phi)).*omega_mid;  % Induced power
        dP_p        =       B.*(dD.*cos(phi)).*omega_mid;  % Profile power

    % Small angle approximations for the elemental blade forces
        dT_small    =       B.*(dL);
        dP_small    =       B.*(dL.*sin(phi)+(dD)).*omega_mid;
        dP_p_small  =       B.*(dD).*omega_mid;

        % Post stall, T is positive? must fix
        %if exist('state') == 1 && strcmpi(state,'windmill') == 1 % Recalculate if in windmill mode
        %dT              =       B.*(dL.*cos(phi)+(dD.*sin(phi)));
        %dQ              =       B.*(dL.*sin(phi)-(dD.*cos(phi))).*mid_span;
        %dP              =       B.*(dL.*sin(phi)-(dD.*cos(phi))).*omega_mid;
        %end

    % Blade forces averaged over full rotation
        % Thrust (integrated from R(1) to tip)
        % Torque (integrated from rotational axis, R = 0, to tip)
        % Longitudinal force (normal force)
        % Lateral force (side force)
        % Rolling moment
        % Pitching moment
        
        T           =       (sum(simps(mid_span,dT)+simps([mid_span(stations-1);R],[dT(stations-1,:);zeros(1,azm)])))./azm;
        Q           =       (sum(simps(mid_span,dQ)+simps([0;mid_span(1)],[zeros(1,azm);dQ(1,:)])+simps([mid_span(stations-1);R],[dQ(stations-1,:);zeros(1,azm)])))./azm;
        Nx          =       (sum(simps(mid_span,dNx)+simps([0;mid_span(1)],[zeros(1,azm);dNx(1,:)])+simps([mid_span(stations-1);R],[dNx(stations-1,:);zeros(1,azm)])))./azm;
        Ny          =       (sum(simps(mid_span,dNy)+simps([0;mid_span(1)],[zeros(1,azm);dNy(1,:)])+simps([mid_span(stations-1);R],[dNy(stations-1,:);zeros(1,azm)])))./azm;
        Mx          =       (sum(simps(mid_span,dMx)+simps([0;mid_span(1)],[zeros(1,azm);dMx(1,:)])+simps([mid_span(stations-1);R],[dMx(stations-1,:);zeros(1,azm)])))./azm;
        My          =       (sum(simps(mid_span,dMy)+simps([0;mid_span(1)],[zeros(1,azm);dMy(1,:)])+simps([mid_span(stations-1);R],[dMy(stations-1,:);zeros(1,azm)])))./azm;

        % Vector sum of in-plane forces and azimuth direction (in rads) 
        Nsum        =       sqrt(Nx^2+Ny^2);
        N_angle     =       atan2(Ny,Nx);                   % [Rads]
        
        % Total power and decomposed power sources (Total, Induced, Parasite)
        P           =       (sum(simps(mid_span,dP)+simps([0;mid_span(1)],[zeros(1,azm);dP(1,:)])+simps([mid_span(stations-1);R],[dP(stations-1,:);zeros(1,azm)])))./azm;
        P_i         =       (sum(simps(mid_span,dP_i)+simps([0;mid_span(1)],[zeros(1,azm);dP_i(1,:)])+simps([mid_span(stations-1);R],[dP_i(stations-1,:);zeros(1,azm)])))./azm;
        P_p         =       (sum(simps(mid_span,dP_p)+simps([0;mid_span(1)],[zeros(1,azm);dP_p(1,:)])+simps([mid_span(stations-1);R],[dP_p(stations-1,:);zeros(1,azm)])))./azm;
    
        % Induced drag
        D_i         =       (sum(simps(mid_span,dD_i)+simps([0;mid_span(1)],[zeros(1,azm);dD_i(1,:)])+simps([mid_span(stations-1);R],[dD_i(stations-1,:);zeros(1,azm)])))./azm;

        % "Small angle" approximations
        T_small     =       (sum(simps(mid_span,dT_small)+simps([0;mid_span(1)],[zeros(1,azm);dT_small(1,:)])+simps([mid_span(stations-1);R],[dT_small(stations-1,:);zeros(1,azm)])))./azm;       
        P_small     =       (sum(simps(mid_span,dP_small)+simps([0;mid_span(1)],[zeros(1,azm);dP_small(1,:)])+simps([mid_span(stations-1);R],[dP_small(stations-1,:);zeros(1,azm)])))./azm;       
        P_p_small   =       (sum(simps(mid_span,dP_p_small)+simps([0;mid_span(1)],[zeros(1,azm);dP_p_small(1,:)])+simps([mid_span(stations-1);R],[dP_p_small(stations-1,:);zeros(1,azm)])))./azm;

%% Rotor/propeller coefficients    
    % Rotor force coefficients averaged over full rotation, rotor convention
        CT          =       T./(flow.rho.*(pi*R^2)*(omega*R)^2);     % Thrust, rotor
        CQ          =       Q./(flow.rho.*(pi*R^2)*(omega*R)^2*R);   % Torque, rotor
        CP          =       P./(flow.rho.*(pi*R^2)*(omega*R)^3);     % Power, rotor
        CNx         =       Nx./(flow.rho.*(pi*R^2)*(omega*R)^2);    % Normal longitudinal force x-dir, rotor
        CNy         =       Ny./(flow.rho.*(pi*R^2)*(omega*R)^2);    % Side lateral force y-dir, rotor
        CMx         =       Mx./(flow.rho.*(pi*R^2)*(omega*R)^2*R);  % Rolling moment, rotor
        CMy         =       My./(flow.rho.*(pi*R^2)*(omega*R)^2*R);  % Pitching moment, rotor
    
    % Blade loading coefficients averaged over full rotation      
        CT_sigma    =      (CT)./global_sigma;                       % Solidity weighted thrust (blade loading), rotor
        CQ_sigma    =      (CQ)./global_sigma;                       % Solidity weighted Torque, rotor
        CP_sigma    =      (CP)./global_sigma;                       % Solidity weighted Power, rotor
        CNx_sigma   =      (CNx)./global_sigma;                      % Solidity weighted Normal x force, rotor
        CNy_sigma   =      (CNy)./global_sigma;                      % Solidity weighted Normal y force, rotor 
        CMx_sigma   =      (CMx)./global_sigma;                      % Solidity weighted Normal x moment, rotor
        CMy_sigma   =      (CMy)./global_sigma;                      % Solidity weighted Side y moment, rotor
        
    % Rotor Figure of Merit    
        FM          =      (CT^(3/2)/(sqrt(2)*CP));                  % Figure of Merit
    
    % Propeller force coefficients averaged over full rotation, propeller convention    
        Ctp         =      T./(flow.rho.*rps^2.*diameter^4);         % Thrust, propeller
        Cqp         =      Q./(flow.rho.*rps^2.*diameter^5);         % Torque, propeller
        Cpp         =      P./(flow.rho.*rps^3.*diameter^5);         % Power, propeller
        Cnxp        =      Nx./(flow.rho.*rps^2.*diameter^4);        % Normal x force, propeller
        Cnyp        =      Ny./(flow.rho.*rps^2.*diameter^4);        % Normal y force, propeller
        Cnsump      =      Nsum./(flow.rho.*rps^2.*diameter^4);      % Normal force sum, propeller

%% Save all workspace variables to perf structure depending on options.save_workspace in BEMT_analysis

save('../Saved workspaces/temp_perf.mat'); % Save workspace to .mat file
perf = load('../Saved workspaces/temp_perf.mat'); % Load .mat file to perf structure
delete('../Saved workspaces/temp_perf.mat'); % Clean up folder by deleting .mat file

% New method - Save workspace in field/structure in folder specified by user
if strcmpi(options.save_workspace,'on') 
    % Verify that folder storing workspace structure file also exists
    if exist('Saved workspaces','dir') ~= 7
        % Create folder
        mkdir '../Saved workspaces';
        % Add to path
        addpath('../Saved workspaces');
    end

    % Verify that saved workspace structure file also exists.
    % If so, load it. If not, create it.
    if exist('Saved_workspace.mat','file') ~= 2
        Saved_workspace = [];
        save('../Saved workspaces/Saved_workspace', 'Saved_workspace')
    else
        load('../Saved workspaces/Saved_workspace', 'Saved_workspace')
    end

    % Format & append data to save file with tag and data
    rows = length(Saved_workspace);
    Saved_workspace(rows+1).perf          = perf;
    Saved_workspace(rows+1).identifier    = options.saved_workspace_identifier;
    Saved_workspace(rows+1).date          = datestr(datetime('now'));
    save('../Saved workspaces/Saved_workspace', 'Saved_workspace')
end            
   
% When using in optimization routine, comment out save/load routine and
% instead directly output vars to save resources
%     perf.T = T;
%     perf.P = P;
%     perf.T_small = T_small;
%     perf.P_small = P_small;
%     perf.Q = Q;
%     perf.D_i = D_i;
%     perf.CT = CT;
%     perf.CP = CP;
%     perf.lambda = lambda;
%     perf.r_mid = r_mid;
%     perf.dT = dT;
%     perf.dP = dP;
    
end