% BET script
% Simple blade element theory for initial guess for rotor thrust, used in
% BEMT.




    V_R             =       sqrt((Vrp_a+vi).^2+(Vaz_perp+omega_mid-ui).^2); % Resultant velocity (Axial Velocity + (Rotational speed + R/A effects)) 
    Re_mid          =       flow.rho.*(V_R.*chord_mid)./(flow.mu);  % Reynolds number
    phi             =       atan2(Vrp_a+vi,Vaz_perp+omega_mid);  % Angle of V_R relative to rotor plane. (Beta - phi = AoA)       
    
    alpha_0         =       beta_mid-phi;
    
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
    
    dL              =       0.5.*flow.rho.*(V_R.^2).*chord_mid.*c_l;
    dD              =       0.5.*flow.rho.*(V_R.^2).*chord_mid.*c_d;
    dT              =       B.*(dL.*cos(phi)-(dD.*sin(phi)));
    dCT             =       dT./(flow.rho.*(pi.*R^2).*(omega.*R).^2);
    T               =       (sum(simps(mid_span,dT)+simps([mid_span(stations-1);R],[dT(stations-1,:);zeros(1,azm)])))./azm;
    CT              =       T./(flow.rho.*(pi.*R^2).*(omega.*R).^2);
    
    
    
    