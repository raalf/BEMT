% PERFORMANCE SWEEP SCRIPT
% Rotor performance sweep wrt velocity.
% Limits correspond to sweep_range variable in BEMT_analysis main
% options.sweep_range(1) <- Start point
% options.sweep_range(2) <- Interval
% options.sweep_range(3) <- End point

% Results are output in structured field. In order to plot, values have to
% be referenced using square brackets, for example:
% plot([perf_Vsweep.mu_freestream],[perf_Vsweep.CT])

% If oper.save_workspace is "on", then each loop is also saved as 
% a seperate workspace as well


switch options.sweep_type
    
    case 1 % Velocity
        perf_Vsweep = [];
        for V = options.sweep_range(1):options.sweep_range(2):options.sweep_range(3)
            flow.V        =   V;
            [perf]        =   BEMT_Carroll(blade, flow, oper, rotor, wake, options);
            perf_Vsweep   = [perf_Vsweep; perf];
        end
        
    case 2 % Freestream advance ratio (mu_freestream)
        perf_mu_freestrem_sweep = [];
        for mu_freestream = options.sweep_range(1):options.sweep_range(2):options.sweep_range(3)
            flow.V        =   mu_freestream*((oper.rpm/60).*2*pi*blade.geometry.Radius(end));
            [perf]        =   BEMT_Carroll(blade, flow, oper, rotor, wake, options);
            perf_mu_freestrem_sweep  = [perf_mu_freestrem_sweep; perf];
        end
        
    case 3 % Shaft speed
        perf_RPMsweep = [];
        for RPM = options.sweep_range(1):options.sweep_range(2):options.sweep_range(3)
            oper.rpm      =   RPM;
            [perf]        =   BEMT_Carroll(blade, flow, oper, rotor, wake, options);
            perf_RPMsweep = [perf_RPMsweep; perf];
        end
            
    case 4 % Angle of attack
        perf_AoAsweep = [];
        for AoA = options.sweep_range(1):options.sweep_range(2):options.sweep_range(3)
            flow.inflow_angle   =   AoA;
            [perf]        =   BEMT_Carroll(blade, flow, oper, rotor, wake, options);
            perf_AoAsweep = [perf_AoAsweep; perf];
        end
end


