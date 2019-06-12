% Single- and Multi-rotor analysis tool for rotorcraft vehicles: ROTOR
%       Blade element momentum theory (BEMT) for single rotor performance
%       Potential flow based (rigid) ring vortex wake interaction model (WIM)
%       High lift devices with deployment schedule

%   	MATLAB version compatability: MATLAB 2016b +

% Instructions: (to run from this .m file)
%       Set parameters in BEMT_analysis.m [1]-[6]
%       Run
%       Calculations are output in form of "perf" structure

clc; clear;
FOLDER_ADDRESS = pwd;
if FOLDER_ADDRESS(end-10:end) == 'BEMT Module'
    addpath(genpath(FOLDER_ADDRESS(1:end-11)))
else
    addpath(genpath(FOLDER_ADDRESS))
    cd(strcat(FOLDER_ADDRESS,'\BEMT Module'))
end


%% [1] ANALYSIS TYPE AND SAVE WORKSPACE:
% -------------------------------------- 
    % Save workspace
    options.save_workspace               =   'off';         % on/off toggle to save workspace each time BEMT_Carroll is called (i.e. in performance sweeps)
    options.saved_workspace_identifier   =   'Testing';     % identifier for saved workspace structure, string     
    
    %  ANALYSIS TYPES
    %       1   -   INPUT -> Shaft Speed  / CALC -> Thrust, etc
    %       2   -   INPUT -> Thrust       / ITER CALC -> Shaft Speed, etc
    %       3   -   INPUT -> PERFORMANCE SWEEPS
    options.analysistype =   3;

        % ANALYSIS TYPE 1 -> SHAFT SPEED KNOWN
            % Input RPM in [3a]

        % ANALYSIS TYPE 2 -> THRUST KNOWN
            oper.Treq       =   5.4;                   % Required Thrust [N]
            oper.accuracy   =   0.5;                   % Thrust Allowable Percent Error

        % ANALYSIS TYPE 3 -> PERFORMANCE SWEEPS
        %       1   -   Velocity [m/s] 
        %       2   -   Advance Ratio [Dimensionless]
        %       3   -   Shaft speed [RPM]
        %       4   -   Angle of attack [deg]
            options.sweep_type =   1;
            options.sweep_range     =   [0 0.5 10];            % Start value ; Step size; End value  
        
%% [2] FLOW FIELD INFORMATION:
% ----------------------------
       flow.rho               =   1.225;               % Air density [kg/m^3]
       flow.mu                =   1.8*10^(-5);         % Dynamic viscosity [kg/m s]
       flow.V                 =   5;                   % Freestream velocity [m/s]
       flow.inflow_angle      =   10;                  % Rotor AoA [deg]. Angle between freestream and rotor plane (0 to +/- 90). 
                                                       % (Edgewise flight = 0 [deg], Propeller mode = 90 [deg])                                                
                                                       
%% [3a] ROTOR GEOMETRY, OPERATING PARAMETERS AND COEFFICIENT OPTIONS:
% ----------------------------------------------
    % Rotor .dat file
       % use file name string to link to file saved in rotor folder
       rotor.name             =   'T_motor';           

    % Rotor rotational speed [RPM] (For analysis types 1 and 3)
       oper.rpm               =   3000; 
    
     % for multiple rotor configs, modify individual rotor rpm's as needed eg:
      % oper.rpm_multi = [2000 2000 2010 2010]
       
    % Inflow method
       oper.inflow_type       =   1;                    
                                                       % 1 = Uniform momentum w/ linear inflow model: Edgewise velocity component, forward flight. (default for FF)
                                                       % 2 = Graded momentum: Small angle assumption / no swirl / linear lift curve slope (only used for axial flow and hover)
                                                       % 3 = Graded momentum: Small angle assumption / no swirl / coeffs look-ups + stall model (only used for axial flow and hover)
                                                       % 4 = Graded momentum: Large angles / with swirl (only used for axial flow and hover)
                                                       % 5 = Potential flow: Vortex tube with constant momentum assumption for single rotor wake structure
                                                       
                                                       % Default is 1: Uniform momentum theory as it handles most flight states that can be determined with momentum theory
                                                       % 2-4 are more advanced radially graded forumlations (hover and axial flight only). 
                                                       % Code reverts back to 1 for forward flight states.
                                                       % 5 is vortex theory approach for the rotor wake (verified but not validated vs experiments)
    
    % Number of azimuth positions                      % Number of blade azimuth positions [Integer]. First it at 0 or 2pi rad.                               
       oper.azimuth_num       =   8;                  
    
    % Induced velocity toggle                          % BEMT/BET toggle 'on' (BEMT), 'off' (BET, so vi = 0). Induced velocity component of inflow 
       oper.toggle_vi         =   'on';                
     
    % Viscous effects toggle and options               % Viscous effects toggle: 'on' or 'off' (c_d = 0) 
       oper.toggle_visc       =   'on'; 
       oper.alpha_zero        =   -0.03;               % If viscous effects are off, include also a zero lift angle estimate [rad]
       oper.a_0               =   2*pi;                % Lift curve slope [1/rad]
       
    % Aerodynamic coefficients options                 % To save computation time, airfoil data can be pre-computed. 
                                                       % Coeefs are queried instead of interpolated for each AoA.
                                                       % 'on' database is pre-computed but first checks to see if database exists with same resolution.
                                                       % 'off' data is interpolated for each exact AoA and station
                                                       % ('opt' special case for optimizations)
                                                       % Database naming % convention: '<airfoilname>_<AoAresolution>'
       options.toggle_precompute    =    'on';
       options.AoAresolution        =    0.5;          % Database resolution for angle of attack, [deg].
       options.REresolution         =    10000;        % Database resolution for Reynolds number.
       options.RErange_max          =    300000;       % Max Reynolds number range. Min is set @ zero. Keep @ ~ 10^6
       options.toggle_coeff_plot    =    'off';         % Toggle to generate 3D plots for Re vs AoA vs coeff for newly generated dataset
       
%% [3b] ROTOR QUICK MODIFIERS:
% ----------------------------     
       blade.modify_pitch     =   -1.5;                % Collectively add/subtract from pitch [deg]
       blade.scale_radius     =   1;                   % Scale radius by multiplier from original [default = 1]
       
%% [4] MULTIROTOR ANALYSIS TOOL: VEHICLE CONFIGURATION/WAKE OPTIONS:
% ------------------------------------------------------------------
    % Vehicle configuration
       rotor.num_rotors       =   1;
       rotor.orientation      =   'square';            % Orientation of rotors: 'diamond' (single rotor leading) or 'square' (Two rotors leading)
       rotor.armLENGTH        =   0.5;                 % Length of arm to UAV geometric center, [m].
       rotor.roll             =   0;                   % Collective vehicle roll angle along longitudinal x-axis, [deg].
       rotor.rp_twist         =   0;                   % Twist of rotor support arm, [deg].
       rotor.rp_cant          =   0;                   % Cant of rotor support arm, [deg].
       
       %rotor.armLENGTH        =   ((2+.25).*0.2286./2)./(sind(360./rotor.num_rotors./2)); 
       
    % Adjacent wake effects toggle                                                 
       oper.toggle_WIM        =    'off';              % WIM Wake interactions on/off.
                                                         
    % Vortex wake options                                                       
       wake.num_seg           =    32;                 % Number of segments in ring element
       wake.num_elements      =    50;                 % Number of ring or helix loop elements
       wake.type              =    'ring';             % Type of wake elements used (ring or helix)
       
%% [5] HIGH LIFT DEVICES:
% -----------------------
    % Load Gurney Flap Conditions
    % See separate Gurney_Setup.m for additional options
       oper.gurney            =    0;                   % 1 for ON, 0 for OFF
       
%% [6] PLOTS
    % ---------------------
    % Pre-made plotting functions

    % Blade plotting
         % Blade_planform_plot(blade);
         % Rotor_planform_plot(blade);
         % Blade_plot(blade);
         % Rotor_plot(blade);
          
    % "Rotor" plots (sectional distributions of loades, velocities, etc over TPP)     
         % PlotTPP_3D(perf,perf.lambda); % set(gca,'Position',[0.335 0.325 0.3 0.4]);
       
         
         % Bladeplot3D_L(blade,flow,oper,perf,rotor) % gif
         % Bladeplot3D_normalized_L(blade,flow,oper,perf,rotor); % gif
         
    % WIM plots  (in the progress)      
       
      % Vortex wake plotting options 
       wake.plot              =    {'off','off','off','off','on','off','off'}; % 6 char array of on/off plot toggles. See WIM function for plot types
       wake.contour_plane     =    'xy';               % Plane/axis to plot on for contour or surf plot(4 or 5): 'xy' or 'xz' (top,size views respectively)
       wake.mesh_dim          =    300;                % Mesh dimensions/spacing for surf/countour plots


       
       
       
% ----------------------------------------------------------------------------------------      
% Calculations are performed below. No other required inputs
% ----------------------------------------------------------------------------------------    

%% Initializations and calculation routine
% ----------------------------------------

    % Import Rotor Geometry
       load(strjoin(strcat('Rotors\',cellstr(rotor.name),'.mat')));
       blade.geometry         =    eval(char(rotor.name));
    
       
   % Aerodynamic coefficient pre-compute
       if strcmpi(options.toggle_precompute,'on')
        airfoil_database(blade,options)
        load('airfoil_coeff_database');
        options.airfoil_coeff_database = airfoil_coeff_database;
       end

    % Gurney Flaps options import
       if oper.gurney == 1
           run Gurney_Setup.m
       end

       BEMTfunct        =   'BEMT_Carroll';

       switch options.analysistype
           % ANALYSIS TYPE 1 -> SHAFT SPEED KNOWN
           case 1
               
               % Single rotor 
               if rotor.num_rotors == 1
                   [perf]       =   feval(char(BEMTfunct), blade, flow, oper, rotor, wake, options);
                   
                   % Multiple rotors
               else
                   
                   % Single RPM, multiple rotors
                   if exist('oper.rpm_multi','var') == 0
                       warning('Multiple shaft speeds not provided, reverting to single shaft speed')
                       [perf]       =   feval(char(BEMTfunct), blade, flow, oper, rotor, wake, options);
                       
                       % Multiple RPMs, multiple rotors
                   elseif isequal(size(oper.rpm_multi,2),size(oper.num_rotors)) == 1
                       warning('Number of shaft speeds provided do not match number of rotors, reverting to single shaft speed')
                       [perf]       =   feval(char(BEMTfunct), blade, flow, oper, rotor, wake, options);
                   else
                       for i = 1:rotor.num_rotors
                           oper.rpm = oper.rpm_multi(i);
                           [perf(i)]       =   feval(char(BEMTfunct), blade, flow, oper, rotor, wake, options);
                       end
                   end
                   
                   
                   % If WIM on/off
                   % off -> output all calcs
                   % on -> step n check
                   % Do this with for loops once rotational speed
                   % conditions are determined?
                   RPMIterateMulti
               end
               
               
               % ANALYSIS TYPE 2 -> THRUST KNOWN
           case 2
               if strcmp(oper.toggle_WIM,'off') == 1 % Single rotor, WIM off
                   [perf]       =   ThrustIterate(BEMTfunct, blade, flow, oper, rotor, wake, options)
               else  % Multiple rotors, WIM on
                   [perf]       =   ThrustIterateMulti(BEMTfunct, blade, flow, oper, rotor, wake, options)
               end
               
               % ANALYSIS TYPE 3 -> PERFORMANCE SWEEPS
           case 3
               run Performance_sweep
       end
cd(FOLDER_ADDRESS)