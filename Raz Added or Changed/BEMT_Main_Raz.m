% =========================================================================
% BEMT MAIN
% Creator: Razvan Rusu - 25/04/2016
%
% This script initializes BEMT analysis inputs and rotor parameters. It
% also allows extends the capability of the analysis to iteratively
% determine the required power for a given thrust.
% =========================================================================
clc; clear; close all;

%% BEMT ANALYSIS SETUP:

    % [1] FLOW FIELD INFORMATION
    % --------------------------
       flow.rho             =   1.220;          % Air density [kg/m^3]
       flow.mu              =   1.8*10^(-5);    % Dynamic viscosity [kg/m s]
       flow.V               =   0;              % Freestream velocity [m/s] 
       flow.inflow_angle    =   90;             % Rotor AoA [deg]. Angle between inflow and rotor plane (0 to +/- 90). 
                                                % 90->0 is + AoA (flow downward through rotor plane), 0->-90 is negative AoA 
                                                % (flow upwards through rotor).                                          
                                                                    
    % [2] ROTOR GEOMETRY SETUP
    % ------------------------
       rotor                =   {'ASP_000130_02'};  %  < --- Rotor Name Here
       
     
    % [3] ROTOR OPERATING STATE + WAKE INTERACTION PROPERTIES
    % -------------------------------------------------------
    
        % Select Analysis Type
        %   1   -   INPUT -> Shaft Speed  / CALC -> Thrust, Power
        %   2   -   INPUT -> Thrust       / CALC -> Shaft Speed, Power
        
       oper.analysistype    =   2;

       switch oper.analysistype
           
           % ANALYSIS TYPE 1 -> SHAFT SPEED KNOWN
           case 1
               oper.rpm         =   3400;               % RPM [revs/min]
           
           % ANALYSIS TYPE 2 -> THRUST KNOWN
           case 2
               oper.Treq        =   5.4;                % Required Thrust [N]
               oper.accuracy    =   0.5;                % Thrust Allowable Percent Error
       end
               
    % [4] ANALYIS PARAMETERS + OPTIONS
    % --------------------------------
       oper.azimuth_num     =   8;              % Number of azimuth positions. Default, 1, is at 2pi rad.  
       oper.inflow_option   =   1;              % Inflow type toggle. 0 = Small angle assumption, No swirl. 1 = No small angles + swirl corrections 
       oper.toggle_vi       =   1;              % BEMT/BET toggle ON = 1 (BEMT), OFF = 0 (BET, so vi = 0)   

       
    % [5] QUICK MODIFIERS
    % -------------------
       oper.modify_pitch    =   0;              % Collectively add/subtract from pitch [deg]
       oper.scale_radius    =   1;
       
       
    % [6] GURNEY FLAPS
    % ----------------
    % Load Gurney Flap Conditions
    % Added by Razvan Rusu 21/03/2016
    % See separate Gurney_Setup.m for additional options
       
       oper.gurney          =   0;              % 1 for ON, 0 for OFF
            
       
%% INITIALIZATION:

    % Rotor Geometry Import
       load(strjoin(strcat('.\',rotor,'.mat')));
       blade.geometry       =   eval(char(rotor));
    
    % Gurney Flaps options import
       if oper.gurney == 1
           run Gurney_Setup.m
       end
    
       
%% BEMT Performance Predicition Function Here   

       % Specify Name of BEMT Function
       BEMTfunct        =   'BEMT_Carroll_Large_Angles_and_Swirl';
       
       switch oper.analysistype
           
           % ANALYSIS TYPE 1 -> SHAFT SPEED KNOWN
           case 1
               [perf]       =   feval(char(BEMTfunct), blade, flow, oper);
           
           % ANALYSIS TYPE 2 -> THRUST KNOWN
           case 2
               [perf]       =   ThrustIterate(BEMTfunct, blade, flow, oper);
                                     
       end
       
       perf

%% Plotting

     
