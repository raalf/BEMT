% =========================================================================
% GURNEY FLAP SETUP SCRIPT
% Creator: Razvan Rusu - 21/04/2016
%
% This script appends the oper structure within the BEMT_analysis.m with 
% parameters defining the size, location, and deployment schedule of 
% the gurney flaps
% =========================================================================

%% Geometric Definition

oper.gurney_size          =      1;     % Flap size in PercentChord Fraction (d/c) 
oper.gurney_start         =      0.4;   % Flap Start Location in Span Fraction [0 - 1]
oper.gurney_end           =      0.8;     % Flap End Location in Span Fraction [0 - 1]

% NOTE: The start andend locations will be approximated to the next closes
% inboard station on the blade.



%% Deployment Schedule

% Options for Deployment Profile
%   1   ->  Always ON
%   2   ->  Step Function
%   3   ->  Sinusoidal

oper.gurney_deploy      =       1;      % Gurney Deployment Profile Option

% See below for additional options
% {Azimuth Location Standard -> 0 = Donwstream, 90 = Advancing, etc..}

switch oper.gurney_deploy
    % ALWAYS ON
    % ---------
    case 1                              
        
        % No further input required
        
        
    % STEP FUNCTION DEPLOYMENT
    % ------------------------   
    case 2 
        
        % Azimuth where Gurney is Deployed in Degrees
        oper.gurney_deployON    =       0; 
        % Azimuth where Gurney is Retracted in Degrees
        oper.gurney_deployOFF   =       180;    
    
        
    % SINUSOIDAL FUNCTION DEPLOYMENT
    % ------------------------------    
    case 3
        
          
        
end




