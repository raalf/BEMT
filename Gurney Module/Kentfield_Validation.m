% =========================================================================
% KENTFIELD GURNEY FLAP MODEL TEST
% Creator: Razvan Rusu - 23/04/2016
%
% This script implements the Kentfield gurney flap approximation model to
% the lift curve slope of a common airfoil used in the BEMT code
% =========================================================================
clc; clear; close all;

% Import Airfoil aerodynamic data
%load naca0012.mat;
%airfoil     =   naca0012;
load MA409.mat;
airfoil     =   MA409;


% Specify Reynolds Number
Re          =   100000;

% Extract data
I           =   find(airfoil.Re == Re);
airfoil     =   airfoil(I(1):I(end),:);
alpha       =   airfoil.alpha;
CL          =   airfoil.CL;
CD          =   airfoil.CD;
CDp         =   airfoil.CDp;

% Define Gurney Range

Gsize       =   1:5;      % Flap size in percent chord

%% Use kentfield approx. to find increase in lift

delCl       =   0.31858 * Gsize - 0.07281 * Gsize .^2 ...
                + 0.00693 * Gsize .^ 3;

% Create empty matrices
GCL         =   zeros(numel(CL), numel(Gsize));
%GCD         =   GCL;
delCD       =   GCL;

% Populate updated CL with Gurney Flaps
for i = 1:numel(Gsize)
    GCL(:,i)  =   delCl(i) + CL;  
end

%% Use Kentfield approx. to calculate increase in drag


for i = 1:numel(Gsize)
    delCD(:,i) = 0.135 * (CD .^ (-1/3)) * (Gsize(i)/100) ^ (4/3);
end

for i = 1:numel(Gsize)
    GCD(:,i)  =   delCD(:,i) + CD;  
end



%% Plot Cl - alpha
symbols = ['d','s','v','o','<','x','+','*'];

figure
plot(alpha, CL, strcat('-', symbols(1)))
title('Lift Curve for MA409 Airfoil - Re = 1E6')
xlabel('Alpha [deg]')
ylabel('Lift Coefficient C_l')
hold on

for i = 1:numel(Gsize)
    plot(alpha, GCL(:,i), strcat('-', symbols(i+1)))
end

legend('Baseline', 'd/C = 1%', 'd/C = 2%', 'd/C = 3%', 'd/C = 4%',...
    'd/C = 5%', 'Location','NorthWest')

xlim([0 10])

%% Plot CD - alpha


figure
plot(alpha, CD, strcat('-', symbols(1)))
title('Drag Curve for MA409 Airfoil - Re = 1E6')
xlabel('Alpha [deg]')
ylabel('Drag Coefficient C_d')
hold on

for i = 1:numel(Gsize)
    plot(alpha, GCD(:,i), strcat('-', symbols(i+1)))
end

legend('Baseline', 'd/C = 1%', 'd/C = 2%', 'd/C = 3%', 'd/C = 4%',...
    'd/C = 5%', 'Location','NorthWest')

xlim([0 10])


%% Plot Cl - Cd
figure
plot(CD, CL, strcat('-', symbols(1)))
title('Lift - Drag Curve for MA409 Airfoil - Re = 1E6')
xlabel('Drag Coefficient C_d')
ylabel('Lift Coefficient C_l')
hold on

for i = 1:numel(Gsize)
    plot(GCD(:,i), GCL(:,i), strcat('-', symbols(i+1)))
end

legend('Baseline', 'd/C = 1%', 'd/C = 2%', 'd/C = 3%', 'd/C = 4%',...
    'd/C = 5%', 'Location','NorthWest')

xlim([0 0.05])

%% Find percent increase in Cl_max

Clmax = max(CL);
GClmax = zeros(1,numel(Gsize));

for i = 1:numel(Gsize)
    GClmax(i) = max(GCL(:,i));
end

% Calculate percent increase in Clmax
delClmax = (GClmax - Clmax) / Clmax * 100;

% Plot Increase in Clmax with Gurney Height
figure
plot(Gsize, delClmax, strcat('-', symbols(1)))
title('Clmax Increase due to Gurney Flaps for MA409 Airfoil - Re = 1E6')
xlabel('Gurney Height in Percent Chord')
ylabel('Percent Increase in Clmax')
xlim([0 6])

