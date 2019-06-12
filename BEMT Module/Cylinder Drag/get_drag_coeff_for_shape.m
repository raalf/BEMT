% Modified Script for Cd determination for cylinder and sphere (Unmodified)

function get_drag_coeff_for_shape() %test function
 
    loglog(0); hold on; grid on;  

    [Rex_in, Cdy_in, Rex_out, Cdy_out] = generateReynoldsCurveCylinder();
    loglog(Rex_in, Cdy_in, 'bo');
    loglog(Rex_out, Cdy_out, 'b');
    
    [Rex_in, Cdy_in, Rex_out, Cdy_out] = generateReynoldsCurveSphere();
    loglog(Rex_in, Cdy_in, 'ro');
    loglog(Rex_out, Cdy_out, 'r');
    
    %legend
    h(1) = plot(NaN, NaN, 'b', 'LineWidth', 2);
    h(2) = plot(NaN, NaN, 'r', 'LineWidth', 2);    
    legend(h, {'ReynoldsCurveCylinder','ReynoldsCurveSphere'});
end

function [Rex_in, Cdy_in, Rex_out, Cdy_out] = generateReynoldsCurveCylinder()
    Rex_in=[0.1 0.3 1 3 10 40 100 300 1000 10000 100000 300000 480000 1000000];
    Cdy_in=[60 25 10 4 3 2 1.8 1.5 1 1.3 1.4 1 0.28 0.45];    
    [Rex_out, Cdy_out] = generateReynoldsCurve(Rex_in, Cdy_in, 100);
end

function [Rex_in, Cdy_in, Rex_out, Cdy_out] = generateReynoldsCurveSphere(Rex, Cdy)
    Rex_in=[0.2 0.4 1 2 6 10 30 10^2 400 2000 10000 100000 400000 1000000 6000000];
    Cdy_in=[10^2 60 25 15 6 4 2 1 0.6 0.4 0.4 0.5 0.08 0.1 0.2];
    [Rex_out, Cdy_out] = generateReynoldsCurve(Rex_in, Cdy_in, 10000);
end

function [Rex_out, Cdy_out] = generateReynoldsCurve(Rex, Cdy, points)
    decade_i = ceil(log10(Rex(1))) - 1;
    decade_f = ceil(log10(Rex(end)));
    Rex_out = logspace(decade_i, decade_f, points);
    %Cdy_out = spline(log(Rex), Cdy, log(Rex_out)); % kindof messy for sphere...
    
    Cdy_out = interp1(Rex, Cdy, Rex_out, 'pchip', 'extrap');
end

function [Dpar,Cd] = Dpar(part,flow,alpha)
 dx=part.rx*2;
 dy=part.ry*2;
 Re=flow.V*flow.rho*dx/flow.mu;
 
%  %Reynolds number lookup - cylinder
%  Rex=[0.1 0.3 1 3 10 40 100 300 1000 10000 100000 300000 1000000]; 
%  Cdy=[60 11 10 4 3 2 1.8 1.5 1 1.3 1.4 1 0.45];
%  
%  %Reynolds number lookup - sphere
%  Rex=[0.3 0.4 1.5 10 13 10^2 400 2000 10000 100000 400000 1000000 6000000];
%  Cdy=[10^2 16 12 4 2 1 0.6 0.4 0.4 0.5 0.08 0.1 0.2];
%  
 l_eff=part.l*sin(alpha); %calculate effective length based on angle of attack
 Dpar=0.5*Cd*flow.rho*flow.V^2*dy*l_eff;
end

