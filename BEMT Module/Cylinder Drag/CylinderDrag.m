% Modified Script for Cd determination for cylinder and sphere (Unmodified)
 
function [Dpar,Cd_cyl] = CylinderDrag(flow,part)

[Rex_in, Cdy_in, Rex_out, Cdy_out] = generateReynoldsCurveCylinder();

 dx=part.rx*2;
 dy=part.ry*2;
 Re=flow.V*flow.rho*dx/flow.mu;
  
 l_eff=part.l*sind(flow.inflow_angle); %calculate effective length based on angle of attack
 Cd_cyl = interp1(Rex_out,Cdy_out,Re, 'pchip', 'extrap'); %interpolate Cd at Re
 Dpar=0.5*Cd_cyl*flow.rho*flow.V^2*dy*l_eff;
end

function [Rex_in, Cdy_in, Rex_out, Cdy_out] = generateReynoldsCurveCylinder()
    Rex_in=[0.1 0.3 1 3 10 40 100 300 1000 10000 100000 300000 480000 1000000];
    Cdy_in=[60 25 10 4 3 2 1.8 1.5 1 1.3 1.4 1 0.28 0.45];    
    [Rex_out, Cdy_out] = generateReynoldsCurve(Rex_in, Cdy_in, 100);
end

function [Rex_out, Cdy_out] = generateReynoldsCurve(Rex, Cdy, points)
    decade_i = ceil(log10(Rex(1))) - 1;
    decade_f = ceil(log10(Rex(end)));
    Rex_out = logspace(decade_i, decade_f, points);   
    Cdy_out = interp1(Rex, Cdy, Rex_out, 'pchip', 'extrap');
end
