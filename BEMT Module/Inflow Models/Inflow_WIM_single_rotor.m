% Inflow_WIM_single_rotor
% Iterative scheme to determine induced velocity using a vortex tube

% Initializations
epsilon = 1; count = 1;
vi = 0; ui = 0;

% Run BET
BETscript;

while epsilon > 0.0005 && count < 1000       
T_old = T;

% Run WIM
[vi_sum,P]    = WIM(flow,oper,rotor,wake,T,B,mid_span,azimuth,R,AoA_rad,wake.plot);

vix = reshape(vi_sum(:,1),[(stations-1),azm]);
viy = reshape(vi_sum(:,2),[(stations-1),azm]);
viz = reshape(vi_sum(:,3),[(stations-1),azm]);

lambda  = (viz+Vrp_a)./(omega.*R);

BETscript;

epsilon     =   abs((T - T_old)./T);
count       =   count+1;
end
