% Plot wake spacing as a function of V

% Plot chi as a function of V


rho = 1.225;
R = 0.381/2;
n = 4500;
T = 14;
B = 2;
AoA=deg2rad(0);

i = 0;


for V = 0:1:100
    i = i+1;
    
    chi(i)  =  atan((V*cos(AoA))/(V*sin(AoA)+sqrt(T/(2*pi*rho*(R^2))))); %calculate skew angle of wakes for influencing rotor

    z(i) = (1/(n*B))*(V*sin(AoA)+sqrt((V*sin(AoA))^2+(2*T/(rho*pi*R^2))^2));
    
    z2(i) = (1/n*B)*(sqrt(T/(2*pi*rho*(R^2)))*cos(chi(i))+V*sin(AoA)); % New method
    
    z3(i) = (1/n*B)*(sqrt(T/(2*pi*rho*(R^2)))+V*sin(AoA))*cos(chi(i)); % New new method
    
    
    h(i) = (1/(n*B))*((sqrt(T/(2*pi*rho*(R^2)))+V*sin(AoA))*sin(chi(i)) + V*cos(AoA));
    
    Vv(i) = V; 
end

%% Plot with same Ct for different RPM
% figure(1)
% hold on
% plot(Vv,rad2deg(chi),'-','LineWidth',2);
% hold off
% hTitle = title('Rotor skew angle, \chi, versus V_{\infty}. \alpha = 0');
% hXLabel= xlabel('V [m/s]');
% hYLabel= ylabel('\chi [deg]');
% %hLegend = legend('5','10','20');
% 
% 
% v = get(hLegend,'title');
% set(v,'string','Thrust Loading [N]');
% 
% 
% set( gca                       , ...
%     'FontName'   , 'CMU Sans Serif' );
% set([hTitle, hXLabel, hYLabel ], ...
%     'FontName'   , 'CMU Sans Serif');
% set([hLegend, gca]             , ...
%     'FontSize'   , 10           );
% set([hXLabel, hYLabel]  , ...
%     'FontSize'   , 14          );
% set( hTitle                    , ...
%     'FontSize'   , 14          , ...
%     'FontWeight' , 'bold'      );
% 
% set(gca, ...
%   'Box'         , 'off'     , ...
%   'TickDir'     , 'out'     , ...
%   'TickLength'  , [.02 .02] , ...
%   'XMinorTick'  , 'on'      , ...
%   'YMinorTick'  , 'on'      , ...
%   'XGrid'       , 'on'      , ...
%   'YGrid'       , 'on'      , ...
%   'LineWidth'   , .5       ,...
%   'XColor'      , [.3 .3 .3], ...
%   'YColor'      , [.3 .3 .3], ...
%   'LineWidth'   , 1       );




%%
figure(2)
hold on
plot(Vv,z,'sq');
plot(Vv,z2,'--','LineWidth',3);
plot(Vv,z3,'-.k','LineWidth',2);
hold off
title('Wake axial spacing, z_{Axial} (wrt to local Rplane coord-system), versus V_{\infty}. \alpha = 0 ')
xlabel('V [m/s]')
ylabel('z_{A} [m]')

% hLegend = legend('5','10','20');
% 
% v = get(hLegend,'title');
% set(v,'string','Thrust Loading [N]');
% 
% 
% set( gca                       , ...
%     'FontName'   , 'CMU Sans Serif' );
% set([hTitle, hXLabel, hYLabel ], ...
%     'FontName'   , 'CMU Sans Serif');
% set([hLegend, gca]             , ...
%     'FontSize'   , 10           );
% set([hXLabel, hYLabel]  , ...
%     'FontSize'   , 14          );
% set( hTitle                    , ...
%     'FontSize'   , 14          , ...
%     'FontWeight' , 'bold'      );
% 
% set(gca, ...
%   'Box'         , 'off'     , ...
%   'TickDir'     , 'out'     , ...
%   'TickLength'  , [.02 .02] , ...
%   'XMinorTick'  , 'on'      , ...
%   'YMinorTick'  , 'on'      , ...
%   'XGrid'       , 'on'      , ...
%   'YGrid'       , 'on'      , ...
%   'LineWidth'   , .5       ,...
%   'XColor'      , [.3 .3 .3], ...
%   'YColor'      , [.3 .3 .3], ...
%   'LineWidth'   , 1       );




figure(3)
hold on
plot(Vv,h,'-');
hold off
title('Wake tangential spacing, h_{Axial} (wrt to local Rplane coord-system), versus V_{\infty}. \alpha = 0 ')
xlabel('V [m/s]')
ylabel('h_{t} [m]')

