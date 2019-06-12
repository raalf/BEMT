clc; clear;

airfoil_mid = {'MA409s2'};
i = 0;

for Re_mid = 50000:50000:300000
i = i+1;
ii = 0;
    for alpha_0 = deg2rad(-180:2:180)
    ii = ii+1;
    
    alpha_store(ii,i)    = alpha_0; 
    Re_mid_store(ii,i)   = Re_mid; 
    
    [c_l(ii,i),sc(ii,i)] = lift_coeff_lookup(alpha_0,Re_mid,airfoil_mid);     
    [c_d(ii,i)]          = drag_coeff_lookup(c_l,alpha_0,Re_mid,airfoil_mid,sc(ii,i));
    [c_m(ii,i)]          = moment_coeff_lookup(c_l,alpha_0,Re_mid,airfoil_mid,sc(ii,i));
    end
    
end

figure(3)
hold on
plot(rad2deg(alpha_store), c_m)
title('Moment coefficient')
xlabel('Angle of Attack [deg]')
ylabel('c_{d}')
grid on
hold off

pause




figure(1)
hold on
plot(rad2deg(alpha_store), c_l)
title('Lift coefficient')
xlabel('Angle of Attack [deg]')
ylabel('c_{l}')
grid on
hold off

 
figure(2)
hold on
plot(rad2deg(alpha_store), c_d)
title('Drag coefficient')
xlabel('Angle of Attack [deg]')
ylabel('c_{d}')
grid on
hold off


% Surface plots of airfoil data
x= reshape(rad2deg(alpha_store),[],1);
y= reshape(Re_mid_store,[],1);
z= reshape(c_l,[],1);
dx=1;
dy=10000;
x_edge=[floor(min(x)):dx:ceil(max(x))];
y_edge=[floor(min(y)):dy:ceil(max(y))];
[X,Y]=meshgrid(x_edge,y_edge);
Z=griddata(x,y,z,X,Y);

surf(X,Y,Z)
mesh(X,Y,Z)

xlabel('\alpha')
ylabel('Re')
zlabel('CL')


% hold on
% subplot(2,2,3)
% plot(rad2deg(alpha_store),c_m)
% title('Moment coefficient')
% xlabel('Angle of Attack [deg]')
% ylabel('c_{m}')
% ylim([-1 1])
% grid on
% hold off
% 
% hold on
% subplot(2,2,4)
% plot(rad2deg(alpha_store),sc)
% title('Stalled Condition')
% xlabel('Angle of Attack [deg]')
% ylabel('c_{m}')
% ylim([-1 1])
% grid on
% hold off


% Nice plots


figure(1)
hold on
plot(rad2deg(alpha_store(:,1)), c_m(:,1),'-k')
plot(rad2deg(alpha_store(:,2)), c_m(:,2),'--k')
plot(rad2deg(alpha_store(:,3)), c_m(:,3),':k')
plot(rad2deg(alpha_store(:,4)), c_m(:,4),'-.k')
plot(rad2deg(alpha_store(:,5)), c_m(:,5),'.-k')
hXLabel= xlabel('Angle of Attack [deg]');
hYLabel= ylabel('Lift coefficient, c_{l}');
axis([-180 180 -1 1.5])

hold off

hLegend = legend('Re = 25000','Re = 75000','Re = 125000','Re = 175000','Re = 225000');

v = get(hLegend,'title');
set(v,'string','Reynolds Number');


set( gca                       , ...
    'FontName'   , 'Arial' );
set([hLegend, gca]             , ...
    'FontSize'   , 14           );
set([hXLabel, hYLabel]  , ...
    'FontName'   , 'Arial',...
    'FontSize'   , 18          , ...
    'FontWeight' , 'bold'      );
% set( hTitle                    , ...
%     'FontSize'   , 14          , ...
%     'FontWeight' , 'bold'      );

set(gca, ...
  'Box'         , 'off'     , ...
  'TickDir'     , 'out'     , ...
  'TickLength'  , [.01 .01] , ...
  'XMinorTick'  , 'on'      , ...
  'YMinorTick'  , 'on'      , ...
  'XGrid'       , 'on'      , ...
  'YGrid'       , 'on'      , ...
  'XColor'      , [.3 .3 .3], ...
  'YColor'      , [.3 .3 .3], ...
  'LineWidth'   , 0.5       );



