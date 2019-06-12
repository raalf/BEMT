   
oper.rpm=4500;
    
iii = 1;

       for ii = 0:15:90 %flow.inflow_angle
           flow.inflow_angle = ii;       
           jjj = 1;
           for jj = 0:2:12 %flow.V
               flow.V = jj;

               [perf]               =   BEMT_function(blade,flow,oper);

               P(jjj,iii) = perf.P;
               T(jjj,iii) = perf.T;
               V_plot(jjj,iii)   = jj;
               Angle_plot(jjj,iii) = ii;
               
               jjj=jjj+1;
           end
           iii=iii+1;
       end 
           
figure()
plot(V_plot, T)
legend('0','15', '30', '45', '60', '75', '90')
xlabel('Velocity')
ylabel('Thrust [N]')
title('Thrust vs Vehicle Speed For Various Rotor Plane Angles (0 being parallel to flow)')

figure()
plot(V_plot, P)       
legend('0','15', '30', '45', '60', '75', '90')
xlabel('Velocity')
ylabel('Power [W]')
title('Power vs Vehicle Speed For Various Rotor Plane Angles (0 being parallel to flow)')

figure()
plot(V_plot(:,1), T(:,1))       
xlabel('Velocity')
ylabel('Thurst [N]')
title('Thrust vs Vehicle Speed For 0 Degrees Case')

figure()
plot(V_plot(:,1), P(:,1))       
xlabel('Velocity')
ylabel('Power [W]')
title('Power vs Vehicle Speed For 0 Degrees Case')