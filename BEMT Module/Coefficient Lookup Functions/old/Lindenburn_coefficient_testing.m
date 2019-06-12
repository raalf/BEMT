%Lindenburg stall model
clc; clear;
i = 0;

for alpha_0 = deg2rad(-180:1:180)
        
    i = i+1;
    
    alpha_high_S = deg2rad(24);
    alpha_low_S  = deg2rad(-5);

    c_d_90 = 1.98;
    c_d_0 = 0.02;
    
    AoA = alpha_0;

    
% NOT Stalled
    if (AoA >= alpha_low_S) && (AoA <= alpha_high_S)
        % Assume when airfoil is "backwards" (+/- 180 deg) then it is also
        % in the "stalled regime"
        
        % Known data here
        
%         c_l = 2*pi*AoA;
%         c_t = c_d_0*cos(AoA);
%         c_n = (c_l + c_t*sin(AoA))/(cos(AoA));
%         c_d = c_n*sin(AoA)+c_t*cos(AoA);
%         c_m = -c_n*(0.25-(0.175*(1-(2*AoA/pi))));
      
        c_d = 0;
        c_l = 0;
        c_m = 0;

        j(i) = 0;
        
% Stalled        
    else 
        
    % Handle AoA to be within 0 -> 180 range    
    if AoA < alpha_low_S
        AoA = AoA + pi;
    elseif AoA >= (pi+alpha_low_S)
        AoA = AoA - pi; 
    end
       
    % c_l and c_d
        c_n = c_d_90*(sin(AoA))./(0.56+0.44*sin(AoA));
        c_t = 0.5*c_d_0*cos(AoA);
        c_l = c_n*cos(AoA)-c_t*sin(AoA);
        c_d = c_n*sin(AoA)+c_t*cos(AoA);
        
    % c_m
        c_n = c_d_90*(sin(AoA))./(0.56+0.44*sin(AoA));
        c_m = -c_n*(0.25-(0.175*(1-(2*AoA/pi))));

        if alpha_0<0
            c_m = -c_m;
        end
        
        
        j(i) = 1;
        
    end
    
        c_l_store(i) = c_l;
        c_t_store(i) = c_t;
        c_n_store(i) = c_n;
        c_d_store(i) = c_d;
        c_m_store(i) = c_m; 
        alpha_store(i) = alpha_0;
end

figure
subplot(2,2,1)
plot(rad2deg(alpha_store), c_l_store)
title('CL')

subplot(2,2,2)
plot(rad2deg(alpha_store), c_d_store)
title('CD')

subplot(2,2,3)
plot(rad2deg(alpha_store),c_m_store)
title('CM')

subplot(2,2,4)
plot(rad2deg(alpha_store),j)
title('Stalled (=1) vs not stalled (=0) range')
ylim([-0.5 1.5])