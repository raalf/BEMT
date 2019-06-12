load('airfoil5')

% Griddata approach, interpolates all entries

%     tic
%     x = airfoil5.Re;
%     y = airfoil5.alpha;
%     v = airfoil5.CL;
%     [xq,yq] = meshgrid(min(x):1000:max(x),min(y):0.1:max(y));
%     vq = griddata(x,y,v,xq,yq);
%    
% 
%     figure(1); hold on;
%     mesh(xq,yq,vq)
%     stem3(x,y,v,'bo')
%     
%     xlabel('Reynolds','FontSize',16)
%     ylabel('alpha','FontSize',16)
%     zlabel('Lift Coefficient','FontSize',16)
%     
%     camlight('headlight')
%     
%     sz = 100; % Represents number of sample of "query" points (for clocking purpose currently)
%     
%     idx = sub2ind(size(vq),randi([1, 301],1,sz), randi([1, 301],1,sz)); %  Generate linear index
%     v = vq(idx); % Query by linear index
%     toc
%     
    
    
    
% Interpolation and Extrapolation method
% 
%     tic
%     x = airfoil5.Re;
%     y = airfoil5.alpha;
%     v = airfoil5.CL;
%     [xq,yq] = meshgrid(0:1000:1250000,min(y):0.1:max(y));
%     
%     F1 = scatteredInterpolant(x,y,v,'linear','nearest');
%     vq = F1(xq,yq);
% 
%     toc
% 
%     figure(1); hold on;
%     surf(xq,yq,vq,'LineStyle','none')
%     stem3(x,y,v,'bo')
%     camlight('headlight') 
% 
%     xlabel('Reynolds','FontSize',16)
%     ylabel('alpha','FontSize',16)
%     zlabel('Lift Coefficient','FontSize',16)
    
    
% Loop existing coeff look-up functions

CL_store = [];

Re_start = 1000;
Re_end = 50000;
Re_int = 50000;

AoA_start = -pi;
AoA_end = pi;
AoA_int = 2*pi/3600;

tic

    for k = Re_start:Re_int:Re_end % Reynolds number
        for l = AoA_start:AoA_int:AoA_end % Angle of attack
            
            [c_l,stall_condition] = lift_coeff_lookup(l,k,{'airfoil5'});
           
            CL_store = [CL_store; [k l c_l stall_condition]];
           
        end
    end
    
toc
    
% griddata    
    [xq,yq] = meshgrid(Re_start:Re_int:Re_end,AoA_start:AoA_int:AoA_end);
    vq = griddata(CL_store(:,1),CL_store(:,2),CL_store(:,3),xq,yq,'cubic');
    
% scatteredInterpolation
%     [xq,yq] = meshgrid(1000:50000:500000,-pi:2*pi/3600:pi);
%     F1 = scatteredInterpolant(x,y,v,'linear','nearest');
%     vq = F1(xq,yq);

    hold on
    surf(xq,yq,vq,'LineStyle','none')
    
    xlabel('Reynolds','FontSize',16)
    ylabel('Angle of attack','FontSize',16)
    zlabel('Lift Coefficient','FontSize',16)
    
    stem3(CL_store(:,1),CL_store(:,2),CL_store(:,3))
