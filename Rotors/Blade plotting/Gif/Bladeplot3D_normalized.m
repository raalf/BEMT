function Bladeplot3D_normalized(blade,flow,oper,perf,rotor)
% Normalized Blade Stall Diagram
    % Make this into its own function in script
    
    figure() 
        for azm = 1:1:oper.azimuth_num
            hold on
            theta = perf.azimuth(azm)-((2*pi)/(2*oper.azimuth_num)):...
            2*pi/(oper.azimuth_num*height(blade.geometry)):...
            perf.azimuth(azm)+((2*pi)/(2*oper.azimuth_num));
        
            xx = bsxfun(@times,perf.x_span,cos(theta));
            yy = bsxfun(@times,perf.x_span,sin(theta));
            zz = repmat(perf.dCt_dx(:,azm)-perf.dCt_dx(:,oper.azimuth_num),1,length(theta));
            h  = surf(xx,yy,zz,'EdgeAlpha',0.25)
            %set(h,'LineStyle','none') %black mesh lines in plot (comment out for them on)
            
            %% plot of blade here (fill)
            
            hold off
                   % Flow direction marker
            hold on
            plot([1.5,1.25],[0,0],'r-','LineWidth',3);
            plot(1.25,0,'r<', 'MarkerSize',7,'MarkerFaceColor','red');
            plot([1.25,1.25],[0,.25],'b-','LineWidth',2);
            plot(1.25,0.25,'b^', 'MarkerSize',7,'MarkerFaceColor','blue')
            hold off
        
%             axis vis3d
%             axis equal
            axis ([-1 1 -1 1 0 .01]);
            axis off;
%             xlabel('x')
%             ylabel('y')
%             zlabel('z') 


           title('Difference in Thrust Distribution Along Blade per Rev (N)')
           view(-30, 40);
            
            % gif utilities
            set(gcf,'color','w'); % set figure background to white
            drawnow;
            frame = getframe(1);
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);
            outfile = 'Bladeplot3D_normalized.gif';
 
            % On the first loop, create the file. In subsequent loops, append.
            if azm==1
                imwrite(imind,cm,outfile,'gif','DelayTime',0.1,'loopcount',inf);
            else
                imwrite(imind,cm,outfile,'gif','DelayTime',0.1,'writemode','append');
            end
 
        end
end 