function Bladeplot3D(blade,flow,oper,perf,rotor)

    figure();
        for azm = 1:1:oper.azimuth_num
            hold on
            theta = perf.azimuth(azm)-((2*pi)/(2*oper.azimuth_num)):...
            2*pi/(oper.azimuth_num*height(blade.geometry)):...
            perf.azimuth(azm)+((2*pi)/(2*oper.azimuth_num));
        
            xx = bsxfun(@times,perf.x_span,cos(theta));
            yy = bsxfun(@times,perf.x_span,sin(theta));
            zz = repmat(perf.dCt_dx(:,azm),1,length(theta));           
            forces(azm) = surf(xx,yy,zz,'EdgeAlpha',0);
            forces2(azm) = surf(-xx,-yy,zz,'EdgeAlpha',0);
            
  
Time_to_loop    = 5; 
visible_length  = 1;
fade_length     = 20;
           
            

%Effects                
           %Initialize
            fade(azm) = 1;
            fade2(azm) = 1;
               
            if azm>visible_length
            fade(1:azm-visible_length) = fade(1:azm-visible_length)-(1/fade_length);
            fade(fade<0) = 0;
            set(forces(azm-visible_length),'edgecolor','none');
            for ii = 1:azm
            alpha(forces(ii),fade(ii));
            end
            fade2(1:azm-visible_length) = fade2(1:azm-visible_length)-(1/fade_length);
            fade2(fade2<0) = 0;
            set(forces2(azm-visible_length),'edgecolor','none');
            for ii = 1:azm
            alpha(forces2(ii),fade2(ii));
            end
            end

            if azm>1
            set(bla,'Visible','off');
            set(bla2,'Visible','off');
            end
            zzz = repmat(perf.dCt_dx(:,azm).*0,1,length(theta));
            bla = surf(xx,yy,zzz,'EdgeAlpha',0.25);
            bla2 = surf(-xx,-yy,zzz,'EdgeAlpha',0.25);
            
            %fill(perf.x_span,blade.geometry{'Chord'});
            
            hold off

            % Flow direction marker
            hold on
            plot([1.5,1.25],[0,0],'r-','LineWidth',3);
            plot(1.25,0,'r<', 'MarkerSize',7,'MarkerFaceColor','red');
            plot([1.25,1.25],[0,.25],'b-','LineWidth',2);
            plot(1.25,0.25,'b^', 'MarkerSize',7,'MarkerFaceColor','blue');
            hold off
        
%             axis vis3d
%             axis equal
            axis ([-1 1 -1 1 0 .35]);
            axis off;
%             xlabel('x')
%             ylabel('y')
%             zlabel('z') 


            title('Thrust Coefficient Distribution per Rev (N)');
            view(-30, 40);
            
            % gif utilities
            set(gcf,'color','w'); % set figure background to white
            drawnow;
            frame = getframe(1);
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);
            outfile = 'Bladeplot3D.gif';
 
            

            % On the first loop, create the file. In subsequent loops, append.
            if azm==1
                imwrite(imind,cm,outfile,'gif','DelayTime',Time_to_loop/oper.azimuth_num,'loopcount',inf);
            else
                imwrite(imind,cm,outfile,'gif','DelayTime',Time_to_loop/oper.azimuth_num,'writemode','append');
            end
 
        end
end
