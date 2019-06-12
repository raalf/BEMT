% Convention: Starting at azm = 0 top down, counter-clockwise numbering

% Assign all rotors the same rotational speed if not specified
if size(rpm_multi,2) == 1
    rpm_multi(1:rotor.num_rotors) = rpm_multi; 
end

WIMcount = 0;


while 1 % Loop until convergence on CT 
    
 % First iteration   
    if WIMcount == 0 && range(oper.rpm) == 0 % All rotors operating at the same rpm
        perf  =   feval(char(BEMTfunct), blade, flow, oper, rotor, wake, options);
        for i = 2:rotor.num_rotors
            perf(i) = perf(1);
        end
    elseif WIMcount == 0 && range(oper.rpm) ~= 0 % Rotors operating at different rpms
        for i = 1:rotor.num_rotors
            oper.rpm = rpm_multi(i);
            perf(i)  =   feval(char(BEMTfunct), blade, flow, oper, rotor, wake, options);
        end
 % All other iterations 
    else        
       
        %VELO = 
        for i = 1:rotor.num_rotors
            oper.rpm = rpm_multi(i);
            perf(i)  =   feval(char(BEMTfunct), blade, flow, oper, rotor, wake, options);
        end
        
    end
    
    [vi_WIM,qvr_pts] = WIM_Multi(flow,oper,rotor,wake,perf, wake.plot);
    vi_WIM_sum = sum(vi_WIM(:,:,:),3);

    
    % break condition on CT
    
    WIMcount = WIMcount+1;
end



