function [perf] = ThrustIterateMulti(BEMTfunct, blade, flow, oper, rotor)
% ThrustIteration: Calculates the power setting to produce a
% required thrust through iteration


    rotor.num_rotors


       % run BEMT code with current rotor speed
       [perf]           =   feval(char(BEMTfunct),blade,flow,oper);

       
       
       % Display current iteraton results
       fprintf('Iteration %2d  RPM = %5.f  T = %5.2f  Error = %3.1f%% \n', count, oper.rpm, perf.T, error)
              
end


function [rpm] = ThrustIterateMultiOBJECTIVE(BEMTfunct, blade, flow, oper )


% f = GA_OF(blade,flow,oper,rotor,wake,options,x)


end