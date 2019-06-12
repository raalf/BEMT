function [ perf ] = ThrustIterate( BEMTfunct, blade, flow, oper )
% ThrustIteration: Calculates the power setting to produce a
% required thrust through iteration

% Find the shaft speed required to produce the required

   % Iteration parameters
   oper.rpm     =   1000;     % Initial RPM start
   stepsize     =   1000;     % RPM step size, initial
   maxiter      =   25;       % Maximum Allowable Iteration
   
   % Initialize counters and markers
   count        =   1;        % Iteration Counter
   found        =   0;        % This marker used to indicate once 
   hist         =   zeros(maxiter, 3); % Preallocate Array

   % Iteration
   while 1

       % run BEMT code with current rotor speed
       [perf]           =   feval(char(BEMTfunct),blade,flow,oper);
       error            =   (abs(perf.T - oper.Treq)/oper.Treq)*100;

       % Store thrust, rpm, error
       hist(count,1)    =   oper.rpm;
       hist(count,2)    =   perf.T;
       hist(count,2)    =   error;

       % Display current iteraton results
       fprintf('Iteration %2d  RPM = %5.f  T = %5.2f  Error = %3.1f%% \n', count, oper.rpm, perf.T, error)

       % Check if accuracy is met
       if error < oper.accuracy
           break
       end                   

       % Check conditions to determine next iteration
       if (perf.T < oper.Treq) & (found == 0)
           % This is active only prior to identifying the upper bounds
           oper.rpm = oper.rpm + stepsize;

       elseif (perf.T < oper.Treq) & (found == 1)
           % This is active only after the upper bounds is identified
           stepsize = stepsize / 2;
           oper.rpm = oper.rpm + stepsize;

       elseif perf.T > oper.Treq
           % If Thrust is greater then required, reduce step
           % increment and go back one and reduce increment.
           oper.rpm = oper.rpm - stepsize;
           stepsize = stepsize / 2;
           oper.rpm = oper.rpm + stepsize;
           % marker indicates that upper bounds is reached
           found  = 1;

       end

       % Increase the iteration counter
       count            =   count + 1;

       % Check if maximum iteration has been reached.
       if count == maxiter
           break
       end           
   end

end

