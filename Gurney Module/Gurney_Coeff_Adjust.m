function [  c_l, c_d  ] = Gurney_Coeff_Adjust( oper, c_l, c_d )
% Gurney_Coeff_Adjust changes the lift and drag coefficients of the blades
% depending on the gurney flap size, location and deployment schedule
% stored in the oper structure
%   Takes the baseline lift and drag coefficients and adjusts them via
%   Kentfield's Approximation
 

switch oper.gurney_deploy
    
    % ALWAYS ON
    % ---------
    case 1
        % Calculate the change in lift coefficient.  Upward translation of lift
        % curve.
        delCl = 0.31858 * oper.gurney_size - 0.07281 * oper.gurney_size^2 ...
                + 0.00693 * oper.gurney_size ^ 3;

        % The change in C_l is added to the baseline C_l values
        c_l = c_l + delCl;

        % The drag coefficients are altered according to Kentfield's equation
        delCd = 0.135 * (c_d .^ (-1/3)) * (oper.gurney_size/100) ^ (4/3);
        c_d = c_d + delCd ;
    
    % STEP FUNCTION DEPLOYMENT
    % ------------------------
    case 2
        
    
    % SINUSOIDAL DEPLOYMENT
    % ---------------------
    case 3
        
        
        
end
        

end

