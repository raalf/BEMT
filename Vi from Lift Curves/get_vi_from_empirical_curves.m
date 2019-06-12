% bpinkney

function get_vi_from_empirical_curves()
format long g
    close all;
    % You can make the following function call with show_details = 1 for
    % all outputs, plots, etc. Conversely if you're only interested in the resulting
    % delta thrust at a certain rpm, Va(airspeed), and angle of attack, 
    % set show_details = 0

    %(rpm, Airspeed (m/s), aoa(deg), (0 => 'x1*rpm + x0' solution, 1 => 'x0' solution, show_details[outputs coeffs and plots])
    total_added_thrust_for_args = get_lift_and_vi_thrust(4000, 10, 5, 0, 1)      
end

% This function can be used to derive coefficients for induced lift and
% incuded velocity (vi), and will plot the resulting fucntions (show_details = 1)
% The Lift fucntion is per unit speed (in m/s) for the incident airstream
% The Vi function is per unit speed (in m/s), per unit angle (in rads)
% See notes on use below:

    % current code impl. uses a single coeff for 'lift' and 'vi' (try simplified_coeff = 1)
    % lift_estimate = lift_coeff*Va;
    % vi_estimate = vi_coeff*Va*AOA;
    
    % A more precise 2-coeff solution for 'lift' and 'vi' can also be derived here
    % (try simplified_coeff = 0) (this is now rpm dependent)
    % lift_estimate = [lift_coeff(1)*rpm + lift_coeff(0)]*Va;
    % vi_estimate = [vi_coeff(1)*rpm + vi_coeff(0)]*Va*AOA;
 
% The actual output of the function is simply the resulting delta_thrust 
% at the rpm, airspeed, and aoa provided in the function call
% (if you need to do this a lot, you're better off outputting the coeffs
% for lift and vi and making a separate function)

function [added_thrust] = get_lift_and_vi_thrust(rpm, Va, aoa, simplified_coeff, show_details)
    rpm_space = linspace(1500, 6500, 1000);
    
    % This test data was generated with an incident airstream of ~9 m/s on
    % our thrust jig. The resulting polyfits from an RPM sweep of
    % 2000-6000 are found below. Obviously with only one airspeed we are
    % limited in how accurate this can be for the moment.
    %1pin jul31 (black cw prop)
    poly_0mps_0aoa_1pin_july31 = [0.000046242725363  -0.030669770647880  32.807306832951035]; % control
    poly_9mps_0aoa_1pin_july31 = [0.000046690895742  -0.025460538547591  48.494594661078679]; % induced lift only (aoa = 0)
    poly_9mps_19aoa_1pin_july31 = [0.000049851396037  -0.043730440718706  30.781488324981989]; % vi and induced lift (aoa != 0)
    poly_9mps_26aoa_1pin_july31 = [0.000050453275090  -0.052396355625896  14.788619913613051];  % vi and induced lift (aoa != 0)
    poly_9mps_41aoa_1pin_july31 = [0.000051988526332  -0.080444428401102  12.730533311259110];     % vi and induced lift (aoa != 0)
    %end 1pin jul31       
    
    control = poly_0mps_0aoa_1pin_july31;
    lift_curve = [poly_9mps_0aoa_1pin_july31, 9]; % include airspeed as 4th element
    vi_curves = [poly_9mps_19aoa_1pin_july31, 9, 19;  % include airspeed as 4th element, aoa as 5th element
                 poly_9mps_26aoa_1pin_july31, 9, 26;
                 poly_9mps_41aoa_1pin_july31, 9, 41];
   
    % derive lift and vi coeffs based on the empirical curve data
    lift_coeffs = get_lift_coeff(simplified_coeff, show_details);
    vi_coeffs = get_vi_coeff(simplified_coeff, show_details);
    
    %plot the results if required
    if (show_details)        
        figure; hold on;
        plot(rpm_space, polyval(poly_0mps_0aoa_1pin_july31, rpm_space), 'r', 'LineWidth', 2);
        plot(rpm_space, polyval(poly_9mps_0aoa_1pin_july31, rpm_space));
        plot(rpm_space, polyval(poly_9mps_19aoa_1pin_july31, rpm_space));
        plot(rpm_space, polyval(poly_9mps_26aoa_1pin_july31, rpm_space));
        plot(rpm_space, polyval(poly_9mps_41aoa_1pin_july31, rpm_space));
        title('Thrust Jig Empirical Data Polyfits');
        xlabel('RPM'); ylabel('Total Thrust Out'); xlim([2000, 6000]);
        legend('Control (No Airstream)', ...
            'Horizontal Airstream (lift only)', ...
            '19° Angled Airstream (lift + vi)', ...
            '26° Angled Airstream (lift + vi)', ...
            '41° Angled Airstream (lift + vi)', 'Location', 'NorthWest');
        
        lift_coeffs
        vi_coeffs        
        plot_combined_lift_vi()
    end

    if(simplified_coeff)
        added_thrust_lift = (lift_coeffs(1))*Va;
        added_thrust_vi = (vi_coeffs(1))*Va*deg2rad(aoa);
        added_thrust = added_thrust_lift + added_thrust_vi;
    else
        added_thrust_lift = (lift_coeffs(1)*rpm + lift_coeffs(2))*Va;
        added_thrust_vi = (vi_coeffs(1)*rpm + vi_coeffs(2))*Va*deg2rad(aoa);
        added_thrust = added_thrust_lift + added_thrust_vi;
    end    
    
    function [coeff] = get_lift_coeff(linear, plot_me)
        no_zero = 1;
        % get the difference between our 'lift' curve and the 'control'
        diff_poly = (lift_curve(1:3) - control); 
        % derive the lift per unit speed (m/s)
        diff = polyval(diff_poly,rpm_space)/lift_curve(4);
        avg_diff = mean(diff);
        if avg_diff < 0
           disp('No lift (or negative lift) was generated, assuming 0 for lift coeffs'); 
           no_zero = 0;
        end    
        if(plot_me)
            figure; hold on; title('Lift curve (grams thrust per m/s incident airpseed)');
            plot(rpm_space, diff);            
            plot(rpm_space, ones(size(diff))*avg_diff, 'k--');
            legend('2-coeff lift solution', '1-coeff lift solution');
            xlabel('RPM');ylabel('Resulting Lift per unit speed (g)');
        end
        
        if(~linear)
           coeff = polyfit(rpm_space, diff, 1)*no_zero; % 2-coeff solution
        else
           coeff = avg_diff*no_zero; % 1-coeff solution
        end   
    end

    function [coeff] = get_vi_coeff(linear, plot_me)
        diff_poly_wlift = {}; 
        diff_angled = {};
        diff = {};        
        if(plot_me)
            figure; hold on; title('Vi curves (grams thrust per unit radian aoa, per unit m/s airpseed');
            xlabel('RPM');ylabel('Resulting Vi per unit speed, per unit aoa (g)');
            legend_entries = {};
        end
        for i = 1:length(vi_curves(:, 1))
           diff_poly_wlift{i} = vi_curves(i, 1:3) - control;
           % subtract off the lift component based on our earlier calculation
           diff_angled{i} = polyval(diff_poly_wlift{i},rpm_space)/vi_curves(i, 4) - polyval(lift_coeffs, rpm_space); %per unit speed, subtract off lift
           % this should now ideally be the isolated vi component (no lift)
           % get poly in per unit radian:
           diff{i} = diff_angled{i}/deg2rad(vi_curves(i, 5));
           if(plot_me)
                plot(rpm_space, diff{i});
                legend_entries{i} = sprintf('2-coeff Vi Solution from %0.0f° Angle of Attack', vi_curves(i, 5));
           end            
        end   
         
        % average sample sets to produce vi poly (this is now isolated from
        % lift, and is per unit angle (rad), and per unit airspeed (m/s)),
        % ideally these curves would all be the same already... (points to shoddy data)
        sum_diffs = zeros(size(diff{1}));
        for i = 1:length(vi_curves(:, 1))
            sum_diffs = sum_diffs + diff{i};
        end        
        mean_diffs = sum_diffs/length(vi_curves(:, 1));
        avg_mean_diffs = mean(mean_diffs);
        if(plot_me)
            plot(rpm_space, mean_diffs, 'k', 'LineWidth', 2);
            plot(rpm_space, avg_mean_diffs*ones(size(mean_diffs)), 'k--', 'LineWidth', 2);
            legend_entries{length(vi_curves(:, 1)) + 1} = '2-coeff Vi Solution from all curves';
            legend_entries{length(vi_curves(:, 1)) + 2} = '1-coeff Vi Solution from all curves';
            legend(legend_entries);
            
        end  
        if(~linear)
           coeff = polyfit(rpm_space, mean_diffs, 1); 
        else
           coeff = avg_mean_diffs;
        end
    end

    function plot_combined_lift_vi()
        % This figure shows the resulting thrust over a range of RPMs and
        % airstream angles for a 1 m/s incident airstream
        figure; hold on; title('Combined Thrust from Lift and Vi Terms [per unit airspeed (m/s)] Over RPM and Angle ranges');
       
        lift = polyval(lift_coeffs,rpm_space);
        legend_titles = {};
        max_angle = 60;
        inc = 5;
        cmap = hsv(max_angle/inc +1);
        for angle = 0:inc:max_angle
            clear vi;
            vi = polyval(vi_coeffs,rpm_space)*deg2rad(angle);
            plot(rpm_space, lift + vi, 'color', cmap(angle/5 +1, :));
            legend_titles{angle/5 +1} = strcat('Angle of Attack = ', num2str(angle), ' degrees');
        end    
        plot(rpm_space, zeros(size(rpm_space)), 'k--');
        xlabel('RPM');
        ylabel('Resulting Delta Thrust (g)');
        legend(legend_titles, 'Location', 'northwest');        
    end
end
