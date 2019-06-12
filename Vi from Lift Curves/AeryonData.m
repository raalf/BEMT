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
   

    
    %plot the results if required
         
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
        
      
        