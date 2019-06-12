% Test Raz
close all

figure
plot(perf.r_mid, perf.c_l1(:,1),'-<', perf.r_mid, perf.c_l(:,1),'->')
title('Lift coefficient')
legend('Baseline', 'With Gurney')


figure
plot(perf.r_mid, perf.c_d1(:,1), '-<', perf.r_mid, perf.c_d(:,1), '-<')
title('Drag coefficient')
legend('Baseline', 'With Gurney')


figure
plot(perf.r_mid, perf.dT(:,1), '-<')
title('Thrust Distribution')
legend('Baseline')