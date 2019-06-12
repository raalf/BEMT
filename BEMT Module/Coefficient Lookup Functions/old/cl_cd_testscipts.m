%% airfoil function test

for k = 1:1:181

a_0     =   6.2832;
alpha(k)   =   deg2rad(-91 + k);

Re_mid  =   50000;
airfoil =   {'mh116'};

[c_l(k),sc(k)]=  lift_coeff_lookup(a_0,alpha(k),0,Re_mid,airfoil); 
[c_d(k)]   =  drag_coeff_lookup(c_l,alpha(k),Re_mid,airfoil,sc(k));

end

alpha = rad2deg(alpha);

figure(1)
hold on
plot(alpha,c_l)
xlabel('alpha')
title('c_l vs AoA ')
hold off

figure(2)
hold on
plot(alpha, c_d)
xlabel('alpha')
title('c_d vs AoA')
hold off

figure(3)
hold on
plot(c_l,c_d)
title('c_d vs c_l')
xlabel('c_l')
ylabel('c_d')
hold off