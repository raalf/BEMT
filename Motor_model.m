%% DC Motor Model

% https://www.precisionmicrodrives.com/tech-blog/2014/02/02/reading-motor-constants-typical-performance-characteristics

% -- Parameters of rotor
Power_rotor = 200; % [W]
RPM = 2000;
Angular_velocity = RPM/60*2*pi; % [rad/s]

Torque_rotor = Power_rotor/(2*pi*(RPM/60));

% -- Parameters of motor

KV = 435; % Motor KV, RPM/Volt
Voltage = 22.2; % Voltage of supply, V.
R = 0.031;% Terminal or wind resistance (Ohms)
Io =1.4; % No load current, A.

% -- Calculations
KVrads = KV*2*pi/60 % Speed constant, (rad/s)/V. AKA KS (Motor speed constant)
KT = 1/(KVrads); % Torque constant, Nm/A. AKA KM, Motor constant.
KE = KT; % Back EMF constant (KE = KT), V/(rad/s)

E = KE*Angular_velocity; % Back EMF






% P = Torque_rotor/

%Power_in = Current*(I*R) + (Angular_velocity * KT);



