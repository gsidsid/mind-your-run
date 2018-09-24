% Execute data gathering services
connector on

%%
clear m
m = mobiledev

%%
% Poll data
clear accel
clear gyro
[accel, t_a] = accellog(m);
[gyro, t_g] = orientlog(m);

