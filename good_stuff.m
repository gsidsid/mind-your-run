% load mail_room.mat
% start = 50;
% finish = 3200;
% 
% Fs = 20;

load mail_room.mat
start = 50;
finish = 3200;


L1 = 42*0.0254;
L2 = 9*0.0254;
m = 150/2.25;
g = 9.81;
accel = table2array(Acceleration);
accel = [(0:size(accel,1)-1)'/20 accel];

t = accel(start:finish,1);
Fs = 1/(t(2) - t(1));
ax = accel(start:finish,2);
ay = accel(start:finish,3);
az = accel(start:finish,4);

gyro = table2array(Orientation);
gyro = [(0:size(gyro,1)-1)'/20 gyro];

t_gyro = gyro(start:finish,1);
yaw = gyro(start:finish,2);
pitch = gyro(start:finish,3);
roll = gyro(start:finish,4);

yaw = yaw + 360*(yaw < 0);
pitch = pitch + 360*(pitch < 0);
roll = roll + 360*(roll < 0);

%%
theta = 90 - pitch;
theta = deg2rad(theta);

%%
theta_d = diff(theta)./diff(t);

xtc = 2*L1*sin(theta);
N = size(theta, 1);
freq = ((0:N-1)-(N + mod(N,2))/2)*Fs/N;


v = 0;
for i = 1:size(theta_d,1)
    if theta_d(i) < 0
       v(i+1) = -(L1)*theta_d(i);
    else
       v(i+1) = v(i);
    end
end

tc = xtc./v';
%%
V = fftshift(fft(v));

k_width = 100;
kernel = ones(k_width, 1)./k_width;

v_smooth = conv(v, kernel);
v_smooth = v_smooth(1,(k_width - mod(k_width, 2))/2 + 1:not(mod(k_width,2)) + size(v_smooth,2) - (k_width - mod(k_width, 2))/2);
plot(t, v_smooth)

%%
half = (k_width - mod(k_width,2))/2;
stride_freq = [];
for i = half +1: (finish-start) -half
    x = theta(i - half : i + half);
    X = fft(x);
    X = abs(X(1:half));
    [val, ind] = sort(X, 'descend');
    
    stride_freq = [stride_freq val(2)/(2*half)*Fs];
end

plot(stride_freq)

%%
t_final = t(half +1:size(t, 1)-half -1);
v_final = v_smooth(half +1:size(v_smooth, 2)-half -1);
stride_length = v_final./stride_freq;
figure
hold on
plot(t_final, v_final)
plot(t_final, stride_freq)
plot(t_final, stride_length)
hold off
legend('Average Speed (m/s)', 'Stride Rate (Hz)', 'Average Stride Length (m)', 'Location', 'best')
