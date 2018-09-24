
% -------------------------------------------------------------------------
% Organize sensor data

accel = [Acceleration.Variables];
t_a = seconds(Acceleration.Timestamp - Acceleration.Timestamp(1));
gyro = [Orientation.Variables];
t_g = seconds(Orientation.Timestamp - Orientation.Timestamp(1));
time = t_a;
dt = mean(diff(time));

time = time(450:800);
accel = accel(450:800,:);
gyro = gyro(450:800,:);

% Sampling frequency info
samplePeriod = 1/20;

clear acc
clear g_a
clear acc_mag
clear rots 
clear stationary
clear acc_magFilt
clear filtCutoff

%%
% -------------------------------------------------------------------------
% Subtract out gravity
rots = angle2dcm(deg2rad(-gyro(:,1)),deg2rad(gyro(:,2)),deg2rad(-gyro(:,3)),'ZXY');
% for i = 1:length(rots)
%     g_a(:,i) = rots(:,:,i)*[0;0;9.7];
% end
% a_g = g_a'
% accel = accel(1:length(time),:) - a_g;

% Create independent vectors
accX = accel(1:length(time),1);
accY = accel(1:length(time),2);
accZ = accel(1:length(time),3);
gyrX = gyro(1:length(time),1);
gyrY = gyro(1:length(time),2);
gyrZ = gyro(1:length(time),3);

% -------------------------------------------------------------------------
% Bandpass filtering

% Compute accelerometer magnitude
acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);

% HP filter accelerometer data
filtCutOff = 0.02;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
acc_magFilt = filtfilt(b, a, acc_mag);
accX = filtfilt(b, a, accX);
accY = filtfilt(b, a, accY);
accZ = filtfilt(b, a, accZ);

% Compute absolute value
acc_magFilt = abs(acc_magFilt);

% LP filter accelerometer data
filtCutOff = 5;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
acc_magFilt = filtfilt(b, a, acc_magFilt);
accX = filtfilt(b, a, accX);
accY = filtfilt(b, a, accY);
accZ = filtfilt(b, a, accZ);

accX = accX - mean(accX);
accY = accY - mean(accY);
accZ = accZ - mean(accZ);

% -------------------------------------------------------------------------

figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Sensor Data');
ax(1) = subplot(2,1,1);
    hold on;
    plot(time, gyrX, 'r');
    plot(time, gyrY, 'g');
    plot(time, gyrZ, 'b');
    title('Attitude');
    xlabel('Time (s)');
    ylabel('Angle');
    legend('X', 'Y', 'Z');
    hold off;
ax(2) = subplot(2,1,2);
    hold on;
    plot(time, accX(1:length(time)), 'r');
    plot(time, accY(1:length(time)), 'g');
    plot(time, accZ(1:length(time)), 'b');
    title('Accelerometer');
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    legend('X', 'Y', 'Z');
    hold off;
linkaxes(ax,'x');


% -------------------------------------------------------------------------
% Compute orientation

% Convert device acceleration and direction to acceleration in the global
% frame
for t = 1:length(time)
    acc(t,:) = rots(:,:,i)*[accX(t);accY(t);accZ(t)];
end

acc = acc(1:length(time),:);


% -------------------------------------------------------------------------
% Integrate acceleration to yield velocity

vel = zeros(size(accel));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + acc(t,:) * samplePeriod;
end

% -------------------------------------------------------------------------
% Integrate velocity to yield position

pos = zeros(size(vel));
for t = 2:length(pos)
    pos(t,:) = pos(t-1,:) + vel(t,:) * samplePeriod;    % integrate velocity to yield position
end

%%

% rots = angle2dcm(deg2rad(-gyro(:,1)),deg2rad(gyro(:,2)),deg2rad(-gyro(:,3)),'ZXY');
% for i = 1:length(rots)
%     g_a(:,i) = rots(:,:,i)*[0;0;9.8];
% end
% a_g = g_a'


figure()
axis('equal')
plot(pos(:,1)./9.8,pos(:,2)./9.8)
xlabel('x')
ylabel('y')
zlabel('z')

figure();
hold on
grid on;
h = animatedline('MaximumNumPoints', 20000);
xlabel('x')
ylabel('y')
zlabel('z')
j=1;
for i = 450:20:800
    addpoints(h,pos(i,1)./9.8,pos(i,2)./9.8,pos(i,3)./9.8);
    drawnow
    daspect([1 1 1])
    axis([-10 -6 40 75])
%     frame = getframe(gcf);
%     im=frame2im(frame);
%     [imind(:,:,1,j),cm]=rgb2ind(im,256);
    j = j+1;
    pause(1)
end

% imwrite(imind,cm,'testFig.gif','gif','Loopcount',inf)

%%
% -------------------------------------------------------------------------
% Gait Analysis

hold on
make_freq_plot(gyrX, 20)
make_freq_plot(gyrY, 20)
make_freq_plot(gyrZ, 20)
title('Frequency spectrum plot of orientation data')
legend('Pitch','Yaw','Roll')

