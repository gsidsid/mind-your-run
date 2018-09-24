[b, a] = butter(2,0.8/(20/2),'high');

clear accelf
clear accelg

%%
g = anti_gravity(gyro);
accelg = accel(1:length(gyro),:) - g;
% accelg = [accel(1:length(gyro),1), accel(1:length(gyro),2) - g(:,2), accel(1:length(gyro),3)];
% accel f already seems to have dealt with gravity...
figure()
hold on;
plot(accelg(:,1))
plot(accel(:,1))
hold off

figure()
hold on;
plot(accelg(:,2))
plot(accel(:,2))
hold off

figure()
hold on;
plot(accelg(:,3))
plot(accel(:,3))
hold off

%%

figure()
hold on;
plot(accelg(:,1))
accelf(:,1) = filter(b,a,accelg(:,1));
plot(accelf(:,1))
hold off;

figure()
hold on;
plot(accelg(:,2))
accelf(:,2) = filter(b,a,accelg(:,2));
plot(accelf(:,2))
hold off;

figure()
hold on;
plot(accelg(:,3))
accelf(:,3) = filter(b,a,accelg(:,3));
plot(accelf(:,3))
hold off


%%
time = t_g;
acceleration = [0,0,0];

for i = 2:length(time)
    acceleration(i,:) = acceleration(i-1,:) + directed_accel(gyro(i,:),accelf(i,:));
end

velocity = cumtrapz(time,acceleration);
position = cumtrapz(time,velocity);

plot3(position(:,1),position(:,2),position(:,3))
