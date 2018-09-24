% Path validation
% 
accel = [Acceleration.Variables]
t_a = seconds(Acceleration.Timestamp - Acceleration.Timestamp(1));
gyro = [Orientation.Variables]
t_g = seconds(Orientation.Timestamp - Orientation.Timestamp(1));

[b a] = butter(1,0.2/(20/2),'high');
accel(:,1) = filter(b,a,accel(:,1));
accel(:,2) = filter(b,a,accel(:,2));
accel(:,3) = filter(b,a,accel(:,3));

g = anti_gravity(gyro);
accel = accel - g(length(gyro),:);

xyz = IMU_pathfinder(accel, gyro, t_g)
figure();
hold on
grid on;
h = animatedline('MaximumNumPoints', 20000);
xlabel('x')
ylabel('y')
zlabel('z')
for i = 1:20:length(xyz)
    addpoints(h,xyz(i,1),xyz(i,3),xyz(i,2));
    drawnow
    pause(0.05)
end
