function res = IMU_pathfinder(accel, gyro, t)
    vi(1,:)=[0,0,0];
    pos(1,:)=[0,0,0];
    dt = mean(diff(t));
    for i = 2:length(t)
        vi(i,:) = vi(i-1,:) + directed_accel(gyro(i,:),accel(i,:)) * dt;
        pos(i,:) = pos(i-1,:) + vi(i-1,:) * dt;
    end
    res = pos;
end