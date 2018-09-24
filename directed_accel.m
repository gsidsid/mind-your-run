function res = directed_accel(gyro, accel)
    rots = angle2dcm(deg2rad(-gyro(:,1)),deg2rad(gyro(:,2)),deg2rad(-gyro(:,3)),'ZXY');
    d_a(:,1) = rots(:,:,1)*[accel(1);accel(2);accel(3)];
    res = d_a';
end