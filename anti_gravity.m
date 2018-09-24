function res = anti_gravity(gyro)
    rots = angle2dcm(deg2rad(-gyro(:,1)),deg2rad(gyro(:,2)),deg2rad(-gyro(:,3)),'ZXY');
    for i = 1:length(rots)
        g_a(:,i) = rots(:,:,i)*[0;0;9.7];
    end
    res = g_a'
end