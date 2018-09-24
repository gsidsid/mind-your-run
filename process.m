function res = process(m, vi, pos, dt)
    gyro = m.Orientation
    accel = m.Acceleration
    rotg = angle2dcm(deg2rad(-gyro(1)),deg2rad(gyro(2)),deg2rad(-gyro(3)),'YXZ');
    accel = accel - rotg*[0;0;9.3];
    vi = vi + directed_accel(gyro,accel) .* dt
    pos = pos + vi * dt;
    res = pos;
end