figure();
hold on
grid on;
h = animatedline('MaximumNumPoints', 20000);

vi=[0;0;0]';
pos=[0;0;0]';
Fs = 0.05;
i = 0;
m.Logging = 1
hold on

while i < 100
    xyz = process(m,vi,pos,0.05)
    pos = xyz
    axis([-10 10 -10 10 -10 10])
    addpoints(h,xyz(1),xyz(2),xyz(3));
    drawnow
    pause(0.05)
end

m.Logging = 0