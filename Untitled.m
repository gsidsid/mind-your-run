figure
hold on
for angles = [1:1:length(gyrZ)]
   x = cosd(gyrX(angles));
   y = sind(gyrX(angles));
   plot([0,1],[0,0],[0,x],[0,y])
   x = cosd(gyrY(angles));
   y = sind(gyrY(angles));
   plot([0,1],[0,0],[0,x],[0,y])
   x = cosd(gyrZ(angles));
   y = sind(gyrZ(angles));
   plot([0,1],[0,0],[0,x],[0,y])
   axis([0 1.5 0 0.5])
   pause(.002)
end