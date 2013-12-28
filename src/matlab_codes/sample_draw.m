close all;
f_traj= fopen('../../bin/sample_test.txt','r');
if f_traj == -1
    error('File sample_test.txt could not be opened, check name or path.')
end
traj_line= fgetl(f_traj);
virtual_traj = [];
while ischar(traj_line)
   log_traj = textscan(traj_line,'%f %f %f');
   x = log_traj{1};
   y = log_traj{2};
   z = log_traj{3};
   virtual_traj = [ virtual_traj; [x,y,z] ];
   traj_line= fgetl(f_traj);
end

figure;
hold on;
axis auto;
axis equal;
grid on;
%axis([ 0, 15, -5, 5, 0, 10 ]);
title('plot');
xlabel('Easting(km)');
ylabel('Northing(km)');
zlabel('Altitude(ft)');

for i=1:1:size(virtual_traj,1)
  plot3( virtual_traj(i,1),virtual_traj(i,2),virtual_traj(i,3),'k+' );
end

%view(3);