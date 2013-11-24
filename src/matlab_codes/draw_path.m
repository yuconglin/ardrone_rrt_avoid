f_traj =fopen('../../bin/traj_rec.txt','r');
if f_traj == -1
    error('File traj_rec.txt could not be opened, check name or path.')
end
traj_line= fgetl(f_traj);
virtual_traj = [];
while ischar(traj_line)
   log_traj = textscan(traj_line,'%f %f %f %f');
   x = log_traj{1};
   y = log_traj{2};
   z = log_traj{3};
   t = log_traj{4};
   virtual_traj = [ virtual_traj; [x,y,z,t] ];
   traj_line= fgetl(f_traj);
end

f_obs =fopen('../../bin/obs2ds.txt','r');
if f_obs == -1
    error('File obstacles.txt could not be opened, check name or path.')
end
obs_line= fgetl(f_obs);
obs = [];
while ischar(obs_line)
   log_obs = textscan(obs_line,'%f %f %f %f');
   x =log_obs{1};
   y =log_obs{2};
   r =log_obs{3};
   del_r =log_obs{4};
   obs = [ obs; [x,y,r,del_r] ];
   obs_line= fgetl(f_obs);
end

figure;
hold on;
axis auto;
grid on;
%axis([ 0, 15, -5, 5, 0, 10 ]);
title('plot');
xlabel('Easting(km)');
ylabel('Northing(km)');
zlabel('Altitude(ft)');

for j=1:size(obs,1) 
 x_c = obs(j,1);
 y_c = obs(j,2);
 r = obs(j,3);
 dr = obs(j,4);
 [x,y,z] = cylinder(r+dr);
 ob_x= x+ x_c;
 ob_y= y+ y_c;
 surf( ob_x, ob_y, z );
end

for i=1:1:size(virtual_traj,1)
  %t = virtual_traj(i,4);
  plot3( virtual_traj(i,1),virtual_traj(i,2),virtual_traj(i,3), 'r*' );
end
view(3);