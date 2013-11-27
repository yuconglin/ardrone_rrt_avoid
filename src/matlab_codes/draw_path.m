%f_traj =fopen('../../bin/traj_rec.txt','r');
f_traj= fopen('/home/yucong/.ros/virtual_replan_rec.txt','r');
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

log_data =fopen('../../data/20131127-144459:path.txt','r');
if log_data == -1
     error('File log_data could not be opened, check name or path.')
end
log_line= fgetl(log_data);
reg = [];
if_log= 0;
pre_state= -1;

while ischar(log_line)
    %1383949238.534687106 0.024928 -0.122953 0.731 -176.635 -0.0367324 -0.0209688  0.0159516 -0.00222604 6
   log_reg = textscan(log_line,'%f %f %f %f %f %f %f %f %f %f');
   t = log_reg{1};
   x = log_reg{2};
   y = log_reg{3};
   z = log_reg{4};
   vx = log_reg{6};
   vy = log_reg{7};
   vz = log_reg{8};
   vw = log_reg{9};
   yaw = log_reg{5};
   state = log_reg{10};
  
   log_line= fgetl(log_data);
   
   if(pre_state== 6 && state== 4)
      if_log= 1;
      t0 = t;
   end
   
%    if(pre_state== 3 && state== 7)
%       if_log= 0;
%    end
   
   deg2rad= pi/180.;
   vx_b= vx*cos(yaw*deg2rad)+ vy*sin(yaw*deg2rad);
   vy_b= -vx*sin(yaw*deg2rad)+ vy*cos(yaw*deg2rad);
   %vx= vx_b;
   %vy= vy_b;
   
   pre_state= state;
   
   if(if_log== 1 && state== 3 || state== 7)
   %if(if_log== 1 && state ~= 2 && state ~= 8 && state ~=0 && state~=6 && state~=4)
   %if(if_log== 1)
      t= t-t0;
      reg= [reg; [t,vx,vy,vz,vw,yaw,state,x,y,z] ]; 
   end
   
end

%f_obs =fopen('../../bin/obs2ds.txt','r');
f_obs =fopen('/home/yucong/.ros/obs2ds.txt','r');
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

for i=1:1:size(reg,1)
  plot3( reg(i,8),reg(i,9),reg(i,10),'k+' );
end

view(3);