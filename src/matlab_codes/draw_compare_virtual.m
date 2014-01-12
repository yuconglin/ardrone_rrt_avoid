close all;
%load the file
<<<<<<< HEAD
log_data =fopen('../../data/20140102-001503:path.txt','r');
=======
log_data =fopen('../../data/20140102-222404:path.txt','r');
>>>>>>> a77b4e94a11f486e5e45af949ee80fa3f48cc3e9
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
   
   if(pre_state~= 4 && state== 4)
      if_log= 1;
      t=0;
   end
%    if(t<10) 
%      if_log= 1;
%    end
      
   pre_state= state;
   
   if(if_log== 1 && (state== 3 || state== 7) )
      %t= t-t0;
      reg= [reg; [t,vx,vy,vz,vw,yaw,state,x,y,z] ]; 
   end
   
end

%logged virtual file
<<<<<<< HEAD
obs_rec =fopen('../../virtual_obs/v-1.0_t7.0.txt','r');
=======
obs_rec =fopen('../../virtual_obs/static1.txt','r');
>>>>>>> a77b4e94a11f486e5e45af949ee80fa3f48cc3e9
if obs_rec == -1
    error('File obs_rec.txt could not be opened, check name or path.' ); 
end
obs_line= fgetl(obs_rec);
obs = [];
while ischar(obs_line)
   log_obs = textscan(obs_line,'%f %f %f %f %f %f %f %f');
   t = log_obs{1};
   if t> 0
     if size(obs,1)== 0
        t0 = log_obs{1};
     end
     vx = log_obs{2};
     vy = log_obs{3};
     vz = log_obs{4};
     w = log_obs{5};
     x = log_obs{6};
     y = log_obs{7};
     z = log_obs{8};
     obs = [ obs; [t-t0,vx,vy,vz,w,x,y,z] ];
   end%if ends
   obs_line= fgetl(obs_rec);
end

obs_match = [];
dis_rec = [];
%quad and obstacle time matching
idx = 1;
for i=1:size(reg,1)
    t= reg(i,1);
    
    if( t< obs(1,1) )
      obs_match = [obs_match; obs(1,:)];
    elseif( t> obs( size(obs,1),1) )
      obs_match = [obs_match; obs(size(obs,1),:) ];
    else
      while(1)
         if (idx+1<= size(obs,1) && obs(idx,1)<=t && t<= obs(idx+1,1)) || (idx+1 == size(obs,1) )
            break; 
         else
           idx = idx +1;
         end
      end %while ends
      %idx,t
      obs_match= [obs_match; obs(idx,:)];
    end
    
    dis = sqrt( (obs(idx,6)-reg(i,8))^2+(obs(idx,7)-reg(i,9))^2 +(obs(idx,8)-reg(i,10))^2 );
    dis_rec = [dis_rec; dis];
end
min(dis_rec)

%drawing
figure;
hold on;
axis auto;
grid on;
title('plot');
xlabel('Easting(km)');
ylabel('Northing(km)');
zlabel('Altitude(ft)');
%unit sphere
r = 1.5;
[x,y,z] = sphere;
hSphere = surf( r*x+obs_match(1,6),r*y+obs_match(1,7),r*z+obs_match(1,8) );
%draw every point
for i=1:size(reg,1)
    %pause(0.1);
    plot3( reg(i,8),reg(i,9),reg(i,10), 'r*' );
    delete(hSphere);
    hSphere = surf( r*x+obs_match(i,6),r*y+obs_match(i,7),r*z+obs_match(i,8) );
    view(3);
    drawnow;
end