%to plot multiple plans in replan procedure
close all;
clear all;
%%%%%%%%%load the obstacle

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
file_head= '20140102-203250';
%load the executed path file
path_file= sprintf('%s%s%s','../../data/',file_head,':path.txt');
log_data =fopen(path_file,'r');

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
   %if(t<0.01) 
      if_log= 1;
      t = 0;
   end

   pre_state= state;

   if(if_log== 1 && (state== 3 || state== 7) )
      reg= [reg; [t,vx,vy,vz,vw,yaw,state,x,y,z] ]; 
   end

end
%%%%%%%%%%%%%%%%%%%%%%%load the virtual obstacle
obs_rec =fopen('../../virtual_obs/v-1.0_t7.0.txt','r');
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
    
end
%%%%%%%%%load the plan
k=0;
plan_temp= [];
while(1)
    path_file= sprintf('%s%s_%d%s','../../data/',file_head,k,'.txt');
    log_data =fopen(path_file,'r');

    if log_data == -1 || k==2
       break;
    end
    plan=[];
    log_line= fgetl(log_data);
    
    while ischar(log_line)
       log_plan= textscan(log_line,'%f %f %f %f');
       x= log_plan{1};
       y= log_plan{2};
       z= log_plan{3};
       t= log_plan{4};
       plan= [plan; [x,y,z,t] ];
       log_line= fgetl(log_data);
    end
    %find the closest point to plan_temp
    temp= 0;
    temp_dis= 1e6;
    for i=1:size(plan_temp,1)
        dis= sqrt( (plan(1,1)-plan_temp(i,1))^2+ (plan(1,2)-plan_temp(i,2))^2+ (plan(1,3)-plan_temp(i,3))^2 );
        if(dis< temp_dis)
           temp= i;
           temp_dis= dis;
        end 
    end
    if temp> 0
      plan_temp= [plan_temp(1:temp,:);plan];
    else
      plan_temp= plan;
    end
    %find the closest point from reg
    i_temp= 0;
    dis_t= 100000;
    for i=1:size(reg,1)
        dis= sqrt( (plan(1,1)-reg(i,8))^2+ (plan(1,2)-reg(i,9))^2+ (plan(1,3)-reg(i,10))^2 );
        if(dis< dis_t)
           i_temp= i;
           dis_t= dis;
        end
    end
    %plot
    figure;
    hold on;
    axis auto;
    grid on;
    title('plot');
    xlabel('x(m)');
    ylabel('y(m)');
    zlabel('z(m)');
    axis([ -1, 15, -8, 5, -1, 3]);
    
    %plot the path
%     plot3( plan(:,1),plan(:,2),plan(:,3), 'r*' )
     if i_temp<5
         i_temp=5;
     end
     plot3( reg(1:i_temp,8),reg(1:i_temp,9),reg(1:i_temp,10), 'k*' );
     plot3( plan_temp(:,1),plan_temp(:,2),plan_temp(:,3), 'r*' );
     view(3);
     h= legend('exe','plan');
     set(h,'FontSize',12);
     %plot the obstacle
    r = 1.5;
    [x,y,z] = sphere;
    hSphere = surf( r*x+obs_match(i_temp,6),r*y+obs_match(i_temp,7),r*z+obs_match(i_temp,8), 'FaceColor','g' );
%     if( size(plan_temp,1)== 0 )
%        plan_temp= plan; 
%     end
    %plot ends
    
    %save
    filename= sprintf('/home/yucong/Dropbox/iros_icuas/iros2014/figs/matlab_figs/plan%d.fig',k);
    saveas(gcf,filename);
    k=k+1;
end%end while