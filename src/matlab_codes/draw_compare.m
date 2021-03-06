close all;
%load the file
log_data =fopen('../../data/20140101-024549:path.txt','r');

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
      t0 = t;
   end
      
   pre_state= state;
   
   if(if_log== 1 && (state== 3 || state== 7) )
      t= t-t0;
      reg= [reg; [t,vx,vy,vz,vw,yaw,state,x,y,z] ]; 
   end
   
end

%load the other file
log_data =fopen('../../data/20140101-024525:other.txt','r');
if log_data == -1
     error('File log_data could not be opened, check name or path.')
end
log_line= fgetl(log_data);
reg_other = [];
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
      t0 = t-8;
   end
      
   pre_state= state;
   
   if(if_log== 1 && (state== 3 || state== 7) )
      t= t-t0;
      reg_other = [reg_other; [t,vx,vy,vz,vw,yaw,state,x,y,z] ]; 
   end
   
end

obs_match = [];
dis_rec = [];
%quad and obstacle time matching
idx = 1;
for i=1:size(reg,1)
    t= reg(i,1);
    
    if( t< reg_other(1,1) )
      obs_match = [obs_match; reg_other(1,:)];
    elseif( t> reg_other( size(reg_other,1),1) )
      obs_match = [obs_match; reg_other(size(reg_other,1),:) ];
    else
      while(1)
         if (idx+1<= size(reg_other,1) && reg_other(idx,1)<=t && t<= reg_other(idx+1,1)) || (idx+1 == size(reg_other,1) )
            break; 
         else
           idx = idx +1;
         end
      end %while ends
      
      if(idx > size(reg,1) ) 
         idx= idx-1;
      end
      %idx,t
      obs_match= [obs_match; reg_other(idx,:)];
    end
    
    dis = sqrt( (reg(i,8)-reg_other(idx,8))^2+(reg(i,9)-reg_other(idx,9))^2 +(reg(i,10)-reg_other(idx,10))^2 );
    dis_rec = [dis_rec; dis];
end
min(dis_rec)

%drawing
figure;
hold on;
axis auto;
grid on;
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
axis([ 0, 15, -2, 3, -1, 2.5 ]);
%draw every point
%for k=1:16
    %for i=1:size(reg,1)
    %unit sphere
    r = 1.5;
    [x,y,z] = sphere;
    hSphere = surf( r*x+obs_match(1,6),r*y+obs_match(1,7),r*z+obs_match(1,8) );
    
    %k=1;
    %for i=1:10*k 
    %pause(0.1);
    for i=1:size(reg,1)
        plot3( reg(i,8),reg(i,9),reg(i,10), 'r*' );
        delete(hSphere);
        hSphere = surf( r*x+obs_match(i,8),r*y+obs_match(i,9),r*z+obs_match(i,10) );
        view(3);
        drawnow;
    end
%     filename= sprintf('/home/yucong/Dropbox/iros_icuas/iros2014/figs/matlab_figs/move%d.fig',k);
%     saveas(gcf,filename);
    close all;
%end