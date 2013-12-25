%load the file
log_data =fopen('../../data/20131127-202642:path.txt','r');
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
      t0 = t;
   end
      
   pre_state= state;
   
   if(if_log== 1 && state== 3 || state== 7)
      t= t-t0;
      reg= [reg; [t,vx,vy,vz,vw,yaw,state,x,y,z] ]; 
   end
   
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

for i=1:1:size(reg,1)
  plot3( reg(i,8),reg(i,9),reg(i,10),'k+' );
end

view(3);
