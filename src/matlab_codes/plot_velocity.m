close all;

log_data =fopen('../../data/20131109-011716:0.0:1.0:0.0:test.txt','r');
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
   
   pre_state= state;
   
   if(if_log== 1 && state ~= 2 && state ~= 8 && state ~=0 && state~=6 && state~=4)
      t= t-t0;
      reg= [reg; [t,z,vx,vy,vz,vw,yaw,state] ]; 
   end
   
end

%plot
figure;
%hold;
%ind = find( record1(:,2) > 0);
plot( reg(:,1), reg(:,3), '--r' );
hold on;
plot( reg(:,1), reg(:,4), '--g' );
plot( reg(:,1), reg(:,5), '--b' );
legend('vx','vy','vz');

figure;
%ind= find( reg(:,6)>0 );
hold on;
ylim([-90 90]);
plot( reg(:,1), reg(:,6)*180/pi, '--k' );
plot( reg(:,1), reg(:,7), '--r' );
legend('wz','yaw');