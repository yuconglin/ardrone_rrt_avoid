close all;

%log_data =fopen('../../data/20131111-160035:0.5:0.0:0.0:test.txt','r');
%log_data =fopen('../../data/20131111-154548:1.0:0.0:0.0:good.txt','r');
log_data =fopen('../../data/20140113-000615:1.0:0.0:0.0:0.0:u.txt','r');

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
   
   if(pre_state==4 && state== 4)
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
   
   if(if_log== 1 && state ==3)
   %if(if_log== 1 && state ~= 2 && state ~= 8 && state ~=0 && state~=6 && state~=4)
   %if(if_log== 1)
      t= t-t0;
      reg= [reg; [t,z,vx,vy,vz,vw,yaw,state,vx_b,vy_b,x,y] ]; 
   end
   
end

% %plot
% figure;
% plot( reg(:,1), reg(:,9), '--r' );
% hold on;
% plot( reg(:,1), reg(:,10), '--g' );
% legend('vx_b','vy_b');
% 
% figure;
% %hold;
% %ind = find( record1(:,2) > 0);
% plot( reg(:,1), reg(:,3), '--r' );
% hold on;
% plot( reg(:,1), reg(:,4), '--g' );
% plot( reg(:,1), reg(:,5), '--b' );
% legend('vx','vy','vz');
% 
% figure;
% %ind= find( reg(:,6)>0 );
% hold on;
% ylim([-90 90]);
% plot( reg(:,1), reg(:,6)*180/pi, '--k' );
% plot( reg(:,1), reg(:,7), '--r' );
% legend('wz','yaw');


% compare_data= fopen('../../data/update_rec.txt','r');
% if compare_data == -1
%      error('File compare_data could not be opened, check name or path.')
% end
% log_line= fgetl(compare_data);
% reg_c = [];
% 
% while ischar(log_line)
%     %0.1 0.0160922 0 1 0 0.107281 0 0 0
%    log_reg = textscan(log_line,'%f %f %f %f %f %f %f %f %f');
%    t = log_reg{1};
%    x = log_reg{2};
%    y = log_reg{3};
%    z = log_reg{4};
%    yaw = log_reg{5};
%    vx = log_reg{6};
%    vy = log_reg{7};
%    vz = log_reg{8};
%    v_yaw = log_reg{9};
%      
%    log_line= fgetl(compare_data);
%    
%    reg_c= [reg_c; [t,x,y,z,yaw,vx,vy,vz,v_yaw] ]; 
%    
% end
% 
% figure;
% plot( reg_c(:,1), reg_c(:,6), '--r' );
% hold on;
% plot( reg_c(:,1), reg_c(:,7), '--g' );
% plot( reg_c(:,1), reg_c(:,8), '--b' );
% legend('vx','vy','vz');
% 
% figure;
% hold on;
% ylim([-90 90]);
% plot( reg_c(:,1), reg_c(:,9)*180/pi, '--k' );
% plot( reg_c(:,1), reg_c(:,5), '--r' );
% legend('yaw_rate','yaw');
% 
% figure;
% hold on;
% plot( reg_c(:,1), reg_c(:,2), '*r' );
% plot( reg_c(:,1), reg_c(:,3), '*g' );
% plot( reg_c(:,1), reg_c(:,4), '*b' );
% plot( reg(:,1), reg(:,11), '-r' );
% plot( reg(:,1), reg(:,12), '-g' );
% plot( reg(:,1), reg(:,2), '-b' );
% legend('x_c','y_c','z_c','x','y','z');
compare_data= fopen('../../data/update_rec.txt','r');
if compare_data == -1
     error('File compare_data could not be opened, check name or path.')
end
log_line= fgetl(compare_data);
reg_c = [];

while ischar(log_line)
    %0.1 0.0160922 0 1 0 0.107281 0 0 0
   log_reg = textscan(log_line,'%f %f %f %f %f %f %f %f %f');
   t = log_reg{1};
   x = log_reg{2};
   y = log_reg{3};
   z = log_reg{4};
   yaw = log_reg{5};
   vx = log_reg{6};
   vy = log_reg{7};
   vz = log_reg{8};
   v_yaw = log_reg{9};
     
   log_line= fgetl(compare_data);
   
   reg_c= [reg_c; [t,x,y,z,yaw,vx,vy,vz,v_yaw] ]; 
   
end

%plot velocity compare
figure;
xlabel('t(s)');
ylabel('velocity(m/s)');
hold on;
plot( reg_c(:,1), reg_c(:,6), '*r' );
plot( reg_c(:,1), reg_c(:,7), '*g' );
plot( reg_c(:,1), reg_c(:,8), '*b' );
plot( reg(:,1), reg(:,3), '--r','LineWidth',2 );
plot( reg(:,1), reg(:,4), '--g','LineWidth',2 );
plot( reg(:,1), reg(:,5), '--b','LineWidth',2 );
h1= legend('vx_c','vy_c','vz_c','vx','vy','vz');
set(h1,'FontSize',12);

%plot angular
figure;
xlabel('t(s)');
ylabel('yaw(degree)/ yaw\_rate(degree/s)');
hold on;
ylim([-90 90]);
plot( reg_c(:,1), reg_c(:,9)*180/pi, '*k' );
plot( reg_c(:,1), reg_c(:,5), '*b' );
plot( reg(:,1), reg(:,6)*180/pi, '--k','LineWidth',2 );
plot( reg(:,1), reg(:,7), '--r', 'LineWidth',2);
h2= legend('yaw\_rate_c','yaw_c','yaw\_rate','yaw');
set(h2,'FontSize',12);

figure;
xlabel('t(s)');
ylabel('position(m)');
hold on;
plot( reg_c(:,1), reg_c(:,2), '*r' );
plot( reg_c(:,1), reg_c(:,3), '*g' );
plot( reg_c(:,1), reg_c(:,4), '*b' );
plot( reg(:,1), reg(:,11), '-r','LineWidth',2);
plot( reg(:,1), reg(:,12), '-g','LineWidth',2);
plot( reg(:,1), reg(:,2), '-b','LineWidth',2);
h3= legend('x_c','y_c','z_c','x','y','z');
set(h3,'FontSize',12);
