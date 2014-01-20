close all;
clear all;
% %part2
% f_traj =fopen('../../data/20140101-032724_2.txt','r');
% %f_traj= fopen('/home/yucong/.ros/virtual_replan_rec.txt','r');
% if f_traj == -1
%     error('File traj_rec.txt could not be opened, check name or path.')
% end
% traj_line= fgetl(f_traj);
% vtraj_2 = [];
% while ischar(traj_line)
%    log_traj = textscan(traj_line,'%f %f %f %f');
%    x = log_traj{1};
%    y = log_traj{2};
%    z = log_traj{3};
%    t = log_traj{4};
%    vtraj_2 = [ vtraj_2; [x,y,z,t] ];
%    traj_line= fgetl(f_traj);
% end
%part1

f_traj =fopen('../../data/20140102-001846_1.txt','r');
if f_traj == -1
    error('File traj_rec.txt could not be opened, check name or path.')
end
traj_line= fgetl(f_traj);
vtraj_1 = [];
while ischar(traj_line)
   log_traj = textscan(traj_line,'%f %f %f %f');
   x = log_traj{1};
   y = log_traj{2};
   z = log_traj{3};
   t = log_traj{4};
   vtraj_1 = [ vtraj_1; [x,y,z,t] ];
   traj_line= fgetl(f_traj);
end
%part0
f_traj =fopen('../../data/20140102-001846_0.txt','r');
if f_traj == -1
    error('File traj_rec.txt could not be opened, check name or path.')
end
traj_line= fgetl(f_traj);
vtraj_0 = [];
while ischar(traj_line)
   log_traj = textscan(traj_line,'%f %f %f %f');
   x = log_traj{1};
   y = log_traj{2};
   z = log_traj{3};
   t = log_traj{4};
   vtraj_0 = [ vtraj_0; [x,y,z,t] ];
   traj_line= fgetl(f_traj);
end
%find the one in vtraj_1 closest to vtraj_2's start
%[m,n]= size(vtraj_2);
% vend_2= vtraj_2(1,:);
% i_temp_2= 0;
% dis_t= 100000;
% for i=1:size(vtraj_1,1)
%     dis= sqrt( (vend_2(1)-vtraj_1(i,1))^2+ (vend_2(2)-vtraj_1(i,2))^2+ (vend_2(3)-vtraj_1(i,3))^2 );
%     if(dis< dis_t)
%        i_temp_2= i;
%        dis_t= dis;
%     end
% end
%%%%%%%%%%
vend_1= vtraj_1(1,:);
i_temp_1= 0;
dis_t= 100000;
for i=1:size(vtraj_0,1)
    dis= sqrt( (vend_1(1)-vtraj_0(i,1))^2+ (vend_1(2)-vtraj_0(i,2))^2+ (vend_1(3)-vtraj_0(i,3))^2 );
    if(dis< dis_t)
       i_temp_1= i;
       dis_t= dis;
    end
end
%uav
log_data =fopen('../../data/20140102-001846:path.txt','r');
if log_data == -1
     error('File log_data could not be opened, check name or path.')
end
log_line= fgetl(log_data);
reg = [];
if_log= 0;
pre_state= -1;

while ischar(log_line)
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
   
   if(pre_state~=4 && state== 4)
      if_log= 1;
      t0 = t;
   end
   
   deg2rad= pi/180.;
   vx_b= vx*cos(yaw*deg2rad)+ vy*sin(yaw*deg2rad);
   vy_b= -vx*sin(yaw*deg2rad)+ vy*cos(yaw*deg2rad);
   %vx= vx_b;
   %vy= vy_b;
   
   pre_state= state;
   
   if(if_log== 1 && (state== 3 || state== 7) )
   %if(if_log== 1 && state ~= 2 && state ~= 8 && state ~=0 && state~=6 && state~=4)
   %if(if_log== 1)
      t= t-t0;
      reg= [reg; [t,vx,vy,vz,vw,yaw,state,x,y,z] ]; 
   end
   
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%drawfigure;
hold on;
%axis auto;
grid on;
%axis([ 0, 15, -5, 5, 0, 10 ]);
%title('plot');
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
text( reg(1,8), reg(1,9), reg(1,10), 'start',...
	                 'VerticalAlignment','top',...
	                 'HorizontalAlignment','left',...
	                 'FontSize',14 );
[m,n]=size(reg);
text( reg(m,8), reg(m,9), reg(m,10), 'goal' ,...
	                 'VerticalAlignment','top',...
	                 'HorizontalAlignment','left',...
	                 'FontSize',14 );
plot3( vtraj_0(1:i_temp_1,1),vtraj_0(1:i_temp_1,2),vtraj_0(1:i_temp_1,3), 'r*' );
%plot3( vtraj_1(1:i_temp_2,1),vtraj_1(1:i_temp_2,2),vtraj_1(1:i_temp_2,3), 'r*' );
plot3( vtraj_1(:,1),vtraj_1(:,2),vtraj_1(:,3), 'r*' );
% plot3( vtraj_2(:,1),vtraj_2(:,2),vtraj_2(:,3), 'r*' );
%plot3( vtraj_0(1:177,1),vtraj_0(1:177,2),vtraj_0(1:177,3), 'r*' );
plot3( reg(:,8),reg(:,9),reg(:,10), 'k+' );

h= legend('plan','actual');
set(h,'FontSize',12);
view(3);
