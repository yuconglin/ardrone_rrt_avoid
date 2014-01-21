close all;
f_traj =fopen('../../bin/traj_rec.txt','r');
%f_traj= fopen('/home/yucong/.ros/virtual_replan_rec.txt','r');
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

figure;
hold on;
%axis auto;
grid on;
%axis([ 0, 15, -5, 5, 0, 10 ]);
%title('plot');
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');

plot3( virtual_traj(:,1),virtual_traj(:,2),virtual_traj(:,3), 'r*' );
text( virtual_traj(1,1),virtual_traj(1,2),virtual_traj(1,3), 'start',...
	                 'VerticalAlignment','top',...
	                 'HorizontalAlignment','left',...
	                 'FontSize',14 );
[m,n]= size(virtual_traj);
text( virtual_traj(m,1), virtual_traj(m,2), virtual_traj(m,3), 'goal' ,...
	                 'VerticalAlignment','top',...
	                 'HorizontalAlignment','left',...
	                 'FontSize',14 );
view(3);