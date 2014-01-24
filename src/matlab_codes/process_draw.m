close all;
clear all;

%%%%%branch log%%%%%%%%%%%%%%%
f_branch =fopen('../../bin/branch_log.txt','r');
if f_branch == -1
    error('File branch_log.txt could not be opened, check name or path.')
end
line= fgetl(f_branch);
branch_log = [];
while ischar(line)
   log_traj = textscan(line,'%f %f %f');
   x = log_traj{1};
   y = log_traj{2};
   z = log_traj{3};
   branch_log = [ branch_log; [x,y,z] ];
   line= fgetl(f_branch);
end
%%%%%%%%%%%%%%%%branch sample%%%%%%%%%%%%%%%%%%%%%%%%
f_sample_dubin=fopen('../../bin/sample_dubin.txt','r');
if f_sample_dubin == -1
    error('File sample_dubin.txt could not be opened, check name or path.')
end
line=fgetl(f_sample_dubin);
sample_dubin_log = [];
while ischar(line)
    log_sd= textscan(line,'%f %f %f');
    x = log_sd{1};
    y = log_sd{2};
    z = log_sd{3};
    sample_dubin_log= [sample_dubin_log; [x,y,z] ];
    line= fgetl(f_sample_dubin);
end
%%%%%%%%%%%%%%%%%%%nodes%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f_nodes=fopen('../../bin/node_log.txt','r');
if f_nodes== -1
   error('File node_log.txt could not be opened, check name or path.');
end
line=fgetl(f_nodes);
node_log = [];
while ischar(line)
    log_nd= textscan(line,'%f %f %f');
    x = log_nd{1};
    y = log_nd{2};
    z = log_nd{3};
    node_log= [node_log; [x,y,z] ];
    line= fgetl(f_nodes);
end
%%%%%%%%%%%%%%%%%%path file%%%%%%%%%%%%%%%%%%%%%%%%%%%
f_path= fopen('../../bin/traj_rec.txt','r');
if f_path == -1
    error('File traj_rec.txt could not be opened, check name or path.')
end

traj_line= fgetl(f_path);
virtual_traj = [];
while ischar(traj_line)
   log_traj = textscan(traj_line,'%f %f %f %f');
   x = log_traj{1};
   y = log_traj{2};
   z = log_traj{3};
   t = log_traj{4};
   virtual_traj = [ virtual_traj; [x,y,z,t] ];
   traj_line= fgetl(f_path);
end
%%%%%%%%%%%%%%%%%%%%%%%%%draw dubin branch%%%%%%%%%%%%%%%%%%%%%%
figure;
hold on;
%axis auto;
grid on;
%axis([ 0, 15, -5, 5, 0, 10 ]);
%title('plot');
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');

text( 0, 0, 0.8, 'start',...
	                 'VerticalAlignment','top',...
	                 'HorizontalAlignment','left',...
	                 'FontSize',14 );
text( 14, 0.6, 2.0, 'goal' ,...
	                 'VerticalAlignment','top',...
	                 'HorizontalAlignment','left',...
	                 'FontSize',14 );
               
plot3( branch_log(:,1),branch_log(:,2),branch_log(:,3), 'g*' );
plot3( sample_dubin_log(:,1),sample_dubin_log(:,2),sample_dubin_log(:,3), 'k*','MarkerSize',10 );  
plot3( node_log(:,1),node_log(:,2),node_log(:,3), 'b+','MarkerSize', 8 );
plot3( virtual_traj(:,1),virtual_traj(:,2),virtual_traj(:,3), 'r*' );

h= legend('branch','sample','node','path');
set(h,'FontSize',12);

view(3);