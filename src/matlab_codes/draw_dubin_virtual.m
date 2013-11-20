close all;
%regular
reg_dubin =fopen('../../bin/dubins_record.txt','r');
if reg_dubin == -1
     error('File dubins_record.txt could not be opened, check name or path.')
end
reg_line= fgetl(reg_dubin);
reg = [];
while ischar(reg_line)
   log_reg = textscan(reg_line,'%f %f %f %f');
   x = log_reg{1};
   y = log_reg{2};
   z = log_reg{3};
   t = log_reg{4};
   reg = [ reg; [x,y,z,t] ];
   reg_line= fgetl(reg_dubin);
end
%virtual executed
exe_dubin =fopen('../../bin/dubin_log.txt','r');
if exe_dubin == -1
     error('File dubins_record.txt could not be opened, check name or path.')
end
exe_line= fgetl(exe_dubin);
exe = [];
while ischar(exe_line)
   log_exe = textscan(exe_line,'%f %f %f %f');
   x = log_exe{1};
   y = log_exe{2};
   z = log_exe{3};
   t = log_exe{4};
   exe = [ exe; [x,y,z,t] ];
   exe_line= fgetl(exe_dubin);
end
%actual executed
% exe_ros =fopen('../../build/exe_rec.txt','r');
% if exe_ros == -1
%      error('File exe_rec.txt could not be opened, check name or path.')
% end
% exe_line= fgetl(exe_ros);
% exe_ac = [];
% while ischar(exe_line)
%    log_exe = textscan(exe_line,'%f %f %f %f');
%    x = log_exe{1};
%    y = log_exe{2};
%    z = log_exe{3};
%    the = log_exe{4};
%    exe_ac = [ exe_ac; [x,y,z,the] ];
%    exe_line= fgetl(exe_ros);
% end

%drawing start
%plot
figure;
hold on;
axis auto;
grid on;
%axis([ 0, 300, 0, 300, 0, 25 ]);
title('plot');
xlabel('Easting(km)');
ylabel('Northing(km)');
zlabel('Altitude(ft)');

plot3( reg(:,1),reg(:,2),reg(:,3), 'r*' );
plot3( exe(:,1),exe(:,2),exe(:,3), 'b*' );
%plot3( exe_ac(:,1),exe_ac(:,2),exe_ac(:,3), 'g*' );
view(3);
h= legend('regular','predicted');
