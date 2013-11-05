close all;
%regular
reg_dubin =fopen('../../20131102-015141:3.0:1.4:0.8:dubin.txt','r');
%reg_dubin =fopen('../../20131102-015451:3.0:1.4:0.8:dubin.txt','r')
if reg_dubin == -1
     error('File dubins regular could not be opened, check name or path.')
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

%executed log
exe_dubin =fopen('../../20131102-015141:3.0:1.4:0.8:nav.txt','r');
%exe_dubin =fopen('../../20131102-015451:3.0:1.4:0.8:nav.txt','r');
if exe_dubin == -1
     error('File dubins execute could not be opened, check name or path.')
end
exe_line= fgetl(exe_dubin);
exe= [];
if_log= 0;
pre_state= -1;
while ischar(exe_line)
   %log_exe= textscan(exe_line,' -0.00401432 -0.0013125 0 -177.677 -0.00278545 -0.00133184 0.0630492 6'); 
   log_exe= textscan(exe_line,'%f %f %f %f %f %f %f %d'); 
   x= log_exe{1};
   y= log_exe{2};
   z= log_exe{3};
   state= log_exe{8};
   
   if(pre_state== 6 && state== 4)
      if_log= 1;
   end
   pre_state= state;
   
   if(if_log== 1 && state ~= 2 && state ~= 8)
      exe= [exe; [x,y,z] ]; 
   end
   exe_line= fgetl(exe_dubin);
end

%plot
figure;
hold on;
axis auto;
grid on;
zlim([0 0.9]);
%axis([ 0, 300, 0, 300, 0, 25 ]);
title('plot');
xlabel('Easting(km)');
ylabel('Northing(km)');
zlabel('Altitude(ft)');

plot3( reg(:,1),reg(:,2),reg(:,3), 'r*' );
plot3( exe(:,1),exe(:,2),exe(:,3), 'b*' );
%plot3( exe_ac(:,1),exe_ac(:,2),exe_ac(:,3), 'g*' );
view(3);
h= legend('regular','execute');
