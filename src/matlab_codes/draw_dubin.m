close all;
%regular
<<<<<<< HEAD
%reg_dubin =fopen('../../data/../../data/131102-015141:3.0:1.4:0.8:dubin.txt','r');
%reg_dubin =fopen('../../data//20131104-181815:2.5:2.5:1.3:dubin.txt','r');
%reg_dubin =fopen('../../data//20131102-015451:3.0:1.4:0.8:dubin.txt','r');
%reg_dubin =fopen('../../data//20131104-163711:3.0:3.0:0.8:dubin.txt','r');
%reg_dubin =fopen('../../data//20131104-164045:2.0:2.0:0.8:dubin.txt','r');
%reg_dubin =fopen('../../data//20131104-165247:2.0:2.0:0.8:dubin.txt','r');
%reg_dubin =fopen('../../data//20131104-165407:2.0:2.0:0.8:dubin.txt','r');
%reg_dubin =fopen('../../data//20131104-172749:2.0:2.0:0.8:dubin.txt','r');
%reg_dubin =fopen('../../data//20131104-173357:2.0:2.0:0.8:dubin.txt','r');
%reg_dubin =fopen('../../data//20131104-173748:2.0:2.0:0.8:dubin.txt','r');
%reg_dubin =fopen('../../data//20131104-174126:2.0:2.0:0.8:dubin.txt','r');
%reg_dubin =fopen('../../data//20131104-174309:2.0:2.0:1.3:dubin.txt','r');
%reg_dubin =fopen('../../data//20131104-174438:2.0:2.0:1.3:dubin.txt','r');
%reg_dubin =fopen('../../data//20131104-174709:2.0:2.0:1.3:dubin.txt','r');
%reg_dubin =fopen('../../data//20131104-175529:2.0:2.0:1.3:dubin.txt','r');
%reg_dubin =fopen('../../data//20131104-175720:2.5:2.5:1.3:dubin.txt','r');
%reg_dubin =fopen('../../data//20131104-175823:2.5:2.5:1.3:dubin.txt','r');
%reg_dubin =fopen('../../data//20131104-175926:2.5:2.5:1.3:dubin.txt','r');
%reg_dubin =fopen('../../data//20131104-180046:2.5:2.5:1.3:dubin.txt','r');
=======
%reg_dubin =fopen('../../20131102-015141:3.0:1.4:0.8:dubin.txt','r');
%reg_dubin =fopen('../../20131102-015451:3.0:1.4:0.8:dubin.txt','r')
%reg_dubin =fopen('../data../data131102-015141:3.0:1.4:0.8:dubin.txt','r');
%reg_dubin =fopen('../data/20131104-181815:2.5:2.5:1.3:dubin.txt','r');
%reg_dubin =fopen('../data/20131102-015451:3.0:1.4:0.8:dubin.txt','r');
%reg_dubin =fopen('../data/20131104-163711:3.0:3.0:0.8:dubin.txt','r');
%reg_dubin =fopen('../data/20131104-164045:2.0:2.0:0.8:dubin.txt','r');
%reg_dubin =fopen('../data/20131104-165247:2.0:2.0:0.8:dubin.txt','r');
%reg_dubin =fopen('../data/20131104-165407:2.0:2.0:0.8:dubin.txt','r');
%reg_dubin =fopen('../data/20131104-172749:2.0:2.0:0.8:dubin.txt','r');
%reg_dubin =fopen('../data/20131104-173357:2.0:2.0:0.8:dubin.txt','r');
%reg_dubin =fopen('../data/20131104-173748:2.0:2.0:0.8:dubin.txt','r');
%reg_dubin =fopen('../data/20131104-174126:2.0:2.0:0.8:dubin.txt','r');
%reg_dubin =fopen('../data/20131104-174309:2.0:2.0:1.3:dubin.txt','r');
%reg_dubin =fopen('../data/20131104-174438:2.0:2.0:1.3:dubin.txt','r');
reg_dubin =fopen('../data/20131104-174709:2.0:2.0:1.3:dubin.txt','r');
%reg_dubin =fopen('../data/20131104-175529:2.0:2.0:1.3:dubin.txt','r');
%reg_dubin =fopen('../data/20131104-175720:2.5:2.5:1.3:dubin.txt','r');
%reg_dubin =fopen('../data/20131104-175823:2.5:2.5:1.3:dubin.txt','r');
%reg_dubin =fopen('../data/20131104-175926:2.5:2.5:1.3:dubin.txt','r');
%reg_dubin =fopen('../data/20131104-180046:2.5:2.5:1.3:dubin.txt','r');
>>>>>>> 22cf9f94a676a9eccb44873363f27076b5a5f8d3

%reg_dubin =fopen('../../data//20131104-180336:2.5:0.0:1.3:dubin.txt','r');%straight line
%reg_dubin =fopen('../../data//20131104-180457:2.5:0.0:1.3:dubin.txt','r');%straight line
%reg_dubin =fopen('../../data//20131104-181320:1.0:1.0:1.3:dubin.txt','r'); %quater circle
reg_dubin =fopen('../../data//20131104-181208:1.0:1.0:1.3:dubin.txt','r');%quater circle
%reg_dubin =fopen('../../data//20131104-181055:1.0:1.0:1.3:dubin.txt','r');%quater circle
%reg_dubin =fopen('../../data//20131104-180706:2.5:0.0:1.3:dubin.txt','r');%straight line
%reg_dubin =fopen('../../data//20131104-180541:2.5:0.0:1.3:dubin.txt','r');%straight line

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
<<<<<<< HEAD
%exe_dubin =fopen('../../data//20131102-015141:3.0:1.4:0.8:nav.txt','r');
%exe_dubin =fopen('../../data//20131104-181815:2.5:2.5:1.3:nav.txt','r');
%exe_dubin =fopen('../../data//20131102-015451:3.0:1.4:0.8:nav.txt','r');
%exe_dubin =fopen('../../data//20131104-163711:3.0:3.0:0.8:nav.txt','r');
%exe_dubin =fopen('../../data//20131104-164045:2.0:2.0:0.8:nav.txt','r');
%exe_dubin =fopen('../../data//20131104-165247:2.0:2.0:0.8:nav.txt','r');
%exe_dubin =fopen('../../data//20131104-165407:2.0:2.0:0.8:nav.txt','r');
%exe_dubin =fopen('../../data//20131104-172749:2.0:2.0:0.8:nav.txt','r');
%exe_dubin =fopen('../../data//20131104-173357:2.0:2.0:0.8:nav.txt','r');
%exe_dubin =fopen('../../data//20131104-173748:2.0:2.0:0.8:nav.txt','r');
%exe_dubin =fopen('../../data//20131104-174126:2.0:2.0:0.8:nav.txt','r');
%exe_dubin =fopen('../../data//20131104-174309:2.0:2.0:1.3:nav.txt','r');
%exe_dubin =fopen('../../data//20131104-174438:2.0:2.0:1.3:nav.txt','r');
%exe_dubin =fopen('../../data//20131104-174709:2.0:2.0:1.3:nav.txt','r');
%exe_dubin =fopen('../../data//20131104-175529:2.0:2.0:1.3:nav.txt','r');
%exe_dubin =fopen('../../data//20131104-175720:2.5:2.5:1.3:nav.txt','r');
%exe_dubin =fopen('../../data//20131104-175823:2.5:2.5:1.3:nav.txt','r');
%exe_dubin =fopen('../../data//20131104-175926:2.5:2.5:1.3:nav.txt','r');
%exe_dubin =fopen('../../data//20131104-180046:2.5:2.5:1.3:nav.txt','r');

%exe_dubin =fopen('../../data//20131104-180336:2.5:0.0:1.3:nav.txt','r');%straight line
%exe_dubin =fopen('../../data//20131104-180457:2.5:0.0:1.3:nav.txt','r');%straight line
%exe_dubin =fopen('../../data//20131104-181320:1.0:1.0:1.3:nav.txt','r'); %quater circle
exe_dubin =fopen('../../data//20131104-181208:1.0:1.0:1.3:nav.txt','r');%quater circle
%exe_dubin =fopen('../../data//20131104-181055:1.0:1.0:1.3:nav.txt','r');%quater circle
%exe_dubin =fopen('../../data//20131104-180706:2.5:0.0:1.3:nav.txt','r');%straight line
%exe_dubin =fopen('../../data//20131104-180541:2.5:0.0:1.3:nav.txt','r');%straight line
=======

%exe_dubin =fopen('../../20131102-015141:3.0:1.4:0.8:nav.txt','r');
%exe_dubin =fopen('../../20131102-015451:3.0:1.4:0.8:nav.txt','r');
%exe_dubin =fopen('../data/20131102-015141:3.0:1.4:0.8:nav.txt','r');
%exe_dubin =fopen('../data/20131104-181815:2.5:2.5:1.3:nav.txt','r');
%exe_dubin =fopen('../data/20131102-015451:3.0:1.4:0.8:nav.txt','r');
%exe_dubin =fopen('../data/20131104-163711:3.0:3.0:0.8:nav.txt','r');
%exe_dubin =fopen('../data/20131104-164045:2.0:2.0:0.8:nav.txt','r');
%exe_dubin =fopen('../data/20131104-165247:2.0:2.0:0.8:nav.txt','r');
%exe_dubin =fopen('../data/20131104-165407:2.0:2.0:0.8:nav.txt','r');
%exe_dubin =fopen('../data/20131104-172749:2.0:2.0:0.8:nav.txt','r');
%exe_dubin =fopen('../data/20131104-173357:2.0:2.0:0.8:nav.txt','r');
%exe_dubin =fopen('../data/20131104-173748:2.0:2.0:0.8:nav.txt','r');
%exe_dubin =fopen('../data/20131104-174126:2.0:2.0:0.8:nav.txt','r');
%exe_dubin =fopen('../data/20131104-174309:2.0:2.0:1.3:nav.txt','r');
%exe_dubin =fopen('../data/20131104-174438:2.0:2.0:1.3:nav.txt','r');
exe_dubin =fopen('../data/20131104-174709:2.0:2.0:1.3:nav.txt','r');
%exe_dubin =fopen('../data/20131104-175529:2.0:2.0:1.3:nav.txt','r');
%exe_dubin =fopen('../data/20131104-175720:2.5:2.5:1.3:nav.txt','r');
%exe_dubin =fopen('../data/20131104-175823:2.5:2.5:1.3:nav.txt','r');
%exe_dubin =fopen('../data/20131104-175926:2.5:2.5:1.3:nav.txt','r');
%exe_dubin =fopen('../data/20131104-180046:2.5:2.5:1.3:nav.txt','r');

%exe_dubin =fopen('../data/20131104-180336:2.5:0.0:1.3:nav.txt','r');%straight line
%exe_dubin =fopen('../data/20131104-180457:2.5:0.0:1.3:nav.txt','r');%straight line
%exe_dubin =fopen('../data/20131104-181320:1.0:1.0:1.3:nav.txt','r'); %quater circle
%exe_dubin =fopen('../data/20131104-181208:1.0:1.0:1.3:nav.txt','r');%quater circle
%exe_dubin =fopen('../data/20131104-181055:1.0:1.0:1.3:nav.txt','r');%quater circle
%exe_dubin =fopen('../data/20131104-180706:2.5:0.0:1.3:nav.txt','r');%straight line
%exe_dubin =fopen('../data/20131104-180541:2.5:0.0:1.3:nav.txt','r');%straight line

>>>>>>> 22cf9f94a676a9eccb44873363f27076b5a5f8d3
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
   
   if(if_log== 1 && state ~= 2 && state ~= 8 && state ~=0 && state ~=7 && state~=6 && state~=4)
      exe= [exe; [x,y,z] ]; 
   end
   exe_line= fgetl(exe_dubin);
end

%plot
figure;
hold on;
axis auto;
grid on;
zlim([0 4]);
%ylim([-1 1]);
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
