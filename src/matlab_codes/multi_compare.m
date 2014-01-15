close all;
clear all;
exe_files=[
'20140101-024549:path.txt'
'20140101-024911:path.txt'
'20140101-025957:path.txt'
'20140101-030305:path.txt'
'20140101-031019:path.txt'
'20140101-031737:path.txt'
'20140101-032724:path.txt'
'20140101-032958:path.txt'
'20140101-033830:path.txt'
'20140101-035920:path.txt'
'20140101-040209:path.txt'
'20140101-041034:path.txt'
'20140102-051319:path.txt'
'20140102-204954:path.txt'
'20140102-213104:path.txt'
'20140102-214058:path.txt'
'20140102-214842:path.txt'
];

other_files=[
'20140101-024525:other.txt'
'20140101-024850:other.txt'
'20140101-025946:other.txt'
'20140101-030246:other.txt'
'20140101-031009:other.txt'
'20140101-031731:other.txt'
'20140101-032715:other.txt'
'20140101-032946:other.txt'
'20140101-033823:other.txt'
'20140101-035912:other.txt'
'20140101-040159:other.txt'
'20140101-041017:other.txt'
'20140102-051309:other.txt'
'20140102-204944:other.txt'
'20140102-213053:other.txt'
'20140102-214048:other.txt'
'20140102-214832:other.txt'   
];

min_dis= zeros( size(exe_files,1), 1);
for j=1:size(exe_files,1)
    %%%%%%%%%path
    file_path= sprintf('%s%s','../../data/',exe_files(j,:));
    log_data =fopen(file_path,'r');

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
    %%%%%%%%%%other
    file_other= sprintf('%s%s','../../data/',other_files(j,:) );
    log_data =fopen(file_other,'r');
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
    min_dis(j)= min(dis_rec);
end


