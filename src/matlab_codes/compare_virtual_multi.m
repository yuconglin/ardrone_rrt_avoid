close all;
clear all;
exe_files=[
'20140102-001846:path.txt'
'20140102-163708:path.txt'
'20140102-163921:path.txt'
'20140102-202930:path.txt'
'20140102-203250:path.txt'
];
virtual_files={
      %'v1.0_the0.0.txt';
      'v-1.0_t7.0.txt';
      %'v4.0_the0.0.txt'
    };
%virtual_char= char(virtual_files);
min_dis= zeros( size(virtual_files,1), size(exe_files,1) );

for l=1:size(exe_files,1)
%for l=1:1   
    close all;
    %load the file
    file_path= sprintf('%s%s','../../data/',exe_files(l,:) );
    log_data =fopen(file_path,'r');
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

       if(pre_state~= 4 && state== 4)
          if_log= 1;
          t0=t;
       end

       pre_state= state;

       if(if_log== 1 && (state== 3 || state== 7) )
          t= t-t0;
          reg= [reg; [t,vx,vy,vz,vw,yaw,state,x,y,z] ]; 
       end

    end

    for j=1:size(min_dis,1)
        obs_rec= sprintf('%s%s','../../virtual_obs/',char(virtual_files(j) ) );
        log_data =fopen(obs_rec,'r');
        if log_data == -1
           error('File obs_rec.txt could not be opened, check name or path.' ); 
        end
        obs_line= fgetl(log_data);
        obs = [];
        while ischar(obs_line)
           log_obs = textscan(obs_line,'%f %f %f %f %f %f %f %f');
           t = log_obs{1};
           if t> 0
             if size(obs,1)== 0
                t0 = log_obs{1};
             end
             vx = log_obs{2};
             vy = log_obs{3};
             vz = log_obs{4};
             w = log_obs{5};
             x = log_obs{6};
             y = log_obs{7};
             z = log_obs{8};
             obs = [ obs; [t-t0,vx,vy,vz,w,x,y,z] ];
           end%if ends
           obs_line= fgetl(log_data);
        end

        obs_match = [];
        dis_rec = [];
        %quad and obstacle time matching
        idx = 1;
        for i=1:size(reg,1)
            t= reg(i,1);

            if( t< obs(1,1) )
              obs_match = [obs_match; obs(1,:)];
            elseif( t> obs( size(obs,1),1) )
              obs_match = [obs_match; obs(size(obs,1),:) ];
            else
              while(1)
                 if (idx+1<= size(obs,1) && obs(idx,1)<=t && t<= obs(idx+1,1)) || (idx+1 == size(obs,1) )
                    break; 
                 else
                   idx = idx +1;
                 end
              end %while ends
              %idx,t
              obs_match= [obs_match; obs(idx,:)];
            end

            dis = sqrt( (obs(idx,6)-reg(i,8))^2+(obs(idx,7)-reg(i,9))^2 +(obs(idx,8)-reg(i,10))^2 );
            dis_rec = [dis_rec; dis];
        end
        min_dis(j,l)= min(dis_rec);
    end
    
end
min_tran= min_dis';