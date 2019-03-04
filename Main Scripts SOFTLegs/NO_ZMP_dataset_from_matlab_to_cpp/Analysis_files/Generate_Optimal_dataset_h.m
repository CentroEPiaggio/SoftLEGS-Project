function []=Generate_Optimal_dataset_h(VIS_SIMS,file_name,toSKip)
% This function returns the needed header file to accomplish the optimal
% locomotion by using the SoftLeg gui.

flag_init=0;
flag_finish=0;
idx=0;

    for n_traj=1:toSKip:size(VIS_SIMS,2)

        if n_traj==1
            flag_init=1;
        end

        New_time=VIS_SIMS{8,n_traj};
        Ts=diff(New_time(1:2));
        New_TH=VIS_SIMS{7,n_traj}';
        New_PQ=VIS_SIMS{6,n_traj}';

    if flag_init
       fileID=fopen(file_name,'w');

       str_n=['int n_traj = ',num2str(ceil(size(VIS_SIMS,2)/toSKip)),';'];
       fprintf(fileID,str_n);
       fprintf(fileID,'\n'); 

       for k=1:toSKip:size(VIS_SIMS,2)
       str_i=['Eigen::MatrixXd traj_',num2str(k),'(',num2str(size(VIS_SIMS{7,k},2)),',',num2str(size(VIS_SIMS{7,k},1)),');'];

       fprintf(fileID,str_i);
       fprintf(fileID,'\n'); 
       end

       str_pt=['std::vector<Eigen::MatrixXd*> traj_M(n_traj);\n'];
       fprintf(fileID,str_pt);
       fprintf(fileID,'\n'); 

       str_name=['std::vector<std::string> sim_name(n_traj);\n'];
       fprintf(fileID,str_name);

       fprintf(fileID,'\n'); 


       fprintf(fileID,['double Ts_traj = ',num2str(Ts),';']);
       fprintf(fileID,'\n\n'); 

       str_init=['int Initialize_Optimal_Dataset(){'];
       fprintf(fileID,str_init);
       fprintf(fileID,'\n'); 
       flag_init=0;
    else
        fileID=fopen(file_name,'a');
        fprintf(fileID,'\n\n\n'); 
    end      


    d = strsplit(VIS_SIMS{1,n_traj}(1:end - 1),'_St_');

    str_name_init=['sim_name[',num2str(idx),'] = "',d{2},'"',';'];
       fprintf(fileID,str_name_init);
       fprintf(fileID,'\n'); 
    str_pt=['traj_M.at(',num2str(idx),')=&traj_',num2str(n_traj),';'];
       fprintf(fileID,str_pt);
       fprintf(fileID,'\n\n'); 

    THr=size(New_TH,1);
    THc=size(New_TH,2);

        for ii=1:THr

            for jj=1:THc
                str=['traj_',num2str(n_traj),'(',num2str(ii-1),',',num2str(jj-1),') = ',num2str(New_TH(ii,jj)),';\n'];
                fprintf(fileID,str);
            end
            fprintf(fileID,'\n\n'); 
        end

        idx = idx +1;
    end

str_close=['return 1;}'];
fprintf(fileID,str_close);
fprintf(fileID,'\n');  


fclose(fileID);
end

