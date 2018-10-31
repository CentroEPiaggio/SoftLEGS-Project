% Run this file to modify the syntax of "mbs_sensor_walkman_robotran.m" in
% order that it can be used to generate symbolic expressions in CasADi
function converter_to_casadi_syntax (MBS_data,file_to_convert)

% open, copy, and close the sensor file
cd('..'); % move to main directory 
cfold = pwd; % get current path (this implies that this function is in the right path)
fid = fopen(strcat(cfold,'\symbolicR\',file_to_convert,'.m'),'r'); % move to symbolicR folder
cd('workR'); % move back to workR folder

%fid = fopen(strcat('/home/gian/Documents/MBProjects/Planar_Legs_Soft/symbolicR/',file_to_convert,'.m'),'r');
%fid = fopen(strcat('D:\Projects_Ubuntu\robotran2casadi/symbolicR/',file_to_convert,'.m'),'r');
f=fread(fid,'*char')';
fclose(fid);

file_to_convert_id = strsplit(file_to_convert,'_');
file_to_convert_id = file_to_convert_id(2);

if strcmp(file_to_convert_id,'sensor')
    % replacements for the file symbolicR/mbs_sensor_projectname
    f = strrep(f,'zeros','cell');
    f = strrep(f,'sens.P(','sens.P{');
    f = strrep(f,'sens.R(','sens.R{');
    f = strrep(f,'sens.V(','sens.V{');
    f = strrep(f,'sens.OM(','sens.OM{');
    f = strrep(f,'sens.J(','sens.J{');
    f = strrep(f,'sens.A(','sens.A{');
    f = strrep(f,'sens.OMP(','sens.OMP{');
    f = strrep(f,') = ','} = ');
    
elseif strcmp(file_to_convert_id,'dirdyna')
    % replacements for the file symbolicR/mbs_dirdyna_projectname
    f = strrep(f,'zeros','cell');
    f = strrep(f,' M(',' M{');
    f = strrep(f,' c(',' c{');
    f = strrep(f,') = ','} = ');
    
elseif strcmp(file_to_convert_id,'invdyna')
    % replacements for the file symbolicR/mbs_invdyna_projectname
    f = strrep(f,'zeros','cell');
    f = strrep(f,' Qq(',' Qq{');
    f = strrep(f,') = ','} = ');
    
 elseif strcmp(file_to_convert_id,'accelred')
    % replacements for the file symbolicR/mbs_accelred_projectname
    f = strrep(f,'zeros','cell');
    f = strrep(f,'s.qdd','cell(s.Njoint,1)');
    for i=1:MBS_data.Njoint
        qid = strcat('q(',num2str(i),')');
        qid_new = strcat('q{',num2str(i),'}');
        qdid = strcat('qd(',num2str(i),')');
        qdid_new = strcat('qd{',num2str(i),'}');
        qddid = strcat('qdd(',num2str(i),')');
        qddid_new = strcat('qdd{',num2str(i),'}');
        f = strrep(f,qid,qid_new);
        f = strrep(f,qdid,qdid_new);
        f = strrep(f,qddid,qddid_new);
        Qqid = strcat('Qq(',num2str(i),')');
        Qqid_new = strcat('Qq{',num2str(i),',1}');
        f = strrep(f,Qqid,Qqid_new);
    end
    for i=1:3 % ???
        uddid = strcat('udd(',num2str(i),')');
        uddid_new = strcat('udd{',num2str(i),'}');
        f = strrep(f,uddid,uddid_new);
    end
%     f = strrep(f,') = ','} = ');
    
elseif strcmp(file_to_convert_id,'extforces')
    % replacements for the file symbolicR/mbs_extforces_projectname
    f = strrep(f,'zeros','cell');
    f = strrep(f,' frc(',' frc{');
    f = strrep(f,' trq(',' trq{');
    f = strrep(f,') = ','} = ');
    f = strrep(f,' = usrfun.fext(',' = user_ExtForces(');
    f = strrep(f,'frc = s.frc;','');
%     MBS_data.Nxfrc
    for i=1:MBS_data.Nxfrc
        f = strrep(f,'trq = s.trq;',strcat('PxF',num2str(i),' = cell(3,1);  RxF',num2str(i),' = cell(3,3);  VxF',num2str(i),' = cell(3,1);  OMxF',num2str(i),' = cell(3,1);  AxF',num2str(i),' = cell(3,1);  OMPxF',num2str(i),' = cell(3,1);'));
        f = strrep(f,strcat('PxF',num2str(i),'('),strcat('PxF',num2str(i),'{'));
        f = strrep(f,strcat('RxF',num2str(i),'('),strcat('RxF',num2str(i),'{'));
        f = strrep(f,strcat('VxF',num2str(i),'('),strcat('VxF',num2str(i),'{'));
        f = strrep(f,strcat('OMxF',num2str(i),'('),strcat('OMxF',num2str(i),'{'));
        f = strrep(f,strcat('AxF',num2str(i),'('),strcat('AxF',num2str(i),'{'));
        f = strrep(f,strcat('OMPxF',num2str(i),'('),strcat('OMPxF',num2str(i),'{'));
    end
end

% write a new file conatining all the modifications
cd('..'); % move to main directory 
cfold = pwd; % get current path (this implies that this function is in the right path)
fid = fopen(strcat(cfold,'\workR\Functions\',file_to_convert,'.m'),'w'); % move to symbolicR folder
cd('workR'); % move back to workR folder

%fid  = fopen(strcat('/home/gian/Documents/MBProjects/Planar_Legs_Soft/workR/',file_to_convert,'_casadi.m'),'w');
%fid  = fopen(strcat('D:\Projects_Ubuntu\robotran2casadi/workR/',file_to_convert,'_casadi.m'),'w');
fprintf(fid,'%s',f);
fclose(fid);
end