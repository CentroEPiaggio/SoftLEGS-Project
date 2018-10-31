function start_sim = check2restart(nameDir)
% This script is used to retrieve the number of data already valuated
% during the running of the optimization problem and then to change the
% starting index

% Initialiation
numData = 1;
% Folder data
set = dir(nameDir);
mkdir(nameDir);

% User display
disp(['Do you really want to save in the following existent folder?? Y(y)/N(n)',10]);
disp(['---->  ',nameDir,10]);
s1 = input('Your choose: ','s');

if (strcmp(s1,'Y') || strcmp(s1,'y'))                                       % Restart 
    % User display
    disp(['Do you want to start a NEW collection?? Y(y)/N(n)',10]);    
    s2 = input('Your choose: ','s'); 
    if (strcmp(s2,'Y') || strcmp(s2,'y'))                                   % Start from zero    
        start_sim = 1;
    else                                                                    % Restart from the last data
    	% Set of simulations to test [last:ALL]
        for i_n=1:(size(set,1) - 2) % remove current and previous dir (.\) and (..\)      
            numData = numData + 1;
        end
        start_sim = numData;
    end
else                                                                        % Return [SAFE mode]
    disp([10,'Doh!',10']);
    start_sim = -1;
    return
end

% User display
disp([10,'RESTARTING OPTIMIZATION FROM: ',num2str(numData),10]);

end

