function [ mat ] = cell2mat_casadi( cell )
mat = [];
for i=1:size(cell,1)
    mat_row = [];
    for j=1:size(cell,2)
        if isempty(cell{i,j})
            cell{i,j} = 0;
        end
        mat_row = [mat_row, cell{i,j}];
    end
    	mat = [mat; mat_row];
end
end

