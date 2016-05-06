function set_cpg_params(model_name, path_prefix, path_suffix, params)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    for i = 1:length(params)
        index = params{i}{1};
        path = strcat(model_name, path_prefix, num2str(index), path_suffix, '/');
        for b_name = keys(params{i}{2})
            block_name = b_name{1};
            block_path = strcat(path, block_name);
            set_param(block_path, 'Gain', num2str(params{i}{2}(block_name)));
        end
    end
end


