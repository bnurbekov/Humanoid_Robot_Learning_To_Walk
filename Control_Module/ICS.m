classdef ICS
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
   
    methods(Static)
        
    function ics = get(model_name, indices)
        %Check for right status of the model

        if ~any(ismember(find_system('SearchDepth', 0),model_name))
            error('Model must be opened to extract the Initial Conditions.')
        end

        sim_status = get_param(model_name, 'SimulationStatus');

        if ~strcmp(sim_status, 'paused')
            error('Model must be paused to extract the Initial Conditions.')
        end

        ics = containers.Map('KeyType','double', 'ValueType', 'any');

        for ind = indices
            temp = cell(2, 2);
            for i = 1:2
                for j = 1:2
                    cell_path = strcat(model_name, '/cell_', num2str(ind), '_', num2str(i), '/DTI_', num2str(j));
                    rto = get_param(cell_path,'RuntimeObject');
                    temp{i, j} = rto.OutputPort(1).Data;
                end
            end
            ics(ind) = temp;
        end
        
        %Store AS params at indices 1000+
%         for i = 1:2
%             AS_path = strcat(model_name, '/AS',num2str(i),'/Detect Decrease');
%             ics(1000+i) = Simulink.Mask.get(AS_path).Parameters.Value;
%         end
    end
    
    function set(model_name, ics)
        ks = keys(ics);
        for key = ks
            k = key{1};
            for i = 1:2
                for j = 1:2
                    value = ics(k);
                    cell_path = strcat(model_name, '/cell_', num2str(k), '_', num2str(i), '/DTI_', num2str(j));
                    set_param(cell_path,'InitialCondition', num2str(value{i, j}));
                end
            end
        end
    end    
    end
    
end

