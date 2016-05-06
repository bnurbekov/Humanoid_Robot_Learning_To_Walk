function out = resumeSim()
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    evalin('base', 'set_param(model_name,''SimulationCommand'',''continue'')');
    disp('Continued Simulink sim.')
    out = 1
end

