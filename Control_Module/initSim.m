function out = initSim(params)
    if ~evalin('base','eval(''var'',''setupSucceeded'')');
       disp('The setup.m script needs to be executed until the end first.');
       return
    end
    
    %TODO: set parameters for the model
    evalin('base', strcat('cpg_params{1}{2}(''RG_E_gain'') = ', num2str(params(1)), ';'));
    evalin('base', strcat('cpg_params{1}{2}(''RG_F_gain'') = ', num2str(params(1)), ';'));
    evalin('base', strcat('cpg_params{2}{2}(''RG_E_gain'') = ', num2str(params(2)), ';'));
    evalin('base', strcat('cpg_params{2}{2}(''RG_F_gain'') = ', num2str(params(2)), ';'));
    evalin('base', strcat('cpg_params{3}{2}(''RG_E_gain'') = ', num2str(params(3)), ';'));
    evalin('base', strcat('cpg_params{3}{2}(''RG_F_gain'') = ', num2str(params(3)), ';'));
    evalin('base', strcat('cpg_params{5}{2}(''RG_E_gain'') = ', num2str(params(4)), ';'));
    evalin('base', strcat('cpg_params{5}{2}(''RG_F_gain'') = ', num2str(params(4)), ';'));
%     evalin('base', strcat('set_param(strcat(model_name, ''/PF_MN_7/Shift''), ''Value'', ''', num2str(params(5)) ,''');'));
%     evalin('base', strcat('set_param(strcat(model_name, ''/PF_MN_13/Shift''), ''Value'', ''', num2str(params(5)) ,''');'));
%     evalin('base', strcat('set_param(strcat(model_name, ''/PF_MN_8/Shift''), ''Value'', ''', num2str(params(6)) ,''');'));
%     evalin('base', strcat('set_param(strcat(model_name, ''/PF_MN_14/Shift''), ''Value'', ''', num2str(params(6)) ,''');'));
%     evalin('base', strcat('set_param(strcat(model_name, ''/PF_MN_9/Shift''), ''Value'', ''', num2str(params(7)) ,''');'));
%     evalin('base', strcat('set_param(strcat(model_name, ''/PF_MN_15/Shift''), ''Value'', ''', num2str(params(7)) ,''');'));
    evalin('base', 'set_cpg_params(model_name, ''/PF_MN_'', '''', cpg_params)');
    
    evalin('base', 'set_param(strcat(model_name,''/PauseAfterStep''), ''Gain'',''1'')');
    
    % Disable sensors (subscribing)
%     evalin('base', 'set_param(strcat(model_name,''/EnableSensors''), ''Gain'', ''0'')');
    
    % Run Simulink
    evalin('base', 'set_param(model_name,''SimulationCommand'',''stop'')');
    evalin('base', 'set_param(model_name,''SimulationCommand'',''start'')');

    %wait until simulation is paused
    disp('Waiting for simulation to get paused...')
    while ~strcmp('paused', evalin('base', 'get_param(model_name, ''SimulationStatus'')'))
        pause(0.2)
        if strcmp(evalin('base', 'get_param(model_name, ''SimulationStatus'')'), 'stopped')
            disp('Error occured while waiting.')
            return
        end
    end
    evalin('base', 'set_param(strcat(model_name,''/PauseAfterStep''), ''Gain'',''0'')');
    disp('Simulation was paused.')
    
    command_pub = evalin('base','eval(''var'',''command_pub'')');
    msg = rosmessage(command_pub);
    msg.SetControlMode = 1;
    msg.ControlMode = 1;
    msg.SetBDImode = 1;
    msg.BDImode = 7;
    send(command_pub, msg);
    disp('Sent mode switch command.')    
    
    reset_pub = evalin('base','eval(''var'',''reset_pub'')');
    msg = rosmessage(reset_pub);
    % Extract message
    mux_rt = evalin('base', 'get_param(strcat(model_name, ''/Data Type Conversion''),''RuntimeObject'')');
    jp = mux_rt.OutputPort(1).Data;
    mux2_rt = evalin('base', 'get_param(strcat(model_name, ''/SendJointCommands/Bus Assignment''),''RuntimeObject'')');
    set_jp = mux2_rt.InputPort(4).Data;
    init_jp = evalin('base', 'init_joint_positions');
    
    for i = 1:length(set_jp)
        if set_jp(i) == 0
            jp(i) = init_jp(i);
        end
    end
    
    msg.SendJointCommand = 1;
    msg.Position = jp;
    msg.SetThisJoint = ones(1, 30);
    send(reset_pub, msg);
    disp('Sent reset command.')
    
    out = 1
end

