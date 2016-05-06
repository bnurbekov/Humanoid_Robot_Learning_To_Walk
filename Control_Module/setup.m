clear all
%Share engine with Python
if ~any(matlab.engine.isEngineShared)
    matlab.engine.shareEngine
end
setupSucceeded = false;
%% 
%Setting paths to blocks for which parameters are tuned
global model_name
model_name = 'cpg_optimized';

%%
%Global CPG params

t_m = 0.1;
t_s = 2;
A_f = 0.1;

sigma_s = 1;    
sigma_f = 2.2;

%%
%Joint names to indeces mapping
joint_names = {'back_bkz', 'back_bky', 'back_bkx', 'neck_ry', 'l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky', 'l_leg_akx', 'r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', 'r_leg_akx', 'l_arm_shz', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_wry', 'l_arm_wrx', 'l_arm_wry2', 'r_arm_shz', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_wry', 'r_arm_wrx', 'r_arm_wry2'};
joint_indexes = 1:length(joint_names);
j_i_m = containers.Map(joint_names,joint_indexes);
joints_in_use = [j_i_m('l_leg_hpx') j_i_m('l_leg_hpy') j_i_m('l_leg_kny') j_i_m('l_leg_aky') j_i_m('l_leg_akx') j_i_m('l_arm_shx') j_i_m('r_leg_hpx') j_i_m('r_leg_hpy') j_i_m('r_leg_kny') j_i_m('r_leg_aky') j_i_m('r_leg_akx') j_i_m('r_arm_shx')];
joints_in_use_names = cellstr(['l_leg_hpx'; 'l_leg_hpy'; 'l_leg_kny'; 'l_leg_aky'; 'l_leg_akx'; 'l_arm_shx'; 'r_leg_hpx'; 'r_leg_hpy'; 'r_leg_kny'; 'r_leg_aky'; 'r_leg_akx'; 'r_arm_shx']); 
init_joint_positions = [-0.000173335472936742, 0.0034786623436957598, -0.0001993807527469471, -0.1106385663151741, -0.00854870118200779, 0.07581352442502975, -0.2, 0.45, -0.1, -0.07546507567167282, 0.008677197620272636, -0.07573944330215454, -0.2, 0.45, -0.1, 0.07665836066007614, -0.2996128499507904, -1.308706521987915, 1.8545480966567993, 0.48522788286209106, 0.02279164455831051, -0.005972625687718391, 0.0754905715584755, 0.2996732294559479, 1.3088403940200806, 1.8545966148376465, -0.4852358102798462, 0.023214785382151604, 0.006013402249664068, 0.07549051940441132];
%%
%Joint names to joint limits mapping
%Left leg
joint_limits = containers.Map();
joint_limits('l_leg_hpx') = [-0.52 0.52];
joint_limits('l_leg_hpy') = [-1.57 0.6];
joint_limits('l_leg_kny') = [0 2.35];
joint_limits('l_leg_aky') = [-0.9 0.48];
joint_limits('l_leg_akx') = [-0.43 0.43];
joint_limits('l_arm_shx') = [-1.57 1.57];

%Right leg
joint_limits('r_leg_hpx') = [-0.52 0.52];
joint_limits('r_leg_hpy') = [-1.57 0.6];
joint_limits('r_leg_kny') = [0 2.35];
joint_limits('r_leg_aky') = [-0.9 0.48];
joint_limits('r_leg_akx') = [-0.43 0.43];
joint_limits('r_arm_shx') = [-1.57 1.57];

%Tune ES_FS_params according to the joint limits
ES_FS_params_values = cell(1, length(joints_in_use_names));
for i = 1:length(joints_in_use_names)
    limits = joint_limits(char(joints_in_use_names(i)));
    ES_FS_params_values{i} = getSigmoidParams(limits);
end
ES_FS_params = containers.Map(joints_in_use, ES_FS_params_values);

%%
%Loading the Simulink model
load('NewBuses.mat')
if ~any(ismember(find_system('SearchDepth', 0),model_name))
    open_system(model_name)
end
block_list = find_system(model_name, 'BlockType', 'SubSystem');
%% 
%Setting up the parameters for the simulink model
for i = keys(ES_FS_params)
    index = i{1};
    path = strcat(model_name, '/ES_FS_', num2str(index));
    ES_A = afm(ES_FS_params, index, {1});
    ES_C = afm(ES_FS_params, index, {2});
    FS_A = -ES_A;
    FS_C = ES_C;
    
    %flip parameter A for knees
    if (index == j_i_m('r_leg_kny')) || (index == j_i_m('l_leg_kny'))      
        ES_A = -ES_A;
        FS_A = -FS_A;
    end

    pf_path = strcat(model_name, '/PF_MN_', num2str(index));
    set_param(strcat(pf_path, '/Shift'), 'Value', num2str(init_joint_positions(index)));
    
    set_param(strcat(path, '/ES'), 'a', num2str(ES_A));
    set_param(strcat(path, '/ES'), 'c', num2str(init_joint_positions(index)));
    
    set_param(strcat(path, '/FS'), 'a', num2str(FS_A));
    set_param(strcat(path, '/FS'), 'c', num2str(init_joint_positions(index)));
end

cpg_params = cell(1, length(joints_in_use));
for i = 1:length(joints_in_use)
    cpg_params{i}{1} = joints_in_use(i);
    cpg_params{i}{2} = containers.Map();
end

% 'l_leg_hpx'
cpg_params{1}{2}('ES_gain') = -0.01;
cpg_params{1}{2}('FS_gain') = -0.01;
cpg_params{1}{2}('RG_E_gain') = 1;
cpg_params{1}{2}('RG_F_gain') = 1;
% cpg_params{1}{2}('RG_E_gain') = 0;
% cpg_params{1}{2}('RG_F_gain') = 0;
cpg_params{1}{2}('M_gain') = 50;
% 'l_leg_hpy'
cpg_params{2}{2}('ES_gain') = -0.02;
cpg_params{2}{2}('FS_gain') = -0.02;
cpg_params{2}{2}('FB_E_gain') = 0.07;
cpg_params{2}{2}('FB_F_gain') = -0.07;
cpg_params{2}{2}('FF_E_gain') = -0.07;
cpg_params{2}{2}('FF_F_gain') = 0.07;
cpg_params{2}{2}('RG_E_gain') = 0.75;
cpg_params{2}{2}('RG_F_gain') = 0.75;
% cpg_params{2}{2}('RG_E_gain') = 0;
% cpg_params{2}{2}('RG_F_gain') = 0;
cpg_params{2}{2}('M_gain') = 50;
% 'l_leg_kny'
cpg_params{3}{2}('ES_gain') = -0.02;
cpg_params{3}{2}('FS_gain') = -0.02;
cpg_params{3}{2}('AS_E_gain') = 0.2;
cpg_params{3}{2}('AS_F_gain') = -0.2;
cpg_params{3}{2}('RG_E_gain') = 0.75;
cpg_params{3}{2}('RG_F_gain') = 0.75;
% cpg_params{3}{2}('RG_E_gain') = 0;
% cpg_params{3}{2}('RG_F_gain') = 0;
cpg_params{3}{2}('M_gain') = 50;
% 'l_leg_aky'
cpg_params{4}{2}('ES_gain') = -0.02;
cpg_params{4}{2}('FS_gain') = -0.02;
cpg_params{4}{2}('GB_E_gain') = -0.03;
cpg_params{4}{2}('GB_F_gain') = 0.03;
cpg_params{4}{2}('GF_E_gain') = 0.03;
cpg_params{4}{2}('GF_F_gain') = -0.03;
cpg_params{4}{2}('FB_E_gain') = -0.03;
cpg_params{4}{2}('FB_F_gain') = 0.03;
cpg_params{4}{2}('FF_E_gain') = 0.03;
cpg_params{4}{2}('FF_F_gain') = -0.03;
cpg_params{4}{2}('RG_E_gain') = 0.15;
cpg_params{4}{2}('RG_F_gain') = 0.15;
% cpg_params{4}{2}('RG_E_gain') = 0;
% cpg_params{4}{2}('RG_F_gain') = 0;
cpg_params{4}{2}('M_gain') = 50;
% 'l_leg_akx'
cpg_params{5}{2}('ES_gain') = -0.02;
cpg_params{5}{2}('FS_gain') = -0.02;
cpg_params{5}{2}('FB_E_gain') = -0.03;
cpg_params{5}{2}('FB_F_gain') = 0.03;
cpg_params{5}{2}('FF_E_gain') = 0.03;
cpg_params{5}{2}('FF_F_gain') = -0.03;
cpg_params{5}{2}('RG_E_gain') = 0.5;
cpg_params{5}{2}('RG_F_gain') = 0.5;
% cpg_params{5}{2}('RG_E_gain') = 0;
% cpg_params{5}{2}('RG_F_gain') = 0;
cpg_params{5}{2}('M_gain') = 50;
% 'l_arm_shz'
cpg_params{6}{2}('ES_gain') = -0.02;
cpg_params{6}{2}('FS_gain') = -0.02;
cpg_params{6}{2}('RG_E_gain') = 1;
cpg_params{6}{2}('RG_F_gain') = 1;
% cpg_params{6}{2}('RG_E_gain') = 0;
% cpg_params{6}{2}('RG_F_gain') = 0;
cpg_params{6}{2}('M_gain') = 10;
% 'r_leg_hpx'
cpg_params{7}{2} = cpg_params{1}{2};

% 'r_leg_hpy'
cpg_params{8}{2} = cpg_params{2}{2};

% 'r_leg_kny'
cpg_params{9}{2} = cpg_params{3}{2};

% 'r_leg_aky'
cpg_params{10}{2} = cpg_params{4}{2};

% 'r_leg_akx'
cpg_params{11}{2} = cpg_params{5}{2};

% 'r_arm_shz'
cpg_params{12}{2} = cpg_params{6}{2};

set_cpg_params(model_name, '/PF_MN_', '', cpg_params);

ICNotPresent = 0;
try
    load('ics.mat');
    ICS.set(model_name, ics);
catch
    disp('ics.mat was not found: starting RG neurons with 0 initial conditions (please allow some time for warm-up).')
    ICNotPresent = 1;
end 

% setenv('ROS_IP', '130.215.79.147')
% setenv('ROS_MASTER_URI','http://130.215.75.131:11311')
% setenv('ROS_IP', '169.254.167.86')
% setenv('ROS_MASTER_URI','http://169.254.167.87:11311')
%Configure the remote master
rosshutdown
rosinit
command_pub = rospublisher('/field_command', 'wrecs_msgs/field_command', 'IsLatching', false);
% reset_sub = rossubscriber('/reset_done');
reset_pub = rospublisher('/sim_reset', 'wrecs_msgs/field_command', 'IsLatching', false);
% perf_sub = rossubscriber('/sim_perf');
% runtrial_sub = rossubscriber('/runTrial', @runTrialCallback);


setupSucceeded = true;


