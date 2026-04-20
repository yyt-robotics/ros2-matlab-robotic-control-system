function rebuild_simscape_robot(robot, sm_model_name)
% REBUILD_SIMSCAPE_ROBOT Rebuild a Simscape robot subsystem from URDF
% Usage:
%   rebuild_simscape_robot()                        % 使用默认 URDF 和模型名
%   rebuild_simscape_robot(robot)                   % 使用指定 robot 对象，默认模型名
%   rebuild_simscape_robot(robot, 'my_model_name') % 使用指定 robot 对象和模型名

%% ===== 1. 默认参数处理 =====
if nargin < 1 || isempty(robot)
    % 自动生成 robot
    fprintf('No robot input provided, importing default URDF...\n');
    urdf_path = generate_matlab_urdf();
    robot = importrobot(urdf_path);
end

if nargin < 2 || isempty(sm_model_name)
    sm_model_name = 'gluon_6l3_rebuilt';
end

%% ===== 2. 如果模型已存在，先关闭 =====
if bdIsLoaded(sm_model_name)
    fprintf('Model "%s" already loaded. Closing it first...\n', sm_model_name);
    close_system(sm_model_name, 0); % 0 表示不保存
end

%% ===== 3. 导入 Simscape 模型 =====
fprintf('Importing robot to Simscape model: %s\n', sm_model_name);
Gluon_SM = smimport(robot, 'ModelName', sm_model_name);
fprintf('Simscape model created: %s.slx\n', sm_model_name);

%% ===== 4. 配置 Revolute Joint 参数 =====
joint_blocks = find_system(sm_model_name, ...
    'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', ...
    'MaskType', 'Revolute Joint');

fprintf('Found %d Revolute Joint blocks.\n', numel(joint_blocks));

for k = 1:numel(joint_blocks)
    joint_name = get_param(joint_blocks{k}, 'Name');
    fprintf('Configuring joint: %s\n', joint_name);

    % 设置默认参数
    set_param(joint_blocks{k}, 'MotionActuationMode', 'InputMotion');
    set_param(joint_blocks{k}, 'TorqueActuationMode', 'ComputedTorque');
    set_param(joint_blocks{k}, 'SensePosition', 'on');
end

%% ===== 5. 保存模型到当前目录 =====
% 保存路径指定到 models/ 目录
slx_path = fullfile(pwd, 'models', [sm_model_name '.slx']);
save_system(sm_model_name, slx_path);
fprintf('Simscape model saved to: %s\n', slx_path);
end