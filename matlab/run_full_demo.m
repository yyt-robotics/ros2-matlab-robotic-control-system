clc; clear; close all;

% Automatically add all subfolders
addpath(genpath(pwd));
addpath(genpath(fullfile(pwd, 'models')));
addpath(genpath(fullfile(pwd, 'utils')));
matlab_urdf_path = generate_matlab_urdf();
robot = importrobot(matlab_urdf_path);

%% ===== 1. Path settings =====
project_root = fileparts(mfilename('fullpath'));

external_csv_path = fullfile(getenv('HOME'), ...
    'ros2_project_data', 'matlab', 'trajectory', 'trajectory_log_6dof.csv');

repo_csv_path = fullfile(project_root, 'trajectory', 'trajectory_log_6dof.csv');

if isfile(external_csv_path)
    csv_path = external_csv_path;
    fprintf('Using external trajectory file: %s\n', csv_path);
else
    csv_path = repo_csv_path;
    fprintf('Using repository trajectory file: %s\n', csv_path);
end

model_name = 'pid_control';

%% ===== 2. Check if the CSV exists =====
if ~isfile(csv_path)
    error('Track file not found: %s', csv_path);
end

%% ===== 3. Read trajectory data =====
data = readtable(csv_path);
% Delete rows containing NaN or Inf in the time column or any other column.
valid_idx = isfinite(data.time) & ...
            isfinite(data.q1_ref) & ...
            isfinite(data.q2_ref) & ...
            isfinite(data.q3_ref) & ...
            isfinite(data.q4_ref) & ...
            isfinite(data.q5_ref) & ...
            isfinite(data.q6_ref);

data = data(valid_idx, :);
t = data.time;

q_ref_all = [ ...
    data.q1_ref, ...
    data.q2_ref, ...
    data.q3_ref, ...
    data.q4_ref, ...
    data.q5_ref, ...
    data.q6_ref ...
];

% Construct Simulink input
q_ref_signal = timeseries(q_ref_all, t);

q0_all = q_ref_all(1,:);
qd0_all = zeros(1,6);

q0_1 = q0_all(1); q0_2 = q0_all(2); q0_3 = q0_all(3);
q0_4 = q0_all(4); q0_5 = q0_all(5); q0_6 = q0_all(6);

qd0_1 = 0; qd0_2 = 0; qd0_3 = 0;
qd0_4 = 0; qd0_5 = 0; qd0_6 = 0;

%% ===== 4. Load and run the Simulink model =====
load_system(model_name);

% Stop time automatically retrieves the last moment of the trajectory
stop_time = num2str(t(end));
set_param(model_name, 'StopTime', stop_time);

out = sim(model_name);

%% ===== 5. Extract Simulation Output =====
disp('SimulationOutput contains:')
disp(out.who)

try
    q_actual_ts = out.q_actual_all;
catch
    error('If q_actual_all is not present in the Simulink output, please check the To Workspace module name or confirm that Single simulation output is enabled.');
end

if ~isa(q_actual_ts, 'timeseries')
    error('q_actual_all is not timeseries; please check the Save format in the To Workspace module.');
end

q_actual_data = q_actual_ts.Data;
q_actual_time = q_actual_ts.Time;

if isempty(q_actual_data) || isempty(q_actual_time)
    error('If q_actual_all is empty, please check if Simulink has actually output any data.');
end

% Interpolate to reference time point
q_actual_interp = interp1(q_actual_time, q_actual_data, t, 'linear', 'extrap');

%% ===== 6. Calculation Error =====
error_all = q_ref_all - q_actual_interp;

%% ===== 7. Draw a Tracking Chart =====
figure('Name', '6DOF Joint Tracking', 'NumberTitle', 'off');
for k = 1:6
    subplot(3,2,k);
    plot(t, q_ref_all(:,k), 'r--', 'LineWidth', 2); hold on;
    plot(t, q_actual_interp(:,k), 'b', 'LineWidth', 2);
    grid on;
    title(sprintf('Joint %d Tracking', k));
    xlabel('Time (s)');
    ylabel('Angle (rad)');
    legend(sprintf('q%d_{ref}', k), sprintf('q%d_{actual}', k), 'Location', 'best');
end

%% ===== 8. Draw an Error Graph =====
figure('Name', '6DOF Joint Error', 'NumberTitle', 'off');
for k = 1:6
    subplot(3,2,k);
    plot(t, error_all(:,k), 'LineWidth', 2);
    grid on;
    title(sprintf('Joint %d Error', k));
    xlabel('Time (s)');
    ylabel('Error (rad)');
end

%% ===== 9. Printing Error Statistics =====
fprintf('\n===== Error Summary =====\n');
for k = 1:6
    max_err = max(abs(error_all(:,k)));
    final_err = error_all(end,k);
    rmse = sqrt(mean(error_all(:,k).^2));

    fprintf('Joint %d:\n', k);
    fprintf('  Max Abs Error = %.6f rad\n', max_err);
    fprintf('  Final Error   = %.6f rad\n', final_err);
    fprintf('  RMSE          = %.6f rad\n\n', rmse);
end

%% ===== 10. Export Results to Workspace =====
assignin('base', 'data', data);
assignin('base', 't', t);
assignin('base', 'q_ref_all', q_ref_all);
assignin('base', 'q_ref_signal', q_ref_signal);
assignin('base', 'q_actual_interp', q_actual_interp);
assignin('base', 'error_all', error_all);
assignin('base', 'out', out);

disp('The execution of run_full_demo has been completed.');

