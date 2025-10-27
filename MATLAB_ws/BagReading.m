close all
clear
clc

%%

%nodeMATLAB = ros2node("nodeMATLAB", 0);

folderPath = fullfile(pwd, "custom");
ros2genmsg(folderPath)

%% Check for msg

% ros2 msg show custom_interfaces/SyncData

%%

folderPathBag = fullfile(pwd, "ROS2Bags/rosbag_20251024_185001_144916"); 
bagReader = ros2bagreader(folderPathBag);
% bagReader.AvailableTopics

% bagInfo = ros2('bag', 'info', folderPathBag);

%% Get all messages

messageAll = readMessages(bagReader);

%% Extract Data and merge into one single variable

% Extract the timestamp from the messages
Timestamps = cell2mat(cellfun(@(msg) double(msg.stamp.sec) + double(msg.stamp.nanosec) * 1e-9, ...
                    messageAll, 'UniformOutput', false));
Timestamps = Timestamps - Timestamps(1); % first value as baseline

% Extract the measurements
Mediciones = double(cell2mat(cellfun(@(msg) msg.measurements.data,...
                    messageAll, 'UniformOutput', false)));

% Extract the end-effector position in x
Poses_x = double(cell2mat(cellfun(@(msg) msg.ee_pose.pose.position.x,...
                 messageAll, 'UniformOutput', false)));

% Extract the end-effector position in y
Poses_y = double(cell2mat(cellfun(@(msg) msg.ee_pose.pose.position.y,...
                 messageAll, 'UniformOutput', false)));

% Extract the end-effector position in z
Poses_z = double(cell2mat(cellfun(@(msg) msg.ee_pose.pose.position.z,...
                 messageAll, 'UniformOutput', false)));

% Combine the extracted data into a single matrix
Data = [Timestamps, Poses_x, Poses_y, Poses_z, Mediciones];

%% Create table of combinedData
% combinedTable = array2table(Data, 'VariableNames', {'Time', 'PosX', 'PosY', 'PosZ', 'Measurements'});

%% Plot the data

tiledlayout(3,6);

nexttile(1, [1 3])
plot(Timestamps, Mediciones, "LineWidth", 2)
xlabel('Time (s)')
ylabel('Measurements')
legend('V_k')
grid on

nexttile(7, [1 3])
plot(Timestamps, Poses_x, "LineWidth", 2)
xlabel('Time (s)')
ylabel('X Position')
legend('x_ee')
grid on

nexttile(13, [1 3])
plot(Timestamps, Poses_y, "LineWidth", 2)
xlabel('Time (s)')
ylabel('Y Position')
legend('y_ee')
grid on

nexttile(4, [3 3])
patch([Poses_x; NaN], [Poses_y; NaN], [Mediciones; NaN],...
      'EdgeColor','interp',"LineWidth", 3)
cb = colorbar;
cb.Label.String = 'V_k';
xlabel('x_1')
ylabel('x_2')
legend('X_e(t)')
grid on
axis equal
