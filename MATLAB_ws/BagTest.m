close all
clear
clc

%%

%nodeMATLAB = ros2node("nodeMATLAB", 0);

folderPath = fullfile(pwd, "custom");
ros2genmsg(folderPath)

%% Check for msg

% ros2 msg show custom_interfaces/msg/SyncData

%%

folderPathBag = fullfile(pwd, "ROS2Bags/BagSyncData"); 
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
combinedData = [Timestamps, Poses_x, Poses_y, Poses_z, Mediciones];

%% Create table of combinedData
combinedTable = array2table(combinedData, 'VariableNames', {'Time', 'PosX', 'PosY', 'PosZ', 'Measurements'});

%% Plot the combined data

figure
yyaxis left
plot(Timestamps, Poses_x, Timestamps, Poses_y, Timestamps, Poses_z)
xlabel('Time (s)');
ylabel('Position');
title('End-Effector Positions and Measurements Over Time');
yyaxis right
plot(Timestamps, Mediciones)
ylabel('Measurements');
legend('Position X', 'Position Y', 'Position Z', 'Measurements');
