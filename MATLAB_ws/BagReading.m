function Data = BagReading(folderPathBag)

% Ejecutar esto afuera
% folderPath = fullfile(pwd, "custom");
% ros2genmsg(folderPath)

% folderPathBag = fullfile(pwd, "ROS2Bags/rosbag_20251024_185001_144916");

bagReader = ros2bagreader(folderPathBag);
messageAll = readMessages(bagReader);

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

end