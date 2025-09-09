close all
clear
clc

%% Include and create msgs needed from ROS installation

% folderPath = fullfile(pwd, "custom");
% ros2genmsg(folderPath)

%% Create ROS 2 node
nodeMATLAB = ros2node("/MATLAB_Node", 0);

% Create publisher for PoseStamped messages
pub_pose = ros2publisher(nodeMATLAB,...
                    "/cartesian_compliance_controller/target_frame",...
                    "geometry_msgs/PoseStamped");
pub_wrench = ros2publisher(nodeMATLAB,...
                    "/cartesian_compliance_controller/target_wrench",...
                    "geometry_msgs/WrenchStamped");

% Create message structure
msg_pose = ros2message(pub_pose);
msg_wrench = ros2message(pub_wrench);

%% Trajectory parameters

rate = 100;                % Hz
loop_rate = ros2rate(nodeMATLAB, rate);

t_1 = 10;
t_2 = 14;
t_3 = 18;
t_4 = 22;
t_5 = 26;
x_1 = 0.275;
x_2 = 0.395;
y_prime = 0.05;

%% Loop time setup

% Initial pose loop, ensure touching the plane with 5 N in Z
t = 0;
reset(loop_rate);
while t < t_1
    % Update timestamp
    now = ros2time(nodeMATLAB, "now");
    msg_pose.header.stamp.sec = int32(now.sec);
    msg_pose.header.stamp.nanosec = uint32(now.nanosec);
    msg_pose.header.frame_id = 'world';  % reference frame
    msg_wrench.header.stamp.sec = int32(now.sec);
    msg_wrench.header.stamp.nanosec = uint32(now.nanosec);
    msg_wrench.header.frame_id = 'ee_link'; % 'ur5e_tool0';  % reference frame

    % Initial pose for circular trajectory
    msg_pose.pose.position.x = x_1;
    msg_pose.pose.position.y = y_prime;
    msg_pose.pose.position.z = -0.005; % contacto en el plano

    msg_pose.pose.orientation.x = sqrt(2)/2;
    msg_pose.pose.orientation.y = sqrt(2)/2;
    msg_pose.pose.orientation.z = 0.0;
    msg_pose.pose.orientation.w = 0.0;

    % Desired Wrench
    msg_wrench.wrench.force.x = 0.0;
    msg_wrench.wrench.force.y = 0.0;
    msg_wrench.wrench.force.z = 5.0;
    msg_wrench.wrench.torque.x = 0.0;
    msg_wrench.wrench.torque.y = 0.0;
    msg_wrench.wrench.torque.z = 0.0;

    % Publish the message
    send(pub_pose, msg_pose);
    send(pub_wrench, msg_wrench);
    disp(t + " published pose and wrench")

    % Wait for the next iteration
    waitfor(loop_rate);
    t = t + 1/rate;
end

%% Send trajectory from X_1 to X_2 (First cycle)
t = t_1;
x_i = x_1; % Set initial position for the next trajectory
x_f = x_2; % Set final position for the next trajectory
t_i = t_1; % Set initial time for the next trajectory
t_f = t_2; % Set final time for the next trajectory
reset(loop_rate);
while t < t_2
    % Update timestamp
    now = ros2time(nodeMATLAB, "now");
    msg_pose.header.stamp.sec = int32(now.sec);
    msg_pose.header.stamp.nanosec = uint32(now.nanosec);
    msg_pose.header.frame_id = 'world';  % reference frame
    msg_wrench.header.stamp.sec = int32(now.sec);
    msg_wrench.header.stamp.nanosec = uint32(now.nanosec);
    msg_wrench.header.frame_id = 'ee_link'; % 'ur5e_tool0';  % reference frame

    % trajectory
    msg_pose.pose.position.x = x_i + (x_f - x_i)*(t - t_i)/(t_f - t_i);
    msg_pose.pose.position.y = y_prime;
    msg_pose.pose.position.z = -0.005;

    msg_pose.pose.orientation.x = sqrt(2)/2;
    msg_pose.pose.orientation.y = sqrt(2)/2;
    msg_pose.pose.orientation.z = 0.0;
    msg_pose.pose.orientation.w = 0.0;

    % Desired Wrench
    msg_wrench.wrench.force.x = 0.0;
    msg_wrench.wrench.force.y = 0.0;
    msg_wrench.wrench.force.z = 5.0;
    msg_wrench.wrench.torque.x = 0.0;
    msg_wrench.wrench.torque.y = 0.0;
    msg_wrench.wrench.torque.z = 0.0;

    % Publish the message
    send(pub_pose, msg_pose);
    send(pub_wrench, msg_wrench);
    disp(t + " published pose and wrench")

    % Wait for the next iteration
    waitfor(loop_rate);
    t = t + 1/rate;
end
pause(5)

%% Send trajectory back from X_2 to X_1 (End of first cicle)
t = t_2;
x_i = x_2; % Set initial position for the next trajectory
x_f = x_1; % Set final position for the next trajectory
t_i = t_2; % Set initial time for the next trajectory
t_f = t_3; % Set final time for the next trajectory
reset(loop_rate);
while t < t_3
    % Update timestamp
    now = ros2time(nodeMATLAB, "now");
    msg_pose.header.stamp.sec = int32(now.sec);
    msg_pose.header.stamp.nanosec = uint32(now.nanosec);
    msg_pose.header.frame_id = 'world';  % reference frame
    msg_wrench.header.stamp.sec = int32(now.sec);
    msg_wrench.header.stamp.nanosec = uint32(now.nanosec);
    msg_wrench.header.frame_id = 'ee_link'; % 'ur5e_tool0';  % reference frame

    % trajectory
    msg_pose.pose.position.x = x_i + (x_f - x_i)*(t - t_i)/(t_f - t_i);
    msg_pose.pose.position.y = y_prime;
    msg_pose.pose.position.z = -0.005;

    msg_pose.pose.orientation.x = sqrt(2)/2;
    msg_pose.pose.orientation.y = sqrt(2)/2;
    msg_pose.pose.orientation.z = 0.0;
    msg_pose.pose.orientation.w = 0.0;

    % Desired Wrench
    msg_wrench.wrench.force.x = 0.0;
    msg_wrench.wrench.force.y = 0.0;
    msg_wrench.wrench.force.z = 5.0;
    msg_wrench.wrench.torque.x = 0.0;
    msg_wrench.wrench.torque.y = 0.0;
    msg_wrench.wrench.torque.z = 0.0;

    % Publish the message
    send(pub_pose, msg_pose);
    send(pub_wrench, msg_wrench);
    disp(t + " published pose and wrench")

    % Wait for the next iteration
    waitfor(loop_rate);
    t = t + 1/rate;
end
pause(5)

%% Send trajectory from X_1 to X_2 (second cicle)
t = t_3;
x_i = x_1; % Set initial position for the next trajectory
x_f = x_2; % Set final position for the next trajectory
t_i = t_3; % Set initial time for the next trajectory
t_f = t_4; % Set final time for the next trajectory
reset(loop_rate);
while t < t_4
    % Update timestamp
    now = ros2time(nodeMATLAB, "now");
    msg_pose.header.stamp.sec = int32(now.sec);
    msg_pose.header.stamp.nanosec = uint32(now.nanosec);
    msg_pose.header.frame_id = 'world';  % reference frame
    msg_wrench.header.stamp.sec = int32(now.sec);
    msg_wrench.header.stamp.nanosec = uint32(now.nanosec);
    msg_wrench.header.frame_id = 'ee_link'; % 'ur5e_tool0';  % reference frame

    % trajectory
    msg_pose.pose.position.x = x_i + (x_f - x_i)*(t - t_i)/(t_f - t_i);
    msg_pose.pose.position.y = y_prime;
    msg_pose.pose.position.z = -0.005;

    msg_pose.pose.orientation.x = sqrt(2)/2;
    msg_pose.pose.orientation.y = sqrt(2)/2;
    msg_pose.pose.orientation.z = 0.0;
    msg_pose.pose.orientation.w = 0.0;

    % Desired Wrench
    msg_wrench.wrench.force.x = 0.0;
    msg_wrench.wrench.force.y = 0.0;
    msg_wrench.wrench.force.z = 5.0;
    msg_wrench.wrench.torque.x = 0.0;
    msg_wrench.wrench.torque.y = 0.0;
    msg_wrench.wrench.torque.z = 0.0;

    % Publish the message
    send(pub_pose, msg_pose);
    send(pub_wrench, msg_wrench);
    disp(t + " published pose and wrench")

    % Wait for the next iteration
    waitfor(loop_rate);
    t = t + 1/rate;
end
pause(5)

%% Send trajectory back from X_2 to X_1 (End of second cicle)
t = t_4;
x_i = x_2; % Set initial position for the next trajectory
x_f = x_1; % Set final position for the next trajectory
t_i = t_4; % Set initial time for the next trajectory
t_f = t_5; % Set final time for the next trajectory
reset(loop_rate);
while t < t_5
    % Update timestamp
    now = ros2time(nodeMATLAB, "now");
    msg_pose.header.stamp.sec = int32(now.sec);
    msg_pose.header.stamp.nanosec = uint32(now.nanosec);
    msg_pose.header.frame_id = 'world';  % reference frame
    msg_wrench.header.stamp.sec = int32(now.sec);
    msg_wrench.header.stamp.nanosec = uint32(now.nanosec);
    msg_wrench.header.frame_id = 'ee_link'; % 'ur5e_tool0';  % reference frame

    % trajectory
    msg_pose.pose.position.x = x_i + (x_f - x_i)*(t - t_i)/(t_f - t_i);
    msg_pose.pose.position.y = y_prime;
    msg_pose.pose.position.z = -0.005;

    msg_pose.pose.orientation.x = sqrt(2)/2;
    msg_pose.pose.orientation.y = sqrt(2)/2;
    msg_pose.pose.orientation.z = 0.0;
    msg_pose.pose.orientation.w = 0.0;

    % Desired Wrench
    msg_wrench.wrench.force.x = 0.0;
    msg_wrench.wrench.force.y = 0.0;
    msg_wrench.wrench.force.z = 5.0;
    msg_wrench.wrench.torque.x = 0.0;
    msg_wrench.wrench.torque.y = 0.0;
    msg_wrench.wrench.torque.z = 0.0;

    % Publish the message
    send(pub_pose, msg_pose);
    send(pub_wrench, msg_wrench);
    disp(t + " published pose and wrench")

    % Wait for the next iteration
    waitfor(loop_rate);
    t = t + 1/rate;
end
pause(5)

%% Leave the task
t = 0;
reset(loop_rate);
while t < 5
    % Update timestamp
    now = ros2time(nodeMATLAB, "now");
    msg_pose.header.stamp.sec = int32(now.sec);
    msg_pose.header.stamp.nanosec = uint32(now.nanosec);
    msg_pose.header.frame_id = 'world';  % reference frame
    msg_wrench.header.stamp.sec = int32(now.sec);
    msg_wrench.header.stamp.nanosec = uint32(now.nanosec);
    msg_wrench.header.frame_id = 'ee_link'; % 'ur5e_tool0';  % reference frame

    % Pose
    msg_pose.pose.position.z = 0.05;

    msg_pose.pose.orientation.x = sqrt(2)/2;
    msg_pose.pose.orientation.y = sqrt(2)/2;
    msg_pose.pose.orientation.z = 0.0;
    msg_pose.pose.orientation.w = 0.0;

    % Desired Wrench
    msg_wrench.wrench.force.x = 0.0;
    msg_wrench.wrench.force.y = 0.0;
    msg_wrench.wrench.force.z = 0.0;
    msg_wrench.wrench.torque.x = 0.0;
    msg_wrench.wrench.torque.y = 0.0;
    msg_wrench.wrench.torque.z = 0.0;

    % Publish the message
    send(pub_pose, msg_pose);
    send(pub_wrench, msg_wrench);
    disp(t + " published pose and wrench")

    % Wait for the next iteration
    waitfor(loop_rate);
    t = t + 1/rate;
end

%% Prueba trayectoria

% tt = (0:1/rate:duration)';
% 
% delta_tmp = 2*pi*tt/duration - sin(2*pi*tt/duration);
% 
% x = x_c + radius*cos(delta_tmp);
% y = y_c + radius*sin(delta_tmp);
% 
% plot(x,y,"--")
% axis equal
% hold on
% comet(x,y)
% hold off