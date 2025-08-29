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
                    "/target_frame",...
                    "geometry_msgs/PoseStamped");
pub_wrench = ros2publisher(nodeMATLAB,...
                    "/cartesian_compliance_controller/target_wrench",...
                    "geometry_msgs/WrenchStamped");

% Create message structure
msg_pose = ros2message(pub_pose);
msg_wrench = ros2message(pub_wrench);

%% Trajectory parameters
x_c = (0.15 + 0.44)/2;
y_c = (0.23/2);
radius = 0.1;             % radius of the circle
duration = 20;            % seconds
rate = 100;                % Hz
loop_rate = ros2rate(nodeMATLAB, rate);

%% Loop time setup

% Initial pose loop, ensure touching the plane with 5 N in Z
t = 0;
reset(loop_rate);
while t < 10
    % Update timestamp
    now = ros2time(nodeMATLAB, "now");
    msg_pose.header.stamp.sec = int32(now.sec);
    msg_pose.header.stamp.nanosec = uint32(now.nanosec);
    msg_pose.header.frame_id = 'world';  % reference frame
    msg_wrench.header.stamp.sec = int32(now.sec);
    msg_wrench.header.stamp.nanosec = uint32(now.nanosec);
    msg_wrench.header.frame_id = 'ee_link'; % 'ur5e_tool0';  % reference frame

    % Initial pose for circular trajectory
    msg_pose.pose.position.x = 0.395;
    msg_pose.pose.position.y = 0.115;
    msg_pose.pose.position.z = 0.0;
    % Circular trajectory
    % msg_pose.pose.position.x = x_c + radius * cos(theta);
    % msg_pose.pose.position.y = y_c - radius * sin(theta);
    % msg_pose.pose.position.z = 0.0;

    msg_pose.pose.orientation.x = sqrt(2)/2;
    msg_pose.pose.orientation.y = sqrt(2)/2;
    msg_pose.pose.orientation.z = 0.0;
    msg_pose.pose.orientation.w = 0.0;

    % Desired Wrench
    % msg_wrench.wrench.force.x = 0.0;
    % msg_wrench.wrench.force.y = 0.0;
    msg_wrench.wrench.force.z = 5.0;
    % msg_wrench.wrench.torque.x = 0.0;
    % msg_wrench.wrench.torque.y = 0.0;
    % msg_wrench.wrench.torque.z = 0.0;

    % Publish the message
    send(pub_pose, msg_pose);
    send(pub_wrench, msg_wrench);
    disp(t + " published pose and wrench")

    % Wait for the next iteration
    waitfor(loop_rate);
    t = t + 1/rate;
end

% Send circular trajectory with a desired force in Z
t = 0;
reset(loop_rate);
while t < duration
    % Update timestamp
    now = ros2time(nodeMATLAB, "now");
    msg_pose.header.stamp.sec = int32(now.sec);
    msg_pose.header.stamp.nanosec = uint32(now.nanosec);
    msg_pose.header.frame_id = 'world';  % reference frame
    msg_wrench.header.stamp.sec = int32(now.sec);
    msg_wrench.header.stamp.nanosec = uint32(now.nanosec);
    msg_wrench.header.frame_id = 'ee_link'; % 'ur5e_tool0';  % reference frame

    % Circular trajectory (x = r*cos(θ), y = r*sin(θ))
    theta = angular_speed * t;
    % Initial pose for circular trajectory
    % msg_pose.pose.position.x = 0.395;
    % msg_pose.pose.position.y = 0.115;
    % msg_pose.pose.position.z = 0.0;
    % Circular trajectory
    delta = 2*pi*t/duration - sin(2*pi*t/duration);
    msg_pose.pose.position.x = x_c + radius * cos(delta);
    msg_pose.pose.position.y = y_c - radius * sin(delta);
    msg_pose.pose.position.z = 0.0;

    msg_pose.pose.orientation.x = sqrt(2)/2;
    msg_pose.pose.orientation.y = sqrt(2)/2;
    msg_pose.pose.orientation.z = 0.0;
    msg_pose.pose.orientation.w = 0.0;

    % Desired Wrench
    % msg_wrench.wrench.force.x = 0.0;
    % msg_wrench.wrench.force.y = 0.0;
    msg_wrench.wrench.force.z = 5.0;
    % msg_wrench.wrench.torque.x = 0.0;
    % msg_wrench.wrench.torque.y = 0.0;
    % msg_wrench.wrench.torque.z = 0.0;

    % Publish the message
    send(pub_pose, msg_pose);
    send(pub_wrench, msg_wrench);
    disp(t + " published pose and wrench")

    % Wait for the next iteration
    waitfor(loop_rate);
    t = t + 1/rate;
end

% Leave the task
t = 0;
reset(loop_rate);
while t < duration
    % Update timestamp
    now = ros2time(nodeMATLAB, "now");
    msg_pose.header.stamp.sec = int32(now.sec);
    msg_pose.header.stamp.nanosec = uint32(now.nanosec);
    msg_pose.header.frame_id = 'world';  % reference frame
    msg_wrench.header.stamp.sec = int32(now.sec);
    msg_wrench.header.stamp.nanosec = uint32(now.nanosec);
    msg_wrench.header.frame_id = 'ee_link'; % 'ur5e_tool0';  % reference frame

    % Circular trajectory (x = r*cos(θ), y = r*sin(θ))
    theta = angular_speed * t;
    % Initial pose for circular trajectory
    msg_pose.pose.position.x = 0.395;
    msg_pose.pose.position.y = 0.115;
    msg_pose.pose.position.z = 0.1;
    % Circular trajectory
    % msg_pose.pose.position.x = x_c + radius * cos(theta);
    % msg_pose.pose.position.y = y_c - radius * sin(theta);
    % msg_pose.pose.position.z = 0.0;

    msg_pose.pose.orientation.x = sqrt(2)/2;
    msg_pose.pose.orientation.y = sqrt(2)/2;
    msg_pose.pose.orientation.z = 0.0;
    msg_pose.pose.orientation.w = 0.0;

    % Desired Wrench
    % msg_wrench.wrench.force.x = 0.0;
    % msg_wrench.wrench.force.y = 0.0;
    if t > 10
        msg_wrench.wrench.force.z = 0.0;
    else 
        msg_wrench.wrench.force.z = 5.0;
    end
    % msg_wrench.wrench.torque.x = 0.0;
    % msg_wrench.wrench.torque.y = 0.0;
    % msg_wrench.wrench.torque.z = 0.0;

    % Publish the message
    send(pub_pose, msg_pose);
    send(pub_wrench, msg_wrench);
    disp(t + " published pose and wrench")

    % Wait for the next iteration
    waitfor(loop_rate);
    t = t + 1/rate;
end

%% Prueba trayectoria

tt = (0:1/rate:duration)';

delta_tmp = 2*pi*tt/duration - sin(2*pi*tt/duration);

x = x_c + radius*cos(delta_tmp);
y = y_c + radius*sin(delta_tmp);

plot(x,y,"--")
axis equal
hold on
comet(x,y)
hold off