%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Trayectoria Recta
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Toma en cuenta que el robot ya debe estar en posición inicial y en
% contacto con la superficie! (esto se ejecuta en main.m)

% Parámetros

rate = 100;                % Hz
loop_rate = ros2rate(nodeMATLAB, rate);
% sleeping time between cicles
sleep_time = 2;

t_1 = 0;
t_2 = 4;
t_3 = 8;
t_4 = 12;
t_5 = 16;
x_1 = 0.28; 
y_1 = 0.10;
x_2 = 0.4;
y_2 = 0.24;

%Trayectorias de prueba [x_1; y_1] --> [x_2; y_2]
% de [0.28; 0.10]; --> [0.16; 0.24];
% de [0.28; 0.10]; --> [0.4; 0.24];

% Send trajectory from X_1 to X_2 (First cycle)
t = t_1;
x_i = x_1; % Set initial position for the next trajectory
x_f = x_2; % Set final position for the next trajectory
y_i = y_1;
y_f = y_2;
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
    msg_pose.pose.position.y = y_i + (y_f - y_i)*(t - t_i)/(t_f - t_i);
    msg_pose.pose.position.z = desiredZ(msg_pose.pose.position.x,...
                                        msg_pose.pose.position.y);

    msg_pose.pose.orientation.x = desiredOrientation(1);
    msg_pose.pose.orientation.y = desiredOrientation(2);
    msg_pose.pose.orientation.z = desiredOrientation(3);
    msg_pose.pose.orientation.w = desiredOrientation(4);

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
pause(sleep_time)

% Send trajectory back from X_2 to X_1 (End of first cicle)
t = t_2;
x_i = x_2; 
x_f = x_1; 
y_i = y_2;
y_f = y_1;
t_i = t_2; 
t_f = t_3; 
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
    msg_pose.pose.position.y = y_i + (y_f - y_i)*(t - t_i)/(t_f - t_i);
    msg_pose.pose.position.z = desiredZ(msg_pose.pose.position.x,...
                                        msg_pose.pose.position.y);

    msg_pose.pose.orientation.x = desiredOrientation(1);
    msg_pose.pose.orientation.y = desiredOrientation(2);
    msg_pose.pose.orientation.z = desiredOrientation(3);
    msg_pose.pose.orientation.w = desiredOrientation(4);

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
pause(sleep_time)

% Send trajectory from X_1 to X_2 (second cicle)
t = t_3;
x_i = x_1; 
x_f = x_2; 
y_i = y_1;
y_f = y_2;
t_i = t_3; 
t_f = t_4; 
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
    msg_pose.pose.position.y = y_i + (y_f - y_i)*(t - t_i)/(t_f - t_i);
    msg_pose.pose.position.z = desiredZ(msg_pose.pose.position.x,...
                                        msg_pose.pose.position.y);

    msg_pose.pose.orientation.x = desiredOrientation(1);
    msg_pose.pose.orientation.y = desiredOrientation(2);
    msg_pose.pose.orientation.z = desiredOrientation(3);
    msg_pose.pose.orientation.w = desiredOrientation(4);

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
pause(sleep_time)

% Send trajectory back from X_2 to X_1 (End of second cicle)
t = t_4;
x_i = x_2; 
x_f = x_1; 
y_i = y_2;
y_f = y_1;
t_i = t_4; 
t_f = t_5; 
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
    msg_pose.pose.position.y = y_i + (y_f - y_i)*(t - t_i)/(t_f - t_i);
    msg_pose.pose.position.z = desiredZ(msg_pose.pose.position.x,...
                                        msg_pose.pose.position.y);

    msg_pose.pose.orientation.x = desiredOrientation(1);
    msg_pose.pose.orientation.y = desiredOrientation(2);
    msg_pose.pose.orientation.z = desiredOrientation(3);
    msg_pose.pose.orientation.w = desiredOrientation(4);

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
pause(sleep_time)
