%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Circular Trajectory
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Toma en cuenta que el robot ya debe estar en posición inicial y en
% contacto con la superficie! (esto se ejecuta en main.m)

% Parámetros
x_c = (0.15 + 0.44)/2;
y_c = (0.23/2);
radius = 0.1;             % radius of the circle
% x_c = 0.3450;
% y_c = (0.23/2);
% radius = 0.05;             % radius of the circle
duration = 30;            % seconds
rate = 100;                % Hz
loop_rate = ros2rate(nodeMATLAB, rate);

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

    % Circular trajectory
    delta = 2*pi*t/duration - sin(2*pi*t/duration);
    msg_pose.pose.position.x = x_c + radius * cos(delta);
    msg_pose.pose.position.y = y_c - radius * sin(delta);
    % contacto en el plano
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

