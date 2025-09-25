function ur_traj_init(P_i, P_f, mvr_time, nodeMATLAB, pub_pose, pub_wrench, msg_pose, msg_wrench)

rate = 100;                % Hz
loop_rate = ros2rate(nodeMATLAB, rate);
t_i = 0;
t = t_i;
reset(loop_rate);
while t < mvr_time
    % Update timestamp
    now = ros2time(nodeMATLAB, "now");
    msg_pose.header.stamp.sec = int32(now.sec);
    msg_pose.header.stamp.nanosec = uint32(now.nanosec);
    msg_pose.header.frame_id = 'world';  % reference frame
    msg_wrench.header.stamp.sec = int32(now.sec);
    msg_wrench.header.stamp.nanosec = uint32(now.nanosec);
    msg_wrench.header.frame_id = 'ee_link'; % reference frame

    % Desired pose
    P = rest_to_rest_trajectory(P_i, P_f, t_i, mvr_time, t, 2);
    msg_pose.pose.position.x = P(1);
    msg_pose.pose.position.y = P(2);
    msg_pose.pose.position.z = P(3);
    msg_pose.pose.orientation.x = P(4);
    msg_pose.pose.orientation.y = P(5);
    msg_pose.pose.orientation.z = P(6);
    msg_pose.pose.orientation.w = P(7);

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

end