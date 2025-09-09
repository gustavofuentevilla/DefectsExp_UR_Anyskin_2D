close all
clear
clc

%% Include and create msgs needed from ROS installation

folderPath = fullfile(pwd, "custom");
ros2genmsg(folderPath)

%% Create ROS 2 node
nodeMATLAB = ros2node("/MATLAB_Node", 0);

% Create publishers
pub_pose = ros2publisher(nodeMATLAB,...
                    "/cartesian_compliance_controller/target_frame",...
                    "geometry_msgs/PoseStamped");
pub_wrench = ros2publisher(nodeMATLAB,...
                    "/cartesian_compliance_controller/target_wrench",...
                    "geometry_msgs/WrenchStamped");

% Create subscribers
subs_pose = ros2subscriber(nodeMATLAB, ...
                          "/ee_pose_fast", ...         
                          "geometry_msgs/PoseStamped");

subs_data = ros2subscriber(nodeMATLAB, ...
                          "/synced_data", ...         
                          "custom_interfaces/SyncData");

% Create message structure
msg_pose = ros2message(pub_pose);
msg_wrench = ros2message(pub_wrench);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Drive UR to initial position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Pose_0 = [0.2; 0.05];

[Current_pose, status, statusText] = receive(subs_pose, 10);

P_i = [Current_pose.pose.position.x;
       Current_pose.pose.position.y;
       Current_pose.pose.position.z;
       Current_pose.pose.orientation.x;
       Current_pose.pose.orientation.y;
       Current_pose.pose.orientation.z;
       Current_pose.pose.orientation.w];

% P_f = [0.395; 0.115; 0.3; sqrt(2)/2; sqrt(2)/2; 0.0; 0.0];
P_f = [Pose_0(1); Pose_0(2); 0.1; sqrt(2)/2; sqrt(2)/2; 0.0; 0.0];

mvr_time = 5;
f_z = 0.0;
ur_p_to_p_traj(P_i, P_f, mvr_time, f_z, nodeMATLAB,...
               pub_pose, pub_wrench, msg_pose, msg_wrench);

%% Drive UR to contact position
[Current_pose, status, statusText] = receive(subs_pose, 10);

P_i = [Current_pose.pose.position.x;
       Current_pose.pose.position.y;
       Current_pose.pose.position.z;
       Current_pose.pose.orientation.x;
       Current_pose.pose.orientation.y;
       Current_pose.pose.orientation.z;
       Current_pose.pose.orientation.w];

% computes the z-coordinate needed to reach f = 5 N given 3 tested points
% and the desired x-y position in the plane
z_contact = plane_penetration(Pose_0);

P_f = [Pose_0(1); Pose_0(2); z_contact; sqrt(2)/2; sqrt(2)/2; 0.0; 0.0];
f_z = 5.0;

% Movement from P_i to P_f smoothly
ur_p_to_p_traj(P_i, P_f, mvr_time, f_z, nodeMATLAB,...
               pub_pose, pub_wrench, msg_pose, msg_wrench);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute and send first ergodic trajectory
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Read data and compute next ergodic trajectory
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


