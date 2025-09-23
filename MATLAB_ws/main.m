close all
% clear
clearvars -except UR_N100
clc

%% Cargar función que provee la penetración en Z y orientación deseadas
load('Testing_scripts/Plane_Fit.mat');

%% Include and create custom messages

folderPath = fullfile(pwd, "custom");
ros2genmsg(folderPath)

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create MATLAB ROS 2 node
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Node
nodeMATLAB = ros2node("/MATLAB_Node", 0);

% Publishers
pub_pose = ros2publisher(nodeMATLAB,...
                    "/cartesian_compliance_controller/target_frame",...
                    "geometry_msgs/PoseStamped");
pub_wrench = ros2publisher(nodeMATLAB,...
                    "/cartesian_compliance_controller/target_wrench",...
                    "geometry_msgs/WrenchStamped");

% Subscribers
subs_pose = ros2subscriber(nodeMATLAB, ...
                          "/ee_pose_fast", ...         
                          "geometry_msgs/PoseStamped");

subs_data = ros2subscriber(nodeMATLAB, ...
                          "/synced_data", ...         
                          "custom_interfaces/SyncData");

% Message structure
msg_pose = ros2message(pub_pose);
msg_wrench = ros2message(pub_wrench);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Drive UR to initial position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial position in X-Y plane
% Pose_0 = [0.2; 0.05];

%Testing
% Pose_0 = [0.3950; 0.1150]; %Circle
Pose_0 = [0.32; 0.12];

% Get current pose
[Current_pose, status, statusText] = receive(subs_pose, 10);

P_i = [Current_pose.pose.position.x;
       Current_pose.pose.position.y;
       Current_pose.pose.position.z;
       Current_pose.pose.orientation.x;
       Current_pose.pose.orientation.y;
       Current_pose.pose.orientation.z;
       Current_pose.pose.orientation.w];

% Desired Pose
% P_f = [Pose_0(1); Pose_0(2); 0.1; sqrt(2)/2; sqrt(2)/2; 0.0; 0.0];
P_f = [Pose_0(1); Pose_0(2); 0.1; desiredOrientation];

% Send trajectory to UR with a soft interpolation using Bezier
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

% Computes the z-coordinate needed to reach f = 5 N given more data points
z_desired = desiredZ(Pose_0');

P_f = [Pose_0(1); Pose_0(2); z_desired; desiredOrientation];
f_z = 5.0;

% UR movement from P_i to P_f smoothly
ur_p_to_p_traj(P_i, P_f, mvr_time, f_z, nodeMATLAB,...
               pub_pose, pub_wrench, msg_pose, msg_wrench);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute and send first ergodic trajectory
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% Crear distribución uniforme y obtener trayectoria




%%%% hacer un spline a la trayectoria y guardarla
% en Trayectoria = [t_spline, X_e, orientation]

%%%% Hacer una funcion Data = ur_send_traj(Trayectoria, parametros de nodo) 
% que ejecute la trayectoria en el robot real y guarde la información

%%%% filtrar datos

%%%% Crear nueva distribución (PDF_estimator)

TactExp_GMM
