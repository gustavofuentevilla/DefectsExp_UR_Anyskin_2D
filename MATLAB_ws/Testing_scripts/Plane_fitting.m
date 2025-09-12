close all
clear
clc

%% Empirical tested points with the z-coordinate reaching f = 5 N
P_1 = [0.2; 0.05; -0.0056];
P_2 = [0.395; 0.115; -0.0075];
P_3 = [0.25; 0.24; -0.0105];
P_4 = [0.16; 0.0; -0.0039];
P_5 = [0.16; 0.05; -0.0053];
P_6 = [0.16; 0.1; -0.0067];
P_7 = [0.16; 0.15; -0.0079];
P_8 = [0.16; 0.2; -0.0092];
P_9 = [0.16; 0.24; -0.0102];
P_10 = [0.2; 0.24; -0.0104];
P_11 = [0.2; 0.2; -0.0094];
P_12 = [0.2; 0.10; -0.0069];
P_13 = [0.2; 0.0; -0.0041];
P_14 = [0.25; 0.0; -0.0043];
P_15 = [0.25; 0.05; -0.0056];
P_16 = [0.25; 0.1; -0.0070];
P_17 = [0.25; 0.20; -0.0096];
P_18 = [0.30; 0.24; -0.0106];
P_19 = [0.30; 0.20; -0.0097];
P_20 = [0.30; 0.15; -0.0084];
P_21 = [0.30; 0.10; -0.0072];
P_22 = [0.30; 0.0; -0.0042];
P_23 = [0.35; 0.0; -0.0042];
P_24 = [0.35; 0.10; -0.0071];
P_25 = [0.35; 0.15; -0.0085];
P_26 = [0.35; 0.24; -0.0107];
P_27 = [0.40; 0.24; -0.0106];
P_28 = [0.40; 0.20; -0.0096];
P_29 = [0.40; 0.15; -0.0084];
P_30 = [0.40; 0.10; -0.0072];
P_31 = [0.40; 0.05; -0.0057];
P_32 = [0.40; 0.0; -0.0041];
P_33 = [0.44; 0.0; -0.0042];
P_34 = [0.44; 0.05; -0.0055];
P_35 = [0.44; 0.10; -0.0070];
P_36 = [0.44; 0.15; -0.0083];
P_37 = [0.44; 0.20; -0.0095];
P_38 = [0.44; 0.24; -0.0103];

Points = [P_1, P_2, P_3, P_4, P_5, P_6, P_7, P_8, P_9, P_10, P_11, P_12,...
          P_13, P_14, P_15, P_16, P_17, P_18, P_19, P_20, P_21, P_22,...
          P_23, P_24, P_25, P_26, P_27, P_28, P_29, P_30, P_31, P_32,...
          P_33, P_34, P_35, P_36, P_37, P_38]';

x = Points(:, 1);
y = Points(:, 2);
z = Points(:, 3);

%% Fitting data to a plane 
% Plane => Ax + By + Cz + D = 0

Eq = @(A, B, C, D, x, y) (-A*x - B*y - D)/C;

ft = fittype(Eq, 'independent', {'x', 'y'}, 'dependent', 'z');

desiredZ = fit([x, y], z, ft);

%% Get coefficients to compute normal vector and orientation
coeffs = coeffvalues(desiredZ);
n_x = coeffs(1);
n_y = coeffs(2);
n_z = coeffs(3);

% vector normal al plan n_hat = a*i_hat + b*j_hat + c*k_hat
normal_vect = [n_x; n_y; n_z];

% Orientación del efector final
z_ee = -normal_vect; % vector objetivo
x_world = [1; 0; 0]; % Eje X de {world}

% Aseguramos que z_ee está normalizado
z_ee = z_ee / norm(z_ee);

% Calcula el eje X del efector final como el producto cruzado entre x_world
% y el vector de interés (que ahora es Z del efector final): Esto orienta a
% x_ee con y_world mas o menos en el mismo sentido
x_ee = cross(x_world, z_ee);
x_ee = x_ee / norm(x_ee);

% Ahora calcula Y del efector para que sea ortogonal a Z y X
y_ee = cross(z_ee, x_ee);
y_ee = y_ee / norm(y_ee);

% Matriz de rotación: cada columna es un eje del efector final en {world}
R_e = [x_ee, y_ee, z_ee];

% Convertir matriz de rotación a cuaternión en formato (w, x, y, z)
quat_ee = rotm2quat(R_e);

% Orientación deseada (x, y, z, w) del efector final del UR sobre el plano 
desiredOrientation = [quat_ee(2:4), quat_ee(1)]';

%% plotting

% Define a point on the plane for the normal vector
x_avg = mean(x);
y_avg = mean(y);
pointOnPlane = [x_avg; y_avg; desiredZ(x_avg, y_avg)];

plot(desiredZ,[x,y],z)

xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Surface Fit');
grid on;
hold on
% Calculate the normal vector's direction for visualization
quiver3(pointOnPlane(1), pointOnPlane(2), pointOnPlane(3),...
        normal_vect(1), normal_vect(2), normal_vect(3),...
        0.005, 'LineWidth', 3);
plotFrame(pointOnPlane, R_e, 'Name', 'End-effector Frame', 'Scale', 0.05)
hold off
legend("Datos empíricos", "Vector normal")

%% Guardar función para usarla en main.m
save('Plane_Fit.mat', 'desiredZ', 'desiredOrientation');

