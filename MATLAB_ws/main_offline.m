close all
% clear
clearvars -except UR_N100_v2
clc

% CasADi 
import casadi.*

% loading casadi function object (comment if it's already loaded)
% UR_N100_v2 = Function.load('CasADi_Formulation/UR_N100_v2.casadi');

%% Include and create custom messages

% folderPath = fullfile(pwd, "custom");
% ros2genmsg(folderPath)

%% Initializations

Initializations

% Times
T_ErgC_i = zeros(n_iter_max, 1);
T_PDF_i = zeros(n_iter_max, 1);

%% Offline Loop

% Iteration 
i = 2;

    % Soluciones
    t_erg_init = tic;
    [Z, U] = UR_N100_v2(z_act, phi_k_act); 
    T_ErgC_i(i) = toc(t_erg_init);
    Z = full(Z)';
    U = full(U)';

    % Desired Ergodic Trajectory X_e_d(t) = [x_1, x_2]
    X_e_d = [Z(:, 1), Z(:, 3)];
    X_e_d_dot = [Z(:, 2), Z(:, 4)];

    % Spline: Adding points to the trajectory to get more data from the sensor
    % and pass it to the estimator
    x_1e_d_spline = spline(t, X_e_d(:,1), t_spline);
    x_2e_d_spline = spline(t, X_e_d(:,2), t_spline);
    X_e_d_spline = [x_1e_d_spline, x_2e_d_spline];
    
    % La derivada del spline en realidad se tiene que calcular con diff(*)
    x_1e_dot_spline = spline(t, X_e_d_dot(:,1), t_spline);
    x_2e_dot_spline = spline(t, X_e_d_dot(:,2), t_spline);
    X_e_dot_spline = [x_1e_dot_spline, x_2e_dot_spline];

    u_1_spline = spline(t(1:end-1), U(:,1), t_spline(1:end-1));
    u_2_spline = spline(t(1:end-1), U(:,2), t_spline(1:end-1));
    u_spline = [u_1_spline, u_2_spline];
    
    % Guardar trayectoria en un archivo
    ErgodicTraj = [t_spline, X_e_d_spline];
    T = array2table(ErgodicTraj, 'VariableNames', {'Tiempo', 'x_ee', 'y_ee'});
    archivo = "/home/gustavo-fuentevilla/DefectsExp_UR/Tests2/N100/" + ...
                "0Def/Test/trayectoria_" + i + ".csv";
    writetable(T, archivo) % char(archivo) para cambiar a comillas simples

    %% Ejecutar las rutinas de movimiento en el robot %%%%%%%%%%%%%%%%%%%%%
    % En la terminal con ROS2 lanzar el robot y luego la trayectoria
    %
    % ros2 launch easy_ur_control easy_ur_launcher.launch.py robot_ip:=192.168.100.10 ur_type:=ur3e ctrl:=cartesian_compliance_controller;
    % ros2 launch ur_motion_routines ur_motion_routines.launch.py i:=1;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% Leer datos obtenidos

    % Función para Leer los Datos (con la matriz de datos de salida)
    folderPathBag = fullfile("/home", "gustavo-fuentevilla",...
                    "DefectsExp_UR", "Tests2", "N100", "0Def", "Test",...
                    "synced_data_" + i + ".csv");
    full_data = csvReading(folderPathBag);

    % Función para el pre-processing de los datos (filtro de butterworth, 
    % aplicar el threshold y obtener los datos que si interesan)
    % Data = [X_e_act, sensorSignal]
    planned_vel = [t, X_e_d_dot];
    [Data_current, Preprocess_struct(i)] = Preprocessing(full_data,...
                                                Thresholds,...
                                                planned_vel,...
                                                Par_PDF.flag_ExplorationStage);
    
    % DatosRegistro = [t, xee, yee, zee, signal, clean_signal]
    Data_t_Xe_V = [full_data, Preprocess_struct(i).clean_signal];

    % Plot Bag
    plotBag(Data_t_Xe_V, Thresholds, Preprocess_struct(i), X_e_d,...
            Par_PDF.flag_ExplorationStage)

    %% Registers
    z_reg(:,:,i) = Z;
    u_reg(:,:,i) = U;
    X_e_reg(:,:,i) = X_e_d;
    X_e_dot_reg(:,:,i) = X_e_d_dot;
    phi_k_REG(:,:,i) = phi_k_reg;
    Phi_hat_x_reg(:,:,i) = Phi_hat_x_act;
    X_e_spline_reg(:,:,i) = X_e_d_spline;
    X_e_dot_spline_reg(:,:,i) = X_e_dot_spline;
    u_spline_reg(:,:,i) = u_spline;
    Data_t_Xe_V_reg(1:size(Data_t_Xe_V,1),1:size(Data_t_Xe_V,2),i) = Data_t_Xe_V;
    % V_Xe_reg(:,:,i) = V_Xe;

    % PDF Estimation
    Par_PDF.iteration = i;
    Par_PDF.Prev_Phi_hat_x = Phi_hat_x_act;

    t_pdf_init = tic;
    [Phi_hat_x_next, Estim_sol{i}] = PDF_Estimator(Data_current, Par_PDF);
    T_PDF_i(i) = toc(t_pdf_init);

    % Update Iterations Counter where No data hav been found
    Par_PDF.NoDataIterCounter = Par_PDF.NoDataIterCounter + Estim_sol{i}.flag_NoData;

    if Par_PDF.NoDataIterCounter >= 2
        disp("No se registró ningún dato arriba del threshold en " + i + " iteraciones")
        n_iter = i;
        % break;
    end

    % Save D_KL from first iteration to set the exploration function
    if (i == 1) || (i == 2 && Par_PDF.NoDataIterCounter == 1)
        Par_PDF.D_KL_bar_u = Estim_sol{i}.D_KL;
    end

    % Detect the falling edge of Exploration flag termination
    if i > 1
        falling_edge = (Estim_sol{i}.flag_ExplorationStage - ...
                        Estim_sol{i-1}.flag_ExplorationStage) == -1;
    else
        falling_edge = false;
    end
    % Save the Exploration flag (turned off) and the number of components 
    if falling_edge
        Par_PDF.flag_ExplorationStage = Estim_sol{i}.flag_ExplorationStage;
        Par_PDF.Prev_numComponents = Estim_sol{i}.numComponents;
    end

    % Save found defects if any
    Par_PDF.Prev_Mu_found = cat(1, Par_PDF.Prev_Mu_found, Estim_sol{i}.Mu_found);
    Par_PDF.Prev_Sigma_found = cat(3, Par_PDF.Prev_Sigma_found, Estim_sol{i}.Sigma_found);

    if Estim_sol{i}.flag_done
        n_iter = i; % Save number of iterations achieved
        disp("Algoritmo terminado en " + i + " iteraciones")
    end

    % Saving Data to use it as "Previous data" in next iterations
    Par_PDF.Prev_Data = Estim_sol{i}.Data;

    % Compute new Fourier coefficients for \hat{Phi}(x)
    [phi_k_reg, ~, ~] = FourierCoef_RefPDF(Phi_hat_x_next, Par_struct);

    % Update parameters for next iteration
    z_act = Z(end,:)';           % Initial condition for state
    phi_k_act = phi_k_reg;       % New target coefficients
    Phi_hat_x_act = Phi_hat_x_next;
    
    % Plot next PDF for debbuging
    figure;
    pcolor(x_1_grid, x_2_grid, ...
           reshape(Phi_hat_x_next, length(x_2), length(x_1)),...
           "FaceColor","interp","EdgeColor","none")
    xlim([L_1_l, L_1_u])
    ylim([L_2_l, L_2_u])
    title("Next PDF")
    xlabel('$x_1$ [m]')
    ylabel('$x_2$ [m]')
    axis equal tight
    grid on


%% Remove the initial value (zero values) for defects found
Mu_found = Par_PDF.Prev_Mu_found(2:end, :);
Sigma_found = Par_PDF.Prev_Sigma_found(:,:,2:end);

% ------------Guardar prueba
% save(sprintf("Results2/N200/1Def/output_.mat"), "-regexp", "^(?!(UR_N200_v2)$).");
% save(sprintf("Results2/N150/1Def/output_.mat"), "-regexp", "^(?!(UR_N150_v2)$).");
% save(sprintf("Results2/N100/1Def/output_.mat"), "-regexp", "^(?!(UR_N100_v2)$).");

%% Random Initial conditions

%----3 Def
% z_1 = [0.09, 0.10];
% z_2 = [0.24, 0.16];
% z_3 = [0.06, 0.16];
% z_4 = [0.24, 0.09];
% z_5 = [0.20, 0.04];
% z_6 = [0.12, 0.15];
% z_7 = [0.20, 0.16];
% z_8 = [0.17, 0.03];
% z_9 = [0.21, 0.16];
% z_10 = [0.17, 0.13];

%----2 Def
% z_1 = [0.23, 0.18];
% z_2 = [0.00, 0.10];
% z_3 = [0.18, 0.07];
% z_4 = [0.18, 0.15];
% z_5 = [0.16, 0.10];
% z_6 = [0.07, 0.16];
% z_7 = [0.11, 0.12];
% z_8 = [0.23, 0.05];
% z_9 = [0.10, 0.10];
% z_10 = [0.12, 0.01];

%----1 Defect
% z_1 = [0.23, 0.14];
% z_2 = [0.0, 0.0];
% z_3 = [0.10, 0.12];
% z_4 = [0.06, 0.13];
% z_5 = [0.05, 0.12];
% z_6 = [0.13, 0.13];
% z_7 = [0.18, 0.15];
% z_8 = [0.22, 0.07];
% z_9 = [0.18, 0.05];
% z_10 = [0.03, 0.13];

%----0 Defect
%z_1 = [0.14, 0.10];

%% Initial Condition generator

% clear
% close all
% clc
% 
% L_1_l = 0.0;
% L_1_u = 0.28;
% 
% L_2_l = 0.0;
% L_2_u = 0.20;
% 
% L_i_l = [L_1_l, L_2_l];
% L_i_u = [L_1_u, L_2_u];
% 
% offset = 0.0; 
% X0 = [];
% for i = 1:10
%     Mu_tmp = (L_i_l + offset) + ((L_i_u - offset) - ...
%              (L_i_l + offset)).*rand(1,2);
%     X0 = cat(1, X0, Mu_tmp); 
% end
% 
% scatter(X0(:,1), X0(:,2), 35, "black", "filled", "o")
% xlim([L_1_l, L_1_u])
% ylim([L_2_l, L_2_u])
% grid on