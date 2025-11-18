close all
% clear
clearvars -except UR_N100
clc

% CasADi 
import casadi.*

% loading casadi function object (comment if it's already loaded)
% UR_N100 = Function.load('CasADi_Formulation/UR_N100.casadi');

%% Include and create custom messages

% folderPath = fullfile(pwd, "custom");
% ros2genmsg(folderPath)

%% Initializations

Initializations

%% Offline Loop

% Iteration 
i = 6;



    % Soluciones
    [Z, U] = UR_N100(z_act, phi_k_act); 
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
    archivo = "Trayectorias/trayectoria_" + i + ".csv";
    % writetable(T, archivo) % char(archivo) para cambiar a comillas simples

    %% Leer datos obtenidos (ejecutar después del movimientos del robot)

    % Función para Leer Rosbag (con la matriz de datos de salida)
    folderPathBag = fullfile(pwd, "ROS2Bags/Test1/synced_data_" + i + ".csv");
    full_data = csvReading(folderPathBag);

    % Función para el pre-processing de los datos (filtro de butterworth, 
    % aplicar el threshold y obtener los datos que si interesan)
    % Data = [X_e_act, sensorSignal]
    [Data_current, clean_signal] = Preprocessing(full_data, thres_meas);

    % DatosRegistro = [t, xee, yee, zee, signal, clean_signal]
    Data_t_Xe_V = [full_data, clean_signal];

    % Plot Bag
    plotBag(Data_t_Xe_V, thres_meas)

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
    [Phi_hat_x_next, Estim_sol(i)] = PDF_Estimator(Data_current, Par_PDF);

    % Update Iterations Counter where No data hav been found
    NoDataIterCounter = NoDataIterCounter + Estim_sol(i).flag_NoData;

    if NoDataIterCounter >= 2
        disp("No se registró ningún dato arriba del threshold en " + i + " iteraciones")
    end

    % Save D_KL from first iteration to set the exploration function
    if i == 1
        Par_PDF.D_KL_bar_u = Estim_sol(i).D_KL;
    end

    % Detect the falling edge of Exploration flag termination
    if i > 1
        falling_edge = (Estim_sol(i).flag_ExplorationStage - ...
                        Estim_sol(i-1).flag_ExplorationStage) == -1;
    else
        falling_edge = false;
    end
    % Save the Exploration flag (turned off) and the number of components 
    if falling_edge
        Par_PDF.flag_ExplorationStage = Estim_sol(i).flag_ExplorationStage;
        Par_PDF.Prev_numComponents = Estim_sol(i).numComponents;
    end

    % Save found defects if any
    Par_PDF.Prev_Mu_found = cat(1, Par_PDF.Prev_Mu_found, Estim_sol(i).Mu_found);
    Par_PDF.Prev_Sigma_found = cat(3, Par_PDF.Prev_Sigma_found, Estim_sol(i).Sigma_found);

    if Estim_sol(i).flag_done
        n_iter = i; % Save number of iterations achieved
        disp("Algoritmo terminado en " + i + " iteraciones")
    end

    % Saving Data to use it as "Previous data" in next iterations
    Par_PDF.Prev_Data = Estim_sol(i).Data;

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


%% For plotting the trajectory (testing)

% figure(1)
% 
% plot(t_spline, X_e_d_spline)
% grid on
% legend("x_1", "x_2")
% figure(2)
% pcolor(x_1_grid, x_2_grid, ...
%             reshape(Phi_hat_x_next, length(x_2), length(x_1)),...
%             "EdgeColor","none", "FaceColor","interp")
% hold on
% plot(X_e_d_spline(:,1), X_e_d_spline(:,2), "LineWidth", 3, "Color", "black")
% plot(X_e_d_spline(1,1), X_e_d_spline(1,2),...
%     "ksq", "MarkerSize",15, "LineWidth", 3)
% xlim([L_1_l, L_1_u])
% ylim([L_2_l, L_2_u])
% axis equal tight
% hold off
% legend("\hat{\Phi}", "X_e")

% Guardar prueba
% save(sprintf("Results/output_1.mat"), "-regexp", "^(?!(UR_N100)$).");