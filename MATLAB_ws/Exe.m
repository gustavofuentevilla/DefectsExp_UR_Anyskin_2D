close all
% clear
clearvars -except UR_N100
clc

%% Include and create custom messages

folderPath = fullfile(pwd, "custom");
ros2genmsg(folderPath)

%% Initializations

Initializations

%% Loop

% for i = 1:n_iter_max
i = 1;
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
    
    % AQUI ME QUEDEEEEE XD
    % Guardar trayectoria en un archivo
    ErgodicTraj = [t_spline, X_e_d_spline];
    T = array2table(ErgodicTraj, 'VariableNames', {'Tiempo', 'x_ee', 'y_ee'});
    writetable(T, "Trayectorias/trayectoria_" + i + ".csv")

    % Ejecutar trayectoria en robot real y recolectar información del
    % sensor



    
    % Measurement along the trajectory, V_Xe
    % Upsilon = a + b*pdf(gm_dist, X_e_spline); %Real PDF
    % delta = c*randn(n_points, 1); %Gaussian Noise with Variance c^2
    % V_Xe = Upsilon + delta;
    % 
    % Par_PDF.thres_meas = a + max(delta);

    %% Registers
%     z_reg(:,:,i) = Z;
%     u_reg(:,:,i) = U;
%     X_e_reg(:,:,i) = X_e_d;
%     X_e_dot_reg(:,:,i) = X_e_d_dot;
%     phi_k_REG(:,:,i) = phi_k_reg;
%     Phi_hat_x_reg(:,:,i) = Phi_hat_x_act;
%     X_e_spline_reg(:,:,i) = X_e_d_spline;
%     X_e_dot_spline_reg(:,:,i) = X_e_dot_spline;
%     u_spline_reg(:,:,i) = u_spline;
%     Data_t_Xe_V_reg(:,:,i) = Data_t_Xe_V;
%     % V_Xe_reg(:,:,i) = V_Xe;
% 
%     % PDF Estimation
%     Par_PDF.iteration = i;
%     Par_PDF.Prev_Phi_hat_x = Phi_hat_x_act;
%     [Phi_hat_x_next, Estim_sol(i)] = PDF_Estimator(X_e_d_spline, V_Xe, Par_PDF);
% 
%     % Update Iterations Counter where No data hav been found
%     NoDataIterCounter = NoDataIterCounter + Estim_sol(i).flag_NoData;
% 
%     if NoDataIterCounter == 2
%         n_iter = i;
%         break;
%     end
% 
%     % Save D_KL from first iteration to set the exploration function
%     if i == 1
%         Par_PDF.D_KL_bar_u = Estim_sol(i).D_KL;
%     end
% 
%     % Detect the falling edge of Exploration flag termination
%     if i > 1
%         falling_edge = (Estim_sol(i).flag_ExplorationStage - ...
%                         Estim_sol(i-1).flag_ExplorationStage) == -1;
%     else
%         falling_edge = false;
%     end
%     % Save the Exploration flag (turned off) and the number of components 
%     if falling_edge
%         Par_PDF.flag_ExplorationStage = Estim_sol(i).flag_ExplorationStage;
%         Par_PDF.Prev_numComponents = Estim_sol(i).numComponents;
%     end
% 
%     % Save found defects if any
%     Par_PDF.Prev_Mu_found = cat(1, Par_PDF.Prev_Mu_found, Estim_sol(i).Mu_found);
%     Par_PDF.Prev_Sigma_found = cat(3, Par_PDF.Prev_Sigma_found, Estim_sol(i).Sigma_found);
% 
%     if Estim_sol(i).flag_done
%         n_iter = i; % Save number of iterations achieved
%         break;
%     end
% 
%     % Saving Data to use it as "Previous data" in next iterations
%     Par_PDF.Prev_Data = Estim_sol(i).Data;
%     % Par_PDF.Prev_Priors = Estim_sol(i).Priors;
%     % Par_PDF.Prev_Mu = Estim_sol(i).Mu;
%     % Par_PDF.Prev_Sigma = Estim_sol(i).Sigma;
%     % Par_PDF.Prev_Sigma_a = Estim_sol(i).Sigma_a;
% 
%     % Compute new Fourier coefficients for \hat{Phi}(x)
%     [phi_k_reg, ~, ~] = FourierCoef_RefPDF(Phi_hat_x_next, Par_struct);
% 
%     % Update parameters for next iteration
%     z_act = Z(end,:)';           % Initial condition for state
%     phi_k_act = phi_k_reg;      % New target coefficients
%     Phi_hat_x_act = Phi_hat_x_next;
% 
% end

%% Remove the initial value (zero values) for defects found
% Mu_found = Par_PDF.Prev_Mu_found(2:end, :);
% Sigma_found = Par_PDF.Prev_Sigma_found(:,:,2:end);



