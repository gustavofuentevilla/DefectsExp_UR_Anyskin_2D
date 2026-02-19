close all
clear
clc

%% Load data
load("Results/N100/2Def/output_7.mat")

%% Preliminary computations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Extracción de datos rales

t_real_reg = cell(1, n_iter);
X_e_real_reg = cell(1, n_iter);
V_real_reg = cell(1, n_iter);
T_s_real = zeros(1, n_iter);

for i = 1:n_iter
    % Extract real data
    t_real_reg{i} = Data_t_Xe_V_reg(:,1,i);
    X_e_real_reg{i} = Data_t_Xe_V_reg(:,2:3,i);
    V_real_reg{i} = Data_t_Xe_V_reg(:,end-1,i); % Señal original

    idx_nan = isnan(t_real_reg{i});
    % Remove NaNs
    t_real_reg{i}(idx_nan) = [];
    X_e_real_reg{i}(idx_nan, :) = [];
    V_real_reg{i}(idx_nan) = [];
    
    % Compute sample time of every iteration
    T_s_real(i) = mean( diff( t_real_reg{i} ) );
end

% Empirical distribution and ergodic metric computation
%  --------- trayectoria Real

Varepsilon_real_reg = cell(1, n_iter);
C_x_real_reg = cell(1, n_iter);

for r = 1:n_iter
    c_k_real = zeros(K^n, 1);
    C_x_real = zeros(height(Omega), 1);
    for i = 1:length(t_real_reg{r})

        % Compute Fourier Functions and coefficients on the real position
        f_k_real = prod(cos( K_cal'.*pi.*(X_e_real_reg{r}(i,:) - L_i_l)./...
                        (L_i_u - L_i_l) ), 2) ./ h_k_reg ;
        c_k_real = c_k_real + (f_k_real*T_s_real(r))/(t_f) ;

        % Ergodic metric
        Varepsilon_real = sum( Lambda_k .* (c_k_real - phi_k_REG(:,:,r)).^2 );

        % Empirical distribution reconstruction
        C_x_real_i = zeros(height(Omega), 1);
        for j = 1:K^n
            C_x_real_i = C_x_real_i + c_k_real(j)*f_k_reg(:,j);
        end

        % Se suman todas las distribuciones generadas en cada muestra
        C_x_real = C_x_real + C_x_real_i;

        % Se registra
        C_x_real_reg{r}(:,i) = C_x_real;
        Varepsilon_real_reg{r}(i,:) = Varepsilon_real;

    end

end

% Real defect ellipses
nbDrawingSeg = 100;
tmp_vec = linspace(-pi, pi, nbDrawingSeg)';
Elipse_Phi = zeros(height(tmp_vec), 2, n_def);
for j = 1:n_def
    Elipse_Phi(:,:,j) = [cos(tmp_vec), sin(tmp_vec)] *...
        real(Sigma_ast_Phi(:,:,j)) + repmat(Mu(j,:),nbDrawingSeg,1);
end

% Si el algoritmo no pudo encontrar todos los defectos, se calculan las
% elipses de las estimaciones de los defectos no encontrados
if ~Estim_sol{end}.flag_done 
    Sigma_not_found = Estim_sol{end}.GMModel.Sigma;
    Mu_not_found = Estim_sol{end}.GMModel.mu;
    n_def_not_found = size(Sigma_not_found, 3);

    SD_notfound = zeros(size(Sigma_not_found));
    Sigma_ast_DefNotFound = zeros(size(Sigma_not_found));
    Ellipse_DefNotFound = zeros(height(tmp_vec), 2, n_def_not_found); %Elipse
    for r = 1:n_def_not_found
        SD_notfound(:,:,r) = sqrtm(Sigma_not_found(:,:,r));
        Sigma_ast_DefNotFound(:,:,r) = 3*SD_notfound(:,:,r);
        Ellipse_DefNotFound(:,:,r) = [cos(tmp_vec), sin(tmp_vec)] * real(Sigma_ast_DefNotFound(:,:,r)) +...
            repmat(Mu_not_found(r,:), nbDrawingSeg, 1);
    end
end

% Puntos para generar la geometría del sensor tomando el centro como (0,0)
P_s = [-9.11, -20;
       -11.8, -19.7;
       -14.34, -18.81;
       -16.62, -17.38;
       -18.53, -15.47;
       -19.98, -13.19;
       -20.87, -10.65;
       -21.18, -7.97;
       -20.9, -5.28;
       -20.01, -2.73;
       -10.66, 16.89;
       -8.83, 19.65;
       -6.30, 21.80;
       -3.28, 23.16;
       0, 23.62;
       3.28, 23.16;
       6.30, 21.80;
       8.83, 19.65;
       10.66, 16.89;
       20.01, -2.73;
       20.9, -5.28;
       21.18, -7.97;
       20.87, -10.65;
       19.98, -13.19;
       18.53, -15.47;
       16.62, -17.38;
       14.34, -18.81;
       11.8, -19.7;
       9.11, -20]*1e-3;

% Colores
FoundDef_color = hex2rgb("#238b45");
NotFoundDef_color = "yellow";
RealDef_color = "black";
Trayectory_color = hex2rgb("#d94801");
Sensor_color = hex2rgb("#fd8d3c");

%% First Frame

it = 2;

figh = figure;
figh.Units = "centimeters";
figh.Position = [5, 5, 50, 50];
figh.Units = "pixels";

layouth = tiledlayout(figh, 4, 3);
colormap(brewermap(15,"-Blues"))

title(layouth, "Iteration " + it,...
"interpreter", "latex", "FontSize", 30)

% %%%%%%%%%%%%%%% Gráfica en el plano %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plane_tile = nexttile(layouth, 1, [2, 3]);

Est_PDF_plot = pcolor(x_1_grid, x_2_grid,...
                reshape(Phi_hat_x_reg(:,:,it), length(x_2), length(x_1)),...
                "FaceColor","interp","EdgeColor","none");
xlabel('$x_1$ [m]')
ylabel('$x_2$ [m]')
xtickformat('%.2f')
ytickformat('%.2f')
axis equal
axis manual
xlim([L_1_l, L_1_u])
ylim([L_2_l, L_2_u])
grid on
hold on
%Grafica las elipses de defectos reales
for j = 1:n_def
    RealDef_plot = plot(Elipse_Phi(:,1,j), Elipse_Phi(:,2,j),...
                        "-.", "Color", RealDef_color, "LineWidth", 3);
end
%Grafica los centroides
plot(Mu(:,1),Mu(:,2),'.',"Color", RealDef_color,'MarkerSize',15)

% Gráfica de defectos ya encontrados
for i = 1:n_iter
    SigF_tmp(i).Sigma_found = Estim_sol{i}.Sigma_found;
    MuF_tmp(i).Mu_found = Estim_sol{i}.Mu_found;
end

if it > 1
    nbDrawingSeg = 100;
    tmp_vec = linspace(-pi, pi, nbDrawingSeg)';
    Sigma_tmp = cat(3, SigF_tmp(1:it-1).Sigma_found);
    Mu_tmp = cat(1, MuF_tmp(1:it-1).Mu_found);
    if ~isempty(Sigma_tmp)
        stdev_tmp = zeros(size(Sigma_tmp));
        Sigma_ast_tmp = zeros(size(Sigma_tmp));
        Elipse_tmp = zeros(height(tmp_vec), 2, size(Sigma_tmp, 3));
        for j = 1:size(Sigma_tmp, 3)
            stdev_tmp(:,:,j) = sqrtm(Sigma_tmp(:,:,j));
            Sigma_ast_tmp(:,:,j) = 3*stdev_tmp(:,:,j);
            Elipse_tmp(:,:,j) = [cos(tmp_vec), sin(tmp_vec)]* ...
                                    real(Sigma_ast_tmp(:,:,j)) +...
                                    repmat(Mu_tmp(j,:), nbDrawingSeg, 1);
        end
        
        for j = 1:size(Sigma_tmp, 3)
            F_def_ax = patch(Elipse_tmp(:,1,j), Elipse_tmp(:,2,j),...
                                FoundDef_color, 'LineWidth', 3,...
                                'EdgeColor', FoundDef_color,...
                                "FaceAlpha",0.2);
        end
        plot(Mu_tmp(:,1),Mu_tmp(:,2),'.',...
            'MarkerSize',15, "Color", FoundDef_color)

    end
end

% Gráficas de la geometría del sensor
vertcs_x = P_s(:,1) + X_e_real_reg{it}(1,1)';
vertcs_y = P_s(:,2) + X_e_real_reg{it}(1,2)';
coverage_plot = patch(vertcs_x, vertcs_y, Sensor_color,...
                    "FaceAlpha", 0.025, "EdgeColor", "none");
sensor_plot = patch(vertcs_x, vertcs_y, Sensor_color,...
                    "FaceAlpha", 0.4, "EdgeColor", "none");

% Gráficas de la trayectoria
trayectoria_plot = plot(X_e_real_reg{it}(1,1), X_e_real_reg{it}(1,2),...
                        "Color", Trayectory_color, 'LineWidth',3);
X_e_0_plot = plot(X_e_real_reg{it}(1,1), X_e_real_reg{it}(1,2),'sq',...
                "Color", Trayectory_color,'MarkerSize',10,'LineWidth',10);
X_e_act_plot = plot(X_e_real_reg{it}(1,1), X_e_real_reg{it}(1,2),"o",...
                    "Color", Trayectory_color,'MarkerSize',10,'LineWidth',3);
hold off

% Lógica para leyendas en el plano
flag_legend = false;
if it > 1 
    if ~isempty(Sigma_tmp)
        lgd = legend([Est_PDF_plot, RealDef_plot, trayectoria_plot,...
               X_e_0_plot, sensor_plot, F_def_ax],...
               {"PDF Estimation, $\hat{\Phi}(\mathbf{x})$",...
               "Real defects",...
               "$\mathbf{X_e}(t)$",...
               "$\mathbf{X_e}(0)$",...
               "Sensor footprint",...
               'Found Defects'},...
               'Location','northeastoutside');
        flag_legend = true;
    else
        lgd = legend([Est_PDF_plot, RealDef_plot, trayectoria_plot,...
                X_e_0_plot, sensor_plot],...
               {"PDF Estimation, $\hat{\Phi}(\mathbf{x})$",...
               "Real defects",...
               "$\mathbf{X_e}(t)$",...
               "$\mathbf{X_e}(0)$",...
               "Sensor footprint"},...
               'Location','northeastoutside');
    end
else
lgd = legend([Est_PDF_plot, RealDef_plot, trayectoria_plot,...
            X_e_0_plot, sensor_plot],...
           {"PDF Estimation, $\hat{\Phi}(\mathbf{x})$",...
           "Real defects",...
           "$\mathbf{X_e}(t)$",...
           "$\mathbf{X_e}(0)$",...
           "Sensor footprint"},...
           'Location','northeastoutside');
end

lgd.AutoUpdate = "off";

tiempo_act = t_real_reg{it}(1);
title(plane_tile,"Time: " + num2str(tiempo_act, "%.2f") + " sec",...
        "interpreter", "latex", "FontSize", 30)

% Gráficas de las señales %%%%%%%%%%%%%%%%%%%%
signal_tile = nexttile(layouth, 7, [1, 3]);
signal_plot = plot(t_real_reg{it}(1), V_real_reg{it}(1),...
                   "k-", 'LineWidth', 2);
title("Sensor Signal")
xlabel('Time [s]')
ylabel('Magnetic Flux [$\mu~T$]')
% hold on
% yline(thres_meas, "-", "$\Gamma_V$", "LineWidth", 2.5);
% hold off
grid on
xlim([0, 10])

epsilon_tile = nexttile(layouth, 10, [1, 3]);
epsilon_plot = plot(t_real_reg{it}(1), Varepsilon_real_reg{it}(1),...
                    "k-", "LineWidth",2);
title("Real Ergodic Metric")
xlabel('Time [s]')
ylabel('$\varepsilon \left( \mathbf{X_e}(t), \Phi(\mathbf{x}) \right) $')
grid on
xlim([0, 10])

% %%%%%%%%%%%%%% Setting general plot properties %%%%%%%%%%%%%%%%%%%%%%%%%%
set(findall(figh,'-property','Interpreter'),'Interpreter','latex') 
set(findall(figh,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
set(findall(figh, "-property", "FontSize"), "FontSize", 22)

%% Eliminar datos para tener menos frames

for eliminar_datos = 1:1 %Elimina la mitad de datos en cada ciclo
    N_steps = length(t_real_reg{it});
    idx_borrar = logical(mod(0:N_steps-1, 2));
    
    t_real_reg{it}(idx_borrar) = [];
    X_e_real_reg{it}(idx_borrar,:) = [];
    V_real_reg{it}(idx_borrar) = [];
    Varepsilon_real_reg{it}(idx_borrar) = [];
end

%% %%%%%%%%%%%%%% Frames Pre-allocation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% bias para repetir último frame
bias = 2; 

MovieVector(length(t_real_reg{it}) + bias) = struct("cdata", [], "colormap", []);

%% Loop

% Guardar primer frame 
MovieVector(1) = getframe(figh);

for i = 2:length(t_real_reg{it})
    % Delay (if needed)
    % pause(0.0001)

    % Update title to have the evolution of Time
    tiempo_act = t_real_reg{it}(i);
    title(plane_tile,"Time: " + num2str(tiempo_act, "%.2f") + " sec",...
        "interpreter", "latex", "FontSize", 15)

    % Actualizar el coverage
    vertcs_x = P_s(:,1) + X_e_real_reg{it}(1:i,1)';
    vertcs_y = P_s(:,2) + X_e_real_reg{it}(1:i,2)';
    coverage_plot.XData = vertcs_x;
    coverage_plot.YData = vertcs_y;

    % Actualizar posición del sensor
    vertcs_x = P_s(:,1) + X_e_real_reg{it}(i,1)';
    vertcs_y = P_s(:,2) + X_e_real_reg{it}(i,2)';
    sensor_plot.XData = vertcs_x;
    sensor_plot.YData = vertcs_y;

    % Actualizar trayectoria y señales
    trayectoria_plot.XData(end + 1) = X_e_real_reg{it}(i,1);
    trayectoria_plot.YData(end + 1) = X_e_real_reg{it}(i,2);
    signal_plot.XData(end + 1) = t_real_reg{it}(i);
    signal_plot.YData(end + 1) = V_real_reg{it}(i);
    epsilon_plot.XData(end + 1) = t_real_reg{it}(i);
    epsilon_plot.YData(end + 1) = Varepsilon_real_reg{it}(i);
    
    % Actualizar la posición actual
    X_e_act_plot.XData = X_e_real_reg{it}(i,1);
    X_e_act_plot.YData = X_e_real_reg{it}(i,2);

    % Graficar estimación de defectos encontrados en la iteración actual
    % (en el último frame de la iteración actual)
    if ~isempty(Estim_sol{it}.Mu_found) && (i == length(t_real_reg{it}))
        n_def_found = size(Estim_sol{it}.Sigma_found, 3);
        nbDrawingSeg = 100;
        tmp_vec = linspace(-pi, pi, nbDrawingSeg)';
        SD_DefFound = zeros(size(Estim_sol{it}.Sigma_found));
        Sigma_ast_DefFound = zeros(size(Estim_sol{it}.Sigma_found));
        Ellipse_DefFound = zeros(height(tmp_vec), 2, n_def_found);
        for r = 1:n_def_found
            SD_DefFound(:,:,r) = sqrtm(Estim_sol{it}.Sigma_found(:,:,r));
            Sigma_ast_DefFound(:,:,r) = 3*SD_DefFound(:,:,r);
            Ellipse_DefFound(:,:,r) = [cos(tmp_vec), sin(tmp_vec)] * real(Sigma_ast_DefFound(:,:,r)) +...
                    repmat(Estim_sol{it}.Mu_found(r,:), nbDrawingSeg, 1);
        end
        figh.CurrentAxes = plane_tile;
        hold on
        for r = 1:n_def_found
            F_def_ax = patch(Ellipse_DefFound(:,1,r), Ellipse_DefFound(:,2,r), FoundDef_color,...
                    'LineWidth', 3, 'EdgeColor', FoundDef_color, "FaceAlpha",0.2);
        end
        plot(MuF_tmp(it).Mu_found(:,1), MuF_tmp(it).Mu_found(:,2),'.',...
            'MarkerSize',15, "Color", FoundDef_color)
        hold off
        if ~flag_legend
            legend([Est_PDF_plot, RealDef_plot, trayectoria_plot,...
                    X_e_0_plot, sensor_plot, F_def_ax],...
                   {"PDF Estimation, $\hat{\Phi}(\mathbf{x})$",...
                   "Real defects",...
                   "$\mathbf{X_e}(t)$",...
                   "$\mathbf{X_e}(0)$",...
                   "Sensor footprint",...
                   "Found Defects"},...
                   'Location','northeastoutside')
        end
    end
    
    % Guardar frames
    MovieVector(i) = getframe(figh);

end

for j = 1:bias
    MovieVector(length(t_real_reg{it}) + j) = getframe(figh);
end

% Para reproducir "n" número de veces vez (n = 1)
% movie(MovieVector, 1, 60)

%% Compute frame rate to match real-time
f_rate = round(length(t_real_reg{it}) / t_real_reg{it}(end));

%% Crear y guardar animación

% *Solution for "All 'cdata' fields in FRAMES must be the same size" error
% *Use the following custom function to resize
MovieVector = MakeMovieVectorFramesSameSize(MovieVector);

% *Save animation
myWriter = VideoWriter("Video/Animacion_it_" + it, "Motion JPEG AVI");
myWriter.FrameRate = f_rate;
myWriter.Quality = 95;
open(myWriter);
writeVideo(myWriter, MovieVector);
close(myWriter);