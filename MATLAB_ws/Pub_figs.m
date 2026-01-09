close all
clear
clc

%N100/2Def/7
load("Results/N150/2Def/output_2.mat")

%% Extracción de datos rales

t_real_reg = cell(1, n_iter);
X_e_real_reg = cell(1, n_iter);
V_dirt_reg = cell(1, n_iter);
V_real_reg = cell(1, n_iter);
T_s_real = zeros(1, n_iter);

for i = 1:n_iter
    % Extract real data
    t_real_reg{i} = Data_t_Xe_V_reg(:,1,i);
    X_e_real_reg{i} = Data_t_Xe_V_reg(:,2:3,i);
    V_dirt_reg{i} = Data_t_Xe_V_reg(:,5,i); % Señal sucia
    V_real_reg{i} = Data_t_Xe_V_reg(:,end,i); % Señal limpia

    idx_nan = isnan(t_real_reg{i});
    % Remove NaNs
    t_real_reg{i}(idx_nan) = [];
    X_e_real_reg{i}(idx_nan, :) = [];
    V_dirt_reg{i}(idx_nan) = [];
    V_real_reg{i}(idx_nan) = [];
    
    % Compute sample time of every iteration
    T_s_real(i) = mean( diff( t_real_reg{i} ) );
end


%% Reconstrucción de distribución empírica y métrica ergódica de:

%  --------- trayectoria Real (PENDIENTE)

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


%% Pre-Procesing for charts
% Concatenación de señales a través de todas las iteraciones

% --------- Concatenación de los datos reales

sizesData = zeros(1, n_iter);
for i = 1:n_iter
    sizesData(i) = size(t_real_reg{i}, 1);
end

t_real_total = t_real_reg{1};
X_e_real_total = X_e_real_reg{1};
V_real_total = V_real_reg{1};
Varepsilon_real_total = Varepsilon_real_reg{1};
for i = 2:n_iter
    t_real_total = cat(1, t_real_total, t_f*(i-1) + t_real_reg{i});
    X_e_real_total = cat(1, X_e_real_total, X_e_real_reg{i});
    V_real_total = cat(1, V_real_total, V_real_reg{i});
    Varepsilon_real_total = cat(1, Varepsilon_real_total, Varepsilon_real_reg{i});
end

nbDrawingSeg = 100;
tmp_vec = linspace(-pi, pi, nbDrawingSeg)';
Elipse_Phi = zeros(height(tmp_vec), 2, n_def); %Elipse
for j = 1:n_def
    Elipse_Phi(:,:,j) = [cos(tmp_vec), sin(tmp_vec)] * ...
                        real(3*sqrtm(Sigma(:,:,j))) + ...
                        repmat(Mu(j,:),nbDrawingSeg,1);
end

n_def_found = size(Sigma_found, 3);

nbDrawingSeg = 100;
tmp_vec = linspace(-pi, pi, nbDrawingSeg)';
stdev_Phi_hat = zeros(size(Sigma_found));
Sigma_ast_Phi_hat = zeros(size(Sigma_found));
Elipse_Phi_hat = zeros(height(tmp_vec), 2, n_def_found); %Elipse
for j = 1:n_def_found
    stdev_Phi_hat(:,:,j) = sqrtm(Sigma_found(:,:,j));
    Sigma_ast_Phi_hat(:,:,j) = 3*stdev_Phi_hat(:,:,j);
    Elipse_Phi_hat(:,:,j) = [cos(tmp_vec), sin(tmp_vec)]* ...
                            real(Sigma_ast_Phi_hat(:,:,j)) +...
                            repmat(Mu_found(j,:), nbDrawingSeg, 1);
end

columnas = 5;
filas = ceil((n_iter + 1)/columnas);

FoundDef_color = hex2rgb("#238b45"); %hex2rgb("#d94801");
NotFoundDef_color = "yellow";
RealDef_color = "black";
Trayectory_color = hex2rgb("#d94801");  %hex2rgb("#045a8d"); %"black";

%% Graficación

fig1h = figure(1);
layout1h = tiledlayout(fig1h, filas, columnas);

for i = 1:n_iter
    SigF_tmp(i).Sigma_found = Estim_sol{i}.Sigma_found;
    MuF_tmp(i).Mu_found = Estim_sol{i}.Mu_found;
end

for i = 1:n_iter

    nexttile(layout1h)

    probmap_ax = pcolor(x_1_grid, x_2_grid, ...
            reshape(Phi_hat_x_reg(:,:,i), length(x_2), length(x_1)),...
            "EdgeColor","none", "FaceColor","interp");
    title("Iteration " + i)
    % xlabel('$x_1$ [m]')
    % ylabel('$x_2$ [m]')
    xtickformat('%.2f')
    ytickformat('%.2f')
    axis equal tight
    xlim([L_1_l, L_1_u])
    ylim([L_2_l, L_2_u])
    grid on
    hold on
    for j = 1:n_def
        realdef_ax(j) = plot(Elipse_Phi(:,1,j), Elipse_Phi(:,2,j),...
                            "-.", "LineWidth", 3,...
                            "Color", RealDef_color);
    end
    plot(Mu(:,1),Mu(:,2),'.','MarkerSize',15, "Color", RealDef_color)
    
% ----Código para graficar los defectos ya encontrados en la i-esima it
    if i > 1
        nbDrawingSeg = 100;
        tmp_vec = linspace(-pi, pi, nbDrawingSeg)';
        Sigma_tmp = cat(3, SigF_tmp(1:i-1).Sigma_found);
        Mu_tmp = cat(1, MuF_tmp(1:i-1).Mu_found);
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
                realdef_ax(j) = plot(Elipse_tmp(:,1,j), Elipse_tmp(:,2,j),...
                                    "-.", "LineWidth", 3,...
                                    "Color", FoundDef_color);
            end
            plot(Mu_tmp(:,1),Mu_tmp(:,2),'.',...
                'MarkerSize',15, "Color", FoundDef_color)
            for j = 1:size(Sigma_tmp, 3)
                F_def_ax(j) = patch(Elipse_tmp(:,1,j), Elipse_tmp(:,2,j),...
                                    FoundDef_color, 'LineWidth', 3,...
                                    'EdgeColor', FoundDef_color,...
                                    "FaceAlpha",0.2);
            end
        end
    end
% ----Fin de código para graficar los defectos encontrados en la i-esima it

    traj_ax = plot(X_e_real_reg{i}(:,1), X_e_real_reg{i}(:,2),...
                    "Color", Trayectory_color,'LineWidth',3);
    traj0_ax = plot(X_e_real_reg{i}(1,1), X_e_real_reg{i}(1,2),'sq', "Color",...
                    Trayectory_color, 'MarkerSize',7,'LineWidth',10);
    
    % lgd = legend([probmap_ax, realdef_ax(1), traj_ax, traj0_ax],...
    %         {"$\hat{\Phi}(\mathbf{x})$",...
    %          "Real" + newline + "defects",...
    %          "$\mathbf{X_e}(t)$",...
    %          "$\mathbf{X_e}(0)$"});
    % lgd.Location = "northeastoutside";

    hold off
end

nexttile(layout1h)

% pcolor(x_1_grid, x_2_grid, reshape(Phi_hat_x_reg(:,:,n_iter+1),...
%        length(x_2), length(x_1)),...
%        "EdgeColor","none", "FaceColor","interp")
title("Result",'Interpreter','latex')
% xlabel('$x_1$ [m]','Interpreter','latex')
% ylabel('$x_2$ [m]','Interpreter','latex')
xtickformat('%.2f')
ytickformat('%.2f')
axis equal
xlim([L_1_l, L_1_u])
ylim([L_2_l, L_2_u])
hold on

%Grafica las elipses de defectos reales
for j = 1:n_def
    R_def_ax(j) = plot(Elipse_Phi(:,1,j), Elipse_Phi(:,2,j), "-.",...
                        "LineWidth", 3, "Color", RealDef_color);
end

%Grafica los centroides
plot(Mu(:,1),Mu(:,2),'.','MarkerSize',15, "Color", RealDef_color)
plot(Mu_found(:,1), Mu_found(:,2), '+', ...
    'LineWidth', 3, 'color', FoundDef_color);
if ~Estim_sol{end}.flag_done 
    plot(Mu_not_found(:,1), Mu_not_found(:,2), '+', ...
        'LineWidth', 3, 'color', NotFoundDef_color);
end
hold off

%Grafica los defectos encontrados (si lo hay)
if n_def_found >= 1
    for j = 1:n_def_found
        F_def_ax(j) = patch(Elipse_Phi_hat(:,1,j), Elipse_Phi_hat(:,2,j), ...
            FoundDef_color,...
            'LineWidth', 3, 'EdgeColor', FoundDef_color, "FaceAlpha",0.2);
    end
end

%Grafica los defectos no encontrados (si los hay)
if ~Estim_sol{end}.flag_done 
        for i = 1:n_def_not_found
            NF_def_ax(i) = patch(Elipse_not_found(:,1,i), Elipse_not_found(:,2,i), ...
                NotFoundDef_color,'LineWidth', 3, 'EdgeColor', ...
                NotFoundDef_color, "FaceAlpha",0.2);
        end
        notfoundplot = 1;
end

% Asignar leyendas
% if ~Estim_sol{end}.flag_done
%     lgd = legend([R_def_ax(1)  F_def_ax(1) NF_def_ax(1)],...
%             {'Real Defects','Found Defects', 'Not Found Defects'});
% else
%     lgd = legend([R_def_ax(1)  F_def_ax(1)],...
%             {"Real" + newline + "defects","Found" + newline + "Defects"});
% end
% 
% lgd.Location = 'northeastoutside';

layout1h.TileSpacing = 'compact';
layout1h.Padding = 'compact';

% Remueve los números del eje Y en las últimas 4 gráficas
layout1h.Children(1).YTick = [];
layout1h.Children(2).YTick = [];
layout1h.Children(3).YTick = [];
layout1h.Children(4).YTick = [];

xlabel(layout1h, '$x_1$ [m]','Interpreter','latex', "FontSize", 22)
ylabel(layout1h, '$x_2$ [m]','Interpreter','latex', "FontSize", 22)

set(findall(fig1h,'-property','Interpreter'),'Interpreter','latex') 
set(findall(fig1h,'-property','TickLabelInterpreter'), ...
    'TickLabelInterpreter','latex')
set(findall(fig1h, "-property", "FontSize"), "FontSize", 22)
colormap(brewermap(15,"-Blues"))

%% Sensor coverage plots

Sensor_color = hex2rgb("#88419d");

fig2h = figure(2);
tiledlayout(fig2h, 4, 6);

nexttile(1, [2, 2])
pcolor(x_1_grid, x_2_grid,...
        reshape(Phi_hat_x(:,1), length(x_2), length(x_1)),...
        "EdgeColor","none","FaceColor","interp")
xlabel('$x_1$ [m]')
ylabel('$x_2$ [m]')
axis equal
xlim([L_1_l, L_1_u])
ylim([L_2_l, L_2_u])
xtickformat('%.2f')
ytickformat('%.2f')

hold on
plot(X_e_reg(:,1,1), X_e_reg(:,2,1),...
        'LineWidth',2, 'Color', 'black')
plot(X_e_reg(1,1,1), X_e_reg(1,2,1),...
        'ksq','MarkerSize',17,'LineWidth',3)

vertcs_x = [-0.02; 0; 0.02] + X_e_reg(:,1,1)';
vertcs_y = [-0.02; 0.023; -0.02] + X_e_reg(:,2,1)';
patch(vertcs_x, vertcs_y, Sensor_color, "FaceAlpha", 0.5, "EdgeColor", "none")

hold off

set(findall(fig2h,'-property','Interpreter'),'Interpreter','latex') 
set(findall(fig2h,'-property','TickLabelInterpreter'), ...
    'TickLabelInterpreter','latex')
set(findall(fig2h, "-property", "FontSize"), "FontSize", 22)
colormap(brewermap(15,"-Blues"))

%% Real data plots
fig4h = figure(4);
tiledlayout(fig4h, 4, 6);

nexttile(1, [1 3])
plot(t_real_total, V_real_total, 'LineWidth', 1.5)
title("Sensor measurements over time",'Interpreter','latex')
xlabel('Time [s]','Interpreter','latex')
ylabel('Magnetic Flux [$\mu~T$]','Interpreter','latex')
hold on
yline(thres_meas, "-", "$\Gamma_V$", "LineWidth", 2.5);
hold off
grid on
legend("$V(t)$", "Threshold")
xlim([0, 40])

nexttile(7, [1 3])
plot(t_real_total, X_e_real_total, 'LineWidth', 1.5)
title("Real Position States Measurements",'Interpreter','latex')
xlabel('Time [s]','Interpreter','latex')
ylabel('Position [m]','Interpreter','latex')
legend('$x_1$', '$x_2$','Interpreter','latex')
grid on
xlim([0, 40])

nexttile(13, [1 3])
plot(t_real_total, Varepsilon_real_total, "k-", "LineWidth",3)
% title("Real Ergodic Metric")
xlabel('Time [s]')
ylabel('$\varepsilon \left( \mathbf{X_e}(t), \Phi(\mathbf{x}) \right) $')
grid on
xlim([0, 40])

set(findall(fig4h,'-property','Interpreter'),'Interpreter','latex') 
set(findall(fig4h,'-property','TickLabelInterpreter'), ...
    'TickLabelInterpreter','latex')
set(findall(fig4h, "-property", "FontSize"), "FontSize", 20)

%% Preprocessing figs

%load("Results/N150/2Def/output_2.mat"); it = 3, 4;

it = 3;

def_signal = Preprocess_struct(it).def_signal;
movingAvgAbsDerivative = Preprocess_struct(it).movingAvgAbsDerivative;
v_Norm = sqrt(sum(X_e_dot_reg(:,:,it).^2, 2));
v_Norm_spline = Preprocess_struct(it).v_Norm_spline;
abs_derivative_signal = Preprocess_struct(it).abs_derivative_signal;


figh = figure;
tiledlayout(figh, 4, 6);

nexttile(1, [1 3])
plot(t_real_reg{it}, V_dirt_reg{it}, "LineWidth", 3)
hold on
plot(t_real_reg{it}, V_real_reg{it}, 'LineWidth', 3, "Color", "black");
yline(thres_meas, "LineWidth", 3, "Color", "red",...
    "Label", "Threshold, $\Gamma_{V}$")
% title('Signal')
xlabel('Time [s]')
ylabel('Magnetic Flux [$\mu~T$]')
% legend('Original Signal', 'Clean Signal, $V$');
grid on
hold off
xlim([0, 15])

nexttile(7, [1 3])
plot(t_real_reg{it}, abs_derivative_signal, 'LineWidth', 3);
% title('abs(Derivative)');
xlabel('Time [s]');
% ylabel('abs(D)');
grid on
hold on
plot(t_real_reg{it}, movingAvgAbsDerivative,...
    'LineWidth', 3, "Color", "black");
yline(thres_absD, "Color", "red","LineWidth",3,...
    "Label", "Threshold, $\Gamma_{\rho}$")
hold off
% legend("abs$(\partial V)$",...
%     "Moving average filter"+newline+"applied to abs$(\partial V)$")
xlim([0, 15])

nexttile(13, [1 3])
plot(t, v_Norm, "LineWidth", 3, "Color", "black")
% title("Planned Linear Velocity Norm")
xlabel("Time [s]")
ylabel("velocity [m/s]")
grid on
hold on
yline(thres_vel, "LineWidth",3, "Color", "red",...
    "Label", "Threshold, $\Gamma_{|v|}$")
hold off
% legend("Planned linear velocity, $|v|$")
xlim([0, 15])

nexttile(19, [1 3])
plot(t_real_reg{it}, def_signal, "LineWidth", 3, "Color", "black")
xlabel('Time [s]')
ylabel('Magnetic Flux [$\mu~T$]')
% legend('filtered data, $V_k$')
grid on
xlim([0, 15])

nexttile(4, [2 2])
for j = 1:n_def
    plot(Elipse_Phi(:,1,j), Elipse_Phi(:,2,j), "-.",...
                        "LineWidth", 3, "Color", RealDef_color);
    hold on
end
%Grafica los centroides
plot(Mu(:,1),Mu(:,2),'.','MarkerSize',15, "Color", RealDef_color)
patch([X_e_real_reg{it}(:,1); NaN], [X_e_real_reg{it}(:,2); NaN],...
      [V_dirt_reg{it}; NaN], 'EdgeColor','interp',"LineWidth", 3)
cb = colorbar;
cb.Label.String = '$V(t)$';
cb.Label.Interpreter = "latex";
xlabel('$x_1$')
ylabel('$x_2$')
axis equal
xlim([0, 0.28])
ylim([0, 0.2])
plot(X_e_real_reg{it}(1,1), X_e_real_reg{it}(1,2), "bsq",...
    "MarkerSize",16, "LineWidth",3)
hold off
xtickformat('%.2f')
ytickformat('%.2f')
% legend('$X_e(t)$', "$X_e(0)$")

set(findall(figh,'-property','Interpreter'),'Interpreter','latex') 
set(findall(figh,'-property','TickLabelInterpreter'), ...
    'TickLabelInterpreter','latex')
set(findall(figh, "-property", "FontSize"), "FontSize", 18)

%% Threshold fig

%load("Results/N100/2Def/output_2.mat")

it = 1;

figh = figure;
tiledlayout(figh, 4, 6);

init1 = 0.48;
fin1 = 4.03; % Define the end of the threshold region
init2 = 4.47;
fin2 = 5.62;
init3 = 6.43;
fin3 = 10;

color_region = [0.6, 0.4, 0.9];

nexttile(1, [1 3])
plot(t_real_reg{it}, V_real_reg{it}, 'LineWidth', 3, "Color", "black");
xlabel('Time [s]')
ylabel('Magnetic Flux [$\mu~T$]')
% legend('Clean Signal, $V$');
grid on
hold on
patch([init1, fin1, fin1, init1], [0, 0, 150, 150],...
    color_region, "FaceAlpha", 0.3, "EdgeColor", "none")
patch([init2, fin2, fin2, init2], [0, 0, max(ylim), max(ylim)],...
    color_region, "FaceAlpha", 0.3, "EdgeColor", "none")
patch([init3, fin3, fin3, init3], [0, 0, max(ylim), max(ylim)],...
    color_region, "FaceAlpha", 0.3, "EdgeColor", "none")
hold off
xlim([0, 10])

set(findall(figh,'-property','Interpreter'),'Interpreter','latex') 
set(findall(figh,'-property','TickLabelInterpreter'), ...
    'TickLabelInterpreter','latex')
set(findall(figh, "-property", "FontSize"), "FontSize", 18)

%% Zero defect case
% Run this section alone, not all the code above

% close all
% clear
% clc
% 
% load("Results/N100/0Def/output_1.mat")
% 
% t_real_reg = cell(1, n_iter);
% X_e_real_reg = cell(1, n_iter);
% V_dirt_reg = cell(1, n_iter);
% V_real_reg = cell(1, n_iter);
% T_s_real = zeros(1, n_iter);
% 
% for i = 1:n_iter
%     % Extract real data
%     t_real_reg{i} = Data_t_Xe_V_reg(:,1,i);
%     X_e_real_reg{i} = Data_t_Xe_V_reg(:,2:3,i);
%     V_dirt_reg{i} = Data_t_Xe_V_reg(:,5,i); % Señal sucia
%     V_real_reg{i} = Data_t_Xe_V_reg(:,end,i); % Señal limpia
% 
%     idx_nan = isnan(t_real_reg{i});
%     % Remove NaNs
%     t_real_reg{i}(idx_nan) = [];
%     X_e_real_reg{i}(idx_nan, :) = [];
%     V_dirt_reg{i}(idx_nan) = [];
%     V_real_reg{i}(idx_nan) = [];
% 
%     % Compute sample time of every iteration
%     T_s_real(i) = mean( diff( t_real_reg{i} ) );
% end
% 
% 
% % Reconstrucción de distribución empírica y métrica ergódica de:
% 
% %  --------- trayectoria Real
% 
% Varepsilon_real_reg = cell(1, n_iter);
% C_x_real_reg = cell(1, n_iter);
% 
% for r = 1:n_iter
%     c_k_real = zeros(K^n, 1);
%     C_x_real = zeros(height(Omega), 1);
%     for i = 1:length(t_real_reg{r})
% 
%         % Compute Fourier Functions and coefficients on the real position
%         f_k_real = prod(cos( K_cal'.*pi.*(X_e_real_reg{r}(i,:) - L_i_l)./...
%                         (L_i_u - L_i_l) ), 2) ./ h_k_reg ;
%         c_k_real = c_k_real + (f_k_real*T_s_real(r))/(t_f) ;
% 
%         % Ergodic metric
%         Varepsilon_real = sum( Lambda_k .* (c_k_real - phi_k_REG(:,:,r)).^2 );
% 
%         % Empirical distribution reconstruction
%         C_x_real_i = zeros(height(Omega), 1);
%         for j = 1:K^n
%             C_x_real_i = C_x_real_i + c_k_real(j)*f_k_reg(:,j);
%         end
% 
%         % Se suman todas las distribuciones generadas en cada muestra
%         C_x_real = C_x_real + C_x_real_i;
% 
%         % Se registra
%         C_x_real_reg{r}(:,i) = C_x_real;
%         Varepsilon_real_reg{r}(i,:) = Varepsilon_real;
% 
%     end
% 
% end
% 
% % --------- Concatenación de los datos reales
% 
% sizesData = zeros(1, n_iter);
% for i = 1:n_iter
%     sizesData(i) = size(t_real_reg{i}, 1);
% end
% 
% t_real_total = t_real_reg{1};
% X_e_real_total = X_e_real_reg{1};
% V_real_total = V_real_reg{1};
% Varepsilon_real_total = Varepsilon_real_reg{1};
% for i = 2:n_iter
%     t_real_total = cat(1, t_real_total, t_f*(i-1) + t_real_reg{i});
%     X_e_real_total = cat(1, X_e_real_total, X_e_real_reg{i});
%     V_real_total = cat(1, V_real_total, V_real_reg{i});
%     Varepsilon_real_total = cat(1, Varepsilon_real_total, Varepsilon_real_reg{i});
% end
% 
% % ---------------------- Graficación
% 
% Trayectory_color = hex2rgb("#d94801");
% 
% columnas = 5;
% filas = ceil((n_iter + 1)/columnas);
% 
% fig1h = figure(1);
% % figure("units", "centimeters", "Position", [5, 5, Width, Height])
% layout1h = tiledlayout(fig1h, filas, columnas);
% 
% for i = 1:n_iter
% 
%     nexttile(layout1h)
% 
%     probmap_ax = pcolor(x_1_grid, x_2_grid, ...
%             reshape(Phi_hat_x_reg(:,:,i), length(x_2), length(x_1)),...
%             "EdgeColor","none", "FaceColor","interp");
%     title("Iteration " + i)
%     xtickformat('%.2f')
%     ytickformat('%.2f')
%     axis equal tight
%     xlim([L_1_l, L_1_u])
%     ylim([L_2_l, L_2_u])
%     grid on
%     hold on
%     traj_ax = plot(X_e_real_reg{i}(:,1), X_e_real_reg{i}(:,2),...
%                     "Color", Trayectory_color,'LineWidth',3);
%     traj0_ax = plot(X_e_real_reg{i}(1,1), X_e_real_reg{i}(1,2),'sq', "Color",...
%                     Trayectory_color, 'MarkerSize',7,'LineWidth',10);
%     hold off
% 
%     if i == 2
%         xlabel('$x_1$ [m]','Interpreter','latex', "FontSize", 22);
%     end
% 
% end
% 
% nexttile(layout1h)
% 
% title("Result",'Interpreter','latex')
% xtickformat('%.2f')
% ytickformat('%.2f')
% axis equal
% xlim([L_1_l, L_1_u])
% ylim([L_2_l, L_2_u])
% 
% layout1h.TileSpacing = 'compact';
% layout1h.Padding = 'compact';
% 
% % Remueve los números del eje Y en las últimas 2 gráficas
% layout1h.Children(1).YTick = [];
% layout1h.Children(2).YTick = [];
% 
% % xlabel(layout1h, '$x_1$ [m]','Interpreter','latex', "FontSize", 22)
% ylabel(layout1h, '$x_2$ [m]','Interpreter','latex', "FontSize", 22)
% 
% set(findall(fig1h,'-property','Interpreter'),'Interpreter','latex') 
% set(findall(fig1h,'-property','TickLabelInterpreter'), ...
%     'TickLabelInterpreter','latex')
% set(findall(fig1h, "-property", "FontSize"), "FontSize", 22)
% colormap(brewermap(15,"-Blues"))
% 
% % ---------------------- Signal charts
% figh = figure;
% tiledlayout(figh, 4, 6);
% 
% nexttile(1, [1 3])
% plot(t_real_total, V_real_total, "k-", 'LineWidth', 3)
% % title("Sensor measurements over time",'Interpreter','latex')
% xlabel('Time [s]','Interpreter','latex')
% ylabel('Magnetic Flux [$\mu~T$]','Interpreter','latex')
% hold on
% yline(thres_meas, "r-", "Threshold, $\Gamma_V$", "LineWidth", 3);
% hold off
% grid on
% % legend("$V(t)$", "Threshold")
% xlim([0, 20])
% ylim([10, 75])
% 
% nexttile(7, [1 3])
% plot(t_real_total, Varepsilon_real_total, "k-", "LineWidth",3)
% % title("Real Ergodic Metric")
% xlabel('Time [s]')
% ylabel('$\varepsilon \left( \mathbf{X_e}(t), \Phi(\mathbf{x}) \right) $')
% grid on
% xlim([0, 20])
% 
% set(findall(figh,'-property','Interpreter'),'Interpreter','latex') 
% set(findall(figh,'-property','TickLabelInterpreter'), ...
%     'TickLabelInterpreter','latex')
% set(findall(figh, "-property", "FontSize"), "FontSize", 20)