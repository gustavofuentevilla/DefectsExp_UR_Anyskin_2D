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
Sensor_color = hex2rgb("#fd8d3c");