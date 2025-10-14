%% Parámetros del espacio de búsqueda 
% Omega = [L_1_l, L_1_u] \times [L_2_l, L_2_u]

% Número de dimensiones espaciales
n = 2; 

L_1_l = 0.0;
L_1_u = 0.28;
dx_1 = (L_1_u - L_1_l)/50;

L_2_l = 0.0;
L_2_u = 0.22;
dx_2 = (L_2_u - L_2_l)/50;

% Dimensiones \mathbf{x} = [x_1 x_2]^T
x_1 = (L_1_l:dx_1:L_1_u)';
x_2 = (L_2_l:dx_2:L_2_u)';

% vector de límites inferior y superiores de las dimensiones
L_i_l = [L_1_l, L_2_l];
L_i_u = [L_1_u, L_2_u];

% Longitudes
L_1 = L_1_u - L_1_l;
L_2 = L_2_u - L_2_l;

[x_1_grid, x_2_grid] = meshgrid(x_1, x_2);

% Espacio de búsqueda discretizado
Omega = [reshape(x_1_grid,[],1), reshape(x_2_grid,[],1)];

%% Real PDF (Coins)

% Centros de monedas reales
Mu = [0.1490, 0.04;
      0.232, 0.186;
      0.058, 0.176];

n_def = height(Mu);

% diámetros de las monedas (2 cents of euro, 1 cent of euro)
coinsDiam = [18.75e-3; 16.25e-3; 16.25e-3];

coins_V = eye(2);

coins_D_1 = (coinsDiam(1)/2) * eye(2);
coins_D_2 = (coinsDiam(2)/2) * eye(2);
coins_D_3 = (coinsDiam(3)/2) * eye(2);

coins_S_d_1 = coins_V * coins_D_1 * coins_V' / 3;
coins_S_d_2 = coins_V * coins_D_2 * coins_V' / 3;
coins_S_d_3 = coins_V * coins_D_3 * coins_V' / 3;

Cov_1 = coins_S_d_1 * coins_S_d_1;
Cov_2 = coins_S_d_2 * coins_S_d_2;
Cov_3 = coins_S_d_3 * coins_S_d_3;

Sigma = cat(3, Cov_1, Cov_2, Cov_3);

% Real PDF
gm_dist = gmdistribution(Mu, Sigma);
Phi_x = pdf(gm_dist, Omega);

Sigma_ast_Phi = zeros(size(Sigma));
for j = 1:n_def
    % 3*Standard deviation that represents 99% of data
    Sigma_ast_Phi(:,:,j) = 3*sqrtm(Sigma(:,:,j)); 
end

% plotting
% nbDrawingSeg = 1000;
% tmp_vec = linspace(-pi, pi, nbDrawingSeg)';
% Elipse_Phi = zeros(height(tmp_vec), 2, n_def); % Elipse
% for j = 1:n_def  
%     Elipse_Phi(:,:,j) = [cos(tmp_vec), sin(tmp_vec)] * real(Sigma_ast_Phi(:,:,j)) + repmat(Mu(j,:),nbDrawingSeg,1);
% end
% 
% figure(1)
% pcolor(x_1_grid, x_2_grid, reshape(Phi_x, length(x_2), length(x_1)),...
%        "FaceColor","interp","EdgeColor","none")
% xlim([L_1_l, L_1_u])
% ylim([L_2_l, L_2_u])
% title("Real PDF",'Interpreter','latex')
% xlabel('$x_1$ [m]','Interpreter','latex')
% ylabel('$x_2$ [m]','Interpreter','latex')
% axis equal tight
% grid on
% hold on
% for j = 1:n_def
%     plot(Elipse_Phi(:,1,j), Elipse_Phi(:,2,j), "w", "LineWidth",1.3)
% end
% plot(Mu(:,1), Mu(:,2), ".")
% hold off

%% Uniform PDF as an initial guess

Phi_hat_x_1 = unifpdf(x_1, L_1_l, L_1_u);
Phi_hat_x_2 = unifpdf(x_2, L_2_l, L_2_u);

[Phi_hat_x_1_grid, Phi_hat_x_2_grid] = meshgrid(Phi_hat_x_1, Phi_hat_x_2);
Phi_hat_x = prod([reshape(Phi_hat_x_1_grid,[],1), reshape(Phi_hat_x_2_grid,[],1)], 2);

%% Cálculo de los coeficientes de Fourier para la PDF de referencia

% Coeficientes por dimensión
K = 12;

% Conjunto de valores para k_i
k_1 = (0:K-1)';
k_2 = (0:K-1)';

[k_1_grid, k_2_grid] = meshgrid(k_1, k_2);

% Conjunto de vectores índice
K_cal = [reshape(k_1_grid,1,[]); reshape(k_2_grid,1,[])];

Par_struct.K = K;
Par_struct.n = n;
Par_struct.K_cal = K_cal;
Par_struct.Omega = Omega;
Par_struct.dx_1 = dx_1;
Par_struct.dx_2 = dx_2;
Par_struct.L_i_l = L_i_l;
Par_struct.L_i_u = L_i_u;

[phi_k_reg, f_k_reg, h_k_reg] = FourierCoef_RefPDF(Phi_hat_x, Par_struct);

%% Condiciones Iniciales y parámetros

N = 100; % Número de muestras por iteración
t_f = 10;           %Tiempo final por iteración
T_s = t_f/N;                  % Tiempo de muestreo
t = (0:T_s:t_f)';   %Vector de tiempo por iteración

% Estado inicial z = [z_1; z_2; z_3; z_4] = [x_1; x_1_dot; x_2; x_2_dot]}
z_0 = [0.03; 0; 0.11; 0]; 

%Pre-cálculo de Lambda
p = 2; %norma 2
Lambda_k = (1 + vecnorm(K_cal, p, 1)').^(-(n + 1)/2);

% Vector de tiempo para spline a 100 Hz 
% (Frecuencia de envío de mensajes ROS2 al robot real)
freq_spline = 100;
T_s_spline = 1/freq_spline;
t_spline = (0:T_s_spline:t_f)';

%% Loop for the Search task
n_iter_max = 1;

% sensor uncertainty radius
r_s = 2.36e-2; 

% Registers
z_reg = zeros(N+1, 4, n_iter_max);
u_reg = zeros(N, 2, n_iter_max);
X_e_reg = zeros(N+1, 2, n_iter_max);
X_e_dot_reg = zeros(N+1, 2, n_iter_max);
phi_k_REG = zeros(K^n, 1, n_iter_max);
Phi_hat_x_reg = zeros(height(Omega), 1, n_iter_max + 1);
X_e_spline_reg = zeros(length(t_spline), 2, n_iter_max);
X_e_dot_spline_reg = zeros(length(t_spline), 2, n_iter_max);
u_spline_reg = zeros(length(t_spline) - 1, 2, n_iter_max);
% Datos de (Tiempo - Posición - Mediciones)
Data_t_Xe_V_reg = NaN(4*length(t_spline), 5, n_iter_max);
% V_Xe_reg = zeros(length(t_spline), 1, n_iter_max);

% Initializations
z_act = z_0;
phi_k_act = phi_k_reg;
Phi_hat_x_act = Phi_hat_x;

% Parameters for PDF Estimator
Par_PDF.Omega = Omega;
Par_PDF.dx_1 = dx_1;
Par_PDF.dx_2 = dx_2;
% Range of possible Number of defects to be found
Par_PDF.nbDef_range = [1, n_def + 2]; 

% Threshold definition
Par_PDF.thres_meas = 200;

Par_PDF.Prev_Data = [];
Par_PDF.Prev_numComponents = [];

% Define the dimensions of the registers for the defects found with an
% initial value (these has to be removed at the end)
Par_PDF.Prev_Mu_found = [0, 0];
Par_PDF.Prev_Sigma_found = [0, 0; 0, 0];
NoDataIterCounter = 0;

n_iter = n_iter_max;

Par_PDF.DataEscFact = 1;
% Total variation condition to find a defect
Par_PDF.Thres_Variation = max(coinsDiam) + 2*r_s + 0.001;
% Minimum axes lengths of gaussian elipses (0 = not using this constraint)
Par_PDF.MinAxisLengths = 0; % 0 m.
% Distance needed to consider more than one single defect
Par_PDF.OneClustDistLimit = max(coinsDiam) + 2*r_s + 0.007;
Par_PDF.flag_ExplorationStage = true;

% Parameters definition for the Variation constraint function

% Porcentage of max variation constraint, 
% porcentage of MaxVarCons to match with the first D_KL value
nu_p = 0.35;
% Another way to compute the MaxVarCons is define the number of times of
% Variation Threshold we want to cover per defect, \eta times.
% eta = 6;

% D_KL that matches MaxVarCons
Par_PDF.D_KL_bar_u = []; % computed in estimator 
% Little offset under Variation Threshold for numerical estability
Par_PDF.eps = 0.001;

% Maximum Variation Constraint Computation (2 ways)
Par_PDF.MaxVarCons = nu_p*(L_1 + L_2) + (1 - nu_p)*...
                     (Par_PDF.Thres_Variation - Par_PDF.eps);
% Par_PDF.MaxVarCons = eta*Par_PDF.Thres_Variation;