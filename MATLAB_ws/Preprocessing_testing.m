clear
close all
clc

load("Results/2Def/output_3.mat")

%% 
iter = 4;

idx_Time = ~isnan(Data_t_Xe_V_reg(:,1,iter));

Timestamps = Data_t_Xe_V_reg(idx_Time,1,iter);
X_e_act = Data_t_Xe_V_reg(idx_Time,2:3,iter);
signal = Data_t_Xe_V_reg(idx_Time,end-1,iter);

% ---------- Butterworth Filter

d_Time = diff(Timestamps);
Ts = mean(d_Time);
Fs = 1/Ts;

Fc = 6; % Frecuencia de corte
Wn = 2*Fc/Fs;
filter_order = 6;

[num, den] = butter(filter_order, Wn, "low");

% Respuesta del filtro con delay
% filtered_signal = filter(num, den, Mediciones);

% ---------- Zero-phase filter to correct the delay
clean_signal = filtfilt(num, den, signal);

% Threshold
Gamma_V = 88.5239;

idx_aboveThreshold = clean_signal > Gamma_V;

% ---------- Datos arriba del Threshold
% Data_out = [X_e_act(idx_aboveThreshold,:),...
%             clean_signal(idx_aboveThreshold)];

% ---------- Derivada de la señal limpia
derivative_signal = gradient(clean_signal, Ts);

% Valor absoluto de la derivada
abs_derivative_signal = abs(derivative_signal);

% Moving average of the absolute derivative signal
% Define the window size for moving average
window_sec = 0.5; % 0.4 segundos de ventana
window_samples = round(window_sec/Ts);  
movingAvgAbsDerivative = movmean(abs_derivative_signal, window_samples);

% Definición de Threshold sobre movAvg(abs(Derivative)) para detectar el
% drift del sensor
% idx_MovAvgAbsD_clnsurf = (Timestamps >= 0.2 & Timestamps <= 1.10) |...
%                         (Timestamps >= 8.04 & Timestamps <= 8.32);% |...
%                         % (Timestamps >= 4.6 & Timestamps <= 4.73) |...
%                         % (Timestamps >= 7.63 & Timestamps <= 8.12) |...
%                         % (Timestamps >= 9.875); 
% 
% 
% absD_drift_signal = movingAvgAbsDerivative(idx_MovAvgAbsD_clnsurf);
% a_absD = mean(absD_drift_signal);
% sigma_absD = std(absD_drift_signal);

Gamma_absD = 45.8505; % a_absD + 3*sigma_absD;
%28.8098
%43.7692
%59.6466
%47.1475
%49.8792

%% Filtering the signal for defect detection

% Leaving Measurements above threshold
def_signal = clean_signal;
def_signal(~idx_aboveThreshold) = NaN;

% Identify indices for measurements affected by drift
idx_drift = movingAvgAbsDerivative < Gamma_absD;

% Compensate for the moving average filter by removing samples
half_window = floor(window_samples / 2);

idx_drift_compensated = idx_drift;
for i = half_window+1:length(idx_drift)
    if idx_drift(i)
        idx_drift_compensated(i-half_window:i+half_window) = 1;
    end
end
idx_drift_compensated(end-half_window+1:end) = [];

% Remove measurements affected by drift
def_signal(idx_drift_compensated) = NaN; 


%% Plots

fig40h = figure(40);
tiledlayout(fig40h, 4,1);

nexttile
plot(Timestamps, signal, 'LineWidth', 2);
hold on
plot(Timestamps, clean_signal, 'LineWidth', 2, "Color", "black");
yline(Gamma_V, "LineWidth", 3, "Color", "red")
title('Signal');
xlabel('Time');
ylabel('Signal');
legend('Original Signal', 'Clean Signal');
grid on
hold off

nexttile
plot(Timestamps, derivative_signal, 'LineWidth', 2);
title('Derivative of the Clean Signal');
xlabel('Time');
ylabel('Derivative');
grid on
hold on
% yline([a, a-3*sigma, a+3*sigma], ":", ...
%     ["\mu", "$\Gamma_V^- = \mu- 3\sigma$", "$\Gamma_V^+ = \mu + 3\sigma$"], ...
%     "Color", "r", "FontWeight", "bold", "LineWidth", 3)
hold off

nexttile
plot(Timestamps, abs_derivative_signal, 'LineWidth', 2);
title('abs(Derivative)');
xlabel('Time');
ylabel('abs(D)');
grid on
hold on
plot(Timestamps, movingAvgAbsDerivative, 'LineWidth', 2);
yline(Gamma_absD, "Color", "green","LineWidth",2)
hold off
 
nexttile
plot(Timestamps, def_signal, 'LineWidth', 2);
title('Signal for defect detection');
xlabel('Time');
ylabel('$V_k$');
xlim([0 12])
ylim([0 250])
grid on

set(findall(fig40h,'-property','Interpreter'),'Interpreter','latex') 
set(findall(fig40h,'-property','TickLabelInterpreter'), ...
    'TickLabelInterpreter','latex')
set(findall(fig40h, "-property", "FontSize"), "FontSize", 18)