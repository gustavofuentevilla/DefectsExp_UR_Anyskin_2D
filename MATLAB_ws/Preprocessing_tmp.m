function [Data_out, Preprocess_struct] = Preprocessing_tmp(Data_in, Thresholds, planned_vel, flag_exploration)

% Parameters
thres_meas_stg1 = Thresholds(1);
thres_meas_stg2 = Thresholds(2);
thres_vel = 0.09;
thres_absD = Thresholds(4);
t = planned_vel(:,1);
X_e_d_dot = planned_vel(:,2:3);

Timestamps = Data_in(:,1);
X_e_act = Data_in(:,2:3); % X_e = [x_ee, yee]
signal = Data_in(:,end);

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

% Threshold on measurements
if flag_exploration
    Gamma_V = thres_meas_stg1;
else
    Gamma_V = thres_meas_stg2;
end

idx_aboveThreshold = clean_signal > Gamma_V;

% ---------- Derivada de la señal limpia
derivative_signal = gradient(clean_signal, Ts);

% Valor absoluto de la derivada
abs_derivative_signal = abs(derivative_signal);

% Moving average of the absolute derivative signal
% Define the window size for moving average
window_sec = 0.5; % 0.5 segundos de ventana
window_samples = round(window_sec/Ts);  
movingAvgAbsDerivative = movmean(abs_derivative_signal, window_samples);

% Thresshold on moving average abs derivative
Gamma_absD = thres_absD;

% ------------ Compute velocity norm of planned trajectory

v_Norm = sqrt(sum(X_e_d_dot.^2, 2));
v_Norm_spline = spline(t, v_Norm, Timestamps);

Gamma_vel = thres_vel;

idx_almostZero_vel = v_Norm_spline < Gamma_vel;

% ----------- Filtering the signal for defect detection

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
if length(idx_drift_compensated) > length(idx_drift)
    len_c = length(idx_drift_compensated) - length(idx_drift);
    idx_drift_compensated(end-len_c+1:end) = [];
end

% Leaving Measurements above threshold
def_signal = clean_signal;
def_signal(~idx_aboveThreshold) = NaN;

% Erase non-defective measurements (Drifts)
vel_samples = round(0.4/Ts); % 0.4 segundos
idx_almostZero_vel(1:vel_samples) = 0;
idx_almostZero_vel(end-vel_samples:end) = 0;
idx_eraseDrift = idx_drift_compensated & ~idx_almostZero_vel;
def_signal(idx_eraseDrift) = NaN;

idx_data = ~isnan(def_signal);

%% ---------- Exportar datos
Data_out = [X_e_act(idx_data,:),...
            clean_signal(idx_data)];

% Estructura
Preprocess_struct.clean_signal = clean_signal;
Preprocess_struct.abs_derivative_signal = abs_derivative_signal;
Preprocess_struct.movingAvgAbsDerivative = movingAvgAbsDerivative;
Preprocess_struct.v_Norm_spline = v_Norm_spline;
Preprocess_struct.def_signal = def_signal;

end