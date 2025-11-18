function [Data_out, CleanSignal] = Preprocessing(Data_in, thres_meas)

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

% Threshold
Gamma_V = thres_meas;

idx_aboveThreshold = clean_signal > Gamma_V;

% ---------- Datos arriba del Threshold
Data_out = [X_e_act(idx_aboveThreshold,:),...
            clean_signal(idx_aboveThreshold)];

CleanSignal = clean_signal;


end