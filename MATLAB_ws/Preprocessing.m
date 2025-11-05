function [Data_out, CleanSignal] = Preprocessing(Data_in, thres_meas)

Timestamps = Data_in(:,1);
Mediciones = Data_in(:,end);
X_e_act = Data_in(:,2:3);

d_Time = diff(Timestamps);
Ts = mean(d_Time);
Fs = 1/Ts;

Fc = 6; %Frecuencia de corte
Wn = 2*Fc/Fs;
filter_order = 6;

[num, den] = butter(filter_order, Wn, "low");

% filtered_signal = filter(num, den, Mediciones);

% Zero-phase filter to correct the delay
corrected_signal = filtfilt(num, den, Mediciones);

%Threshold
Gamma_V = thres_meas;

idx_aboveThreshold = corrected_signal > Gamma_V;

Data_out = [X_e_act(idx_aboveThreshold,:),...
            corrected_signal(idx_aboveThreshold)];

CleanSignal = corrected_signal;


end