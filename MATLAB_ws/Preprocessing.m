%% Pre-processing

d_Time = diff(Timestamps);
Ts = mean(d_Time);
Fs = 1/Ts;
L_signal = numel(Timestamps);

Y = fft(Mediciones);

P2 = abs(Y/L_signal);
P1 = P2(1:L_signal/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs/L_signal*(0:(L_signal/2));
figure(1)
plot(f,P1,"LineWidth",2) 
title("Single-Sided Amplitude Spectrum of X(t)")
xlabel("f (Hz)")
ylabel("|P1(f)|")

Fc = 5.5; %Frecuencia de corte
Wn = 2*Fc/Fs;
filter_order = 2;

[num, den] = butter(filter_order, Wn, "low");

filtered_signal = filter(num, den, Mediciones);

figure(2)
plot(Timestamps, Mediciones, 'LineWidth', 2);
hold on
plot(Timestamps, filtered_signal, 'LineWidth', 2);
title('Filtered Signal');
xlabel('Time');
ylabel('Signal');
hold off