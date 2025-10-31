%% Pre-processing

% Butterworth filter design

d_Time = diff(Timestamps);
Ts = mean(d_Time);
Fs = 1/Ts;
L_signal = numel(Timestamps);

Y = fft(Mediciones);

P2 = abs(Y/L_signal);
P1 = P2(1:L_signal/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs/L_signal*(0:(L_signal/2));
figure(2)
plot(f,P1,"LineWidth",2) 
title("Single-Sided Amplitude Spectrum of X(t)")
xlabel("f (Hz)")
ylabel("|P1(f)|")

Fc = 6; %Frecuencia de corte
Wn = 2*Fc/Fs;
filter_order = 6;

[num, den] = butter(filter_order, Wn, "low");

filtered_signal = filter(num, den, Mediciones);

% Zero-phase filter to correct the delay
corrected_signal = filtfilt(num, den, Mediciones);

% Derivada de la señal limpia
derivative_signal = gradient(corrected_signal, Ts);

% Valor absoluto de la derivada
abs_derivative_signal = abs(derivative_signal);

% Squared derivative
SquareDerivative = derivative_signal.^2;

%% Gráficas

fig3h = figure(3);
tiledlayout(fig3h, 4,1);

nexttile
plot(Timestamps, Mediciones, 'LineWidth', 2);
hold on
plot(Timestamps, filtered_signal, 'LineWidth', 2);
plot(Timestamps, corrected_signal, 'LineWidth', 2, "Color", "black");
title('Filtered Signal');
xlabel('Time');
ylabel('Signal');
legend('Original Signal', 'Filtered Signal', 'Corrected Signal');
grid on

nexttile
plot(Timestamps, derivative_signal, 'LineWidth', 2);
title('Derivative of the Cleaned Signal');
xlabel('Time');
ylabel('Derivative');
grid on

nexttile
plot(Timestamps, abs(derivative_signal), 'LineWidth', 2);
title('abs(Derivative)');
xlabel('Time');
ylabel('abs(D)');
grid on

nexttile
plot(Timestamps, SquareDerivative, 'LineWidth', 2);
title('Squared Derivative');
xlabel('Time');
ylabel('$D^2$');
grid on

set(findall(fig3h,'-property','Interpreter'),'Interpreter','latex') 
set(findall(fig3h,'-property','TickLabelInterpreter'), ...
    'TickLabelInterpreter','latex')
set(findall(fig3h, "-property", "FontSize"), "FontSize", 18)

