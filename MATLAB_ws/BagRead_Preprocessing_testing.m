
close all
clear
clc

%% Bag reading

folderPath = fullfile(pwd, "custom");
ros2genmsg(folderPath)

% Check for msg
% ros2 msg show custom_interfaces/SyncData

folderPathBag = fullfile(pwd, "ROS2Bags/rosbag_20251024_185001_144916"); 
bagReader = ros2bagreader(folderPathBag);
% bagReader.AvailableTopics

% bagInfo = ros2('bag', 'info', folderPathBag);

% Get all messages

messageAll = readMessages(bagReader);

% Extract Data and merge into one single variable

% Extract the timestamp from the messages
Timestamps = cell2mat(cellfun(@(msg) double(msg.stamp.sec) + double(msg.stamp.nanosec) * 1e-9, ...
                    messageAll, 'UniformOutput', false));
Timestamps = Timestamps - Timestamps(1); % first value as baseline

% Extract the measurements
Mediciones = double(cell2mat(cellfun(@(msg) msg.measurements.data,...
                    messageAll, 'UniformOutput', false)));

% Extract the end-effector position in x
Poses_x = double(cell2mat(cellfun(@(msg) msg.ee_pose.pose.position.x,...
                 messageAll, 'UniformOutput', false)));

% Extract the end-effector position in y
Poses_y = double(cell2mat(cellfun(@(msg) msg.ee_pose.pose.position.y,...
                 messageAll, 'UniformOutput', false)));

% Extract the end-effector position in z
Poses_z = double(cell2mat(cellfun(@(msg) msg.ee_pose.pose.position.z,...
                 messageAll, 'UniformOutput', false)));

% Combine the extracted data into a single matrix
Data = [Timestamps, Poses_x, Poses_y, Poses_z, Mediciones];

% Create table of combinedData
% combinedTable = array2table(Data, 'VariableNames', {'Time', 'PosX', 'PosY', 'PosZ', 'Measurements'});

% Plot the data

figh = figure(1);
tiledlayout(figh, 3, 6);

nexttile(1, [1 3])
plot(Timestamps, Mediciones, "LineWidth", 2)
xlabel('Time (s)')
ylabel('Measurements')
legend('$V_k$')
grid on

nexttile(7, [1 3])
plot(Timestamps, Poses_x, "LineWidth", 2)
xlabel('Time (s)')
ylabel('X Position')
legend('$x_{ee}$')
grid on

nexttile(13, [1 3])
plot(Timestamps, Poses_y, "LineWidth", 2)
xlabel('Time (s)')
ylabel('Y Position')
legend('$y_{ee}$')
grid on

nexttile(4, [3 3])
patch([Poses_x; NaN], [Poses_y; NaN], [Mediciones; NaN],...
      'EdgeColor','interp',"LineWidth", 3)
cb = colorbar;
cb.Label.String = '$V_k$';
cb.Label.Interpreter = "latex";
xlabel('$x_1$')
ylabel('$x_2$')
legend('$X_e(t)$')
grid on
axis equal
xlim([0, 0.28])
ylim([0, 0.2])

set(findall(figh,'-property','Interpreter'),'Interpreter','latex') 
set(findall(figh,'-property','TickLabelInterpreter'), ...
    'TickLabelInterpreter','latex')
set(findall(figh, "-property", "FontSize"), "FontSize", 18)

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

% Gráficas

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

% Definición de Threshold

idx_clean = (Timestamps <= 0.5) | (Timestamps >= 1 & Timestamps <=1.94) | ...
    (Timestamps >= 2.44 & Timestamps <= 4.06) |...
    (Timestamps >= 4.93 & Timestamps <= 5.9) |...
    (Timestamps >= 6.2 & Timestamps <= 7.59) | (Timestamps >= 8);


CleanSurfMeas = corrected_signal(idx_clean);

a = mean(CleanSurfMeas);

sigma = std(CleanSurfMeas);

Gamma_V = a + 3*sigma; %Threshold

% plot
fig4h = figure(4);
plot(Timestamps, corrected_signal, 'LineWidth', 2, "Color", "black");
title("Filtered signal $\bar{V}_k$");
xlabel('Time');
ylabel('$\mu~T$');
grid on
hold on
yline([a, a-3*sigma, a+3*sigma], ":", ["\mu", "\mu- 3\sigma", "\mu+ 3\sigma"], ...
    "Color", "r", "FontWeight", "bold", "LineWidth", 3)
hold off

set(findall(fig4h,'-property','Interpreter'),'Interpreter','latex') 
set(findall(fig4h,'-property','TickLabelInterpreter'), ...
    'TickLabelInterpreter','latex')
set(findall(fig4h, "-property", "FontSize"), "FontSize", 18)

% Datos arriba del threshold

idx_aboveThreshold = corrected_signal > Gamma_V;

% Extract the measurements above the threshold
aboveThresholdMeasurements = corrected_signal(idx_aboveThreshold);

