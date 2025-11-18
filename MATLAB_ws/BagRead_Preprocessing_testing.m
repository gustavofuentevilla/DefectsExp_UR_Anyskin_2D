
close all
clear
clc

%% Bag reading

folderPath = fullfile(pwd, "custom");
ros2genmsg(folderPath)

%%
% Check for msg
% ros2 msg show custom_interfaces/SyncData

folderPathBag = fullfile(pwd, "ROS2Bags/Pos005005/rosbag_20251113_160903_628377"); 
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

% --- Butterworth filter design

% d_Time = diff(Timestamps);
% Ts = mean(d_Time);
% Fs = 1/Ts;
% L_signal = numel(Timestamps);
% 
% Y = fft(Mediciones);
% 
% P2 = abs(Y/L_signal);
% P1 = P2(1:L_signal/2+1);
% P1(2:end-1) = 2*P1(2:end-1);
% 
% f = Fs/L_signal*(0:(L_signal/2));
% 
% figure(2)
% plot(f,P1,"LineWidth",2) 
% title("Single-Sided Amplitude Spectrum of X(t)")
% xlabel("f (Hz)")
% ylabel("|P1(f)|")

Fc = 6; %Frecuencia de corte
Wn = 2*Fc/Fs;
filter_order = 6;

[num, den] = butter(filter_order, Wn, "low");

filtered_signal = filter(num, den, Mediciones);

% Zero-phase filter to correct the delay
clean_signal = filtfilt(num, den, Mediciones);

% Derivada de la señal limpia
derivative_signal = gradient(clean_signal, Ts);

% Valor absoluto de la derivada
% abs_derivative_signal = abs(derivative_signal);

% Moving average of the absolute derivative signal
% windowSize = 18; % Define the window size for moving average
% movingAvgAbsDerivative = movmean(abs_derivative_signal, windowSize);

% Squared derivative
% SquareDerivative = derivative_signal.^2;
% movingAvgSqDerivative = movmean(SquareDerivative, windowSize);

% Definición de Threshold

% idx_clean = (Timestamps <= 0.35) | (Timestamps >= 0.72 & Timestamps <= 1.765) | ...
%     (Timestamps >= 2.3 & Timestamps <= 4.16) |...
%     (Timestamps >= 4.8 & Timestamps <= 5.72) |...
%     (Timestamps >= 6.06 & Timestamps <= 7.45) | (Timestamps >= 8);

% CleanSurf = derivative_signal(idx_clean);

a = 0; % mean(CleanSurf);

sigma = 33.8268; % std(CleanSurf);

Gamma_V = a + 3*sigma; %Threshold

% measurements above threshold indexes
% idx_aboveThreshold = movingAvgAbsDerivative > Gamma_V;
% thresholdedSignal = clean_signal;
% thresholdedSignal(~idx_aboveThreshold) = NaN;

% Gráficas

fig3h = figure(3);
tiledlayout(fig3h, 2,1);

nexttile
plot(Timestamps, Mediciones, 'LineWidth', 2);
hold on
plot(Timestamps, clean_signal, 'LineWidth', 2, "Color", "black");
yline(85.8619, "LineWidth", 3, "Color", "red")
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
yline([a, a-3*sigma, a+3*sigma], ":", ...
    ["\mu", "$\Gamma_V^- = \mu- 3\sigma$", "$\Gamma_V^+ = \mu + 3\sigma$"], ...
    "Color", "r", "FontWeight", "bold", "LineWidth", 3)
hold off

% nexttile
% plot(Timestamps, abs_derivative_signal, 'LineWidth', 2);
% title('abs(Derivative)');
% xlabel('Time');
% ylabel('abs(D)');
% grid on
% 
% nexttile
% plot(Timestamps, movingAvgAbsDerivative, 'LineWidth', 2);
% title('Moving averaged abs(Derivative)');
% xlabel('Time');
% ylabel('abs(D)*');
% hold on
% yline(100, "LineWidth", 2, "Color","red")
% hold off
% grid on
% 
% nexttile
% plot(Timestamps, SquareDerivative, 'LineWidth', 2);
% title('Squared Derivative');
% xlabel('Time');
% ylabel('$D^2$');
% grid on
% 
% nexttile
% plot(Timestamps, movingAvgSqDerivative, 'LineWidth', 2);
% title('Moving averaged $D^2$');
% xlabel('Time');
% ylabel('$D^2$*');
% hold on
% yline(10000, "LineWidth", 2, "Color","red")
% hold off
% grid on

set(findall(fig3h,'-property','Interpreter'),'Interpreter','latex') 
set(findall(fig3h,'-property','TickLabelInterpreter'), ...
    'TickLabelInterpreter','latex')
set(findall(fig3h, "-property", "FontSize"), "FontSize", 18)

% plot
% fig4h = figure(4);
% tiledlayout(fig4h, 3,1);
% 
% nexttile
% plot(Timestamps, clean_signal, 'LineWidth', 2, "Color", "black");
% title("Filtered signal $\bar{V}_k$");
% xlabel('Time');
% ylabel('$\mu~T$');
% grid on
% 
% nexttile
% plot(Timestamps, movingAvgAbsDerivative, 'LineWidth', 2);
% title('Moving averaged abs(Derivative)');
% xlabel('Time');
% ylabel('abs(D)*');
% grid on
% hold on
% yline([a, a-3*sigma, a+3*sigma], ":", ...
%     ["\mu", "\mu- 3\sigma", "Threshold = \mu+ 3\sigma"], ...
%     "Color", "r", "FontWeight", "bold", "LineWidth", 3)
% hold off
% 
% nexttile
% plot(Timestamps, thresholdedSignal, 'LineWidth', 2, "Color", "black")
% title("Measurements $\bar{V}_k$ above threshold");
% xlabel('Time');
% xlim([0, 10])
% ylabel('$\mu~T$');
% grid on
% 
% set(findall(fig4h,'-property','Interpreter'),'Interpreter','latex') 
% set(findall(fig4h,'-property','TickLabelInterpreter'), ...
%     'TickLabelInterpreter','latex')
% set(findall(fig4h, "-property", "FontSize"), "FontSize", 18)

