clear
close all
clc

load("Results/N100/2Def/output_1.mat")

%% Data
it = 1;
idxnan = isnan(Data_t_Xe_V_reg(:, 1, it));
Data = Data_t_Xe_V_reg(~idxnan, :, it);

Timestamps = Data(:,1);
Poses_x = Data(:,2);
Poses_y = Data(:,3);
Mediciones = Data(:,end-1);
clean_signal = Data(:,end);


%% Visualize

figh = figure;
tiledlayout(figh, 2, 4);

nexttile(1, [1 2])
plot(Timestamps, Mediciones, "LineWidth", 2)
hold on
plot(Timestamps, clean_signal, 'LineWidth', 2, "Color", "black");
title('Signal')
xlabel('Time (s)')
ylabel('Measurements')
legend('Original Signal', 'Clean Signal');
grid on
hold off

nexttile(5, [1 2])
plot(Timestamps, [Poses_x, Poses_y], "LineWidth", 2)
title("Positions")
xlabel("Time")
ylabel("[m]")
legend('$x_1$', '$x_2$');
grid on

nexttile(3, [2 2])
patch([Poses_x; NaN], [Poses_y; NaN], [clean_signal; NaN],...
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
hold on
plot(Poses_x(1), Poses_y(1), "bsq", "MarkerSize",16)
hold off

set(findall(figh,'-property','Interpreter'),'Interpreter','latex') 
set(findall(figh,'-property','TickLabelInterpreter'), ...
    'TickLabelInterpreter','latex')
set(findall(figh, "-property", "FontSize"), "FontSize", 18)

%% Modeling threshold

idx_cleansurf = (Timestamps <= 4.614) | ...
                (Timestamps >= 5.365 & Timestamps <= 6.69) |...
                (Timestamps >= 7.33);
                % (Timestamps >= 3.64 & Timestamps <= 5.345) | ...
                
                % (Timestamps >= 7.25 & Timestamps <= 9.51); | ...
                

mu_thres = mean(clean_signal(idx_cleansurf));
Sd_thres = std(clean_signal(idx_cleansurf));

thres_meas = mu_thres + 3*Sd_thres;

%% Plot threshold

nexttile(1, [1, 2])
hold on
yline(thres_meas, "LineWidth", 3,...
    "Color", "red", "DisplayName", "Threshold",...
    "Label", "Threshold = " + thres_meas, "FontSize",20)
hold off