function plotBag(Data_t_Xe_V, Thresholds, Preprocess_struct, flag_exp)

Timestamps = Data_t_Xe_V(:, 1);
Poses_x = Data_t_Xe_V(:, 2);
Poses_y = Data_t_Xe_V(:, 3);
Mediciones = Data_t_Xe_V(:, 5);
clean_signal = Data_t_Xe_V(:, 6);
thres_meas_stg1 = Thresholds(1);
thres_meas_stg2 = Thresholds(2);
thres_vel = Thresholds(3);
thres_absD = Thresholds(4);
def_signal = Preprocess_struct.def_signal;
movingAvgAbsDerivative = Preprocess_struct.movingAvgAbsDerivative;
v_Norm_spline = Preprocess_struct.v_Norm_spline;
abs_derivative_signal = Preprocess_struct.abs_derivative_signal;

if flag_exp
    thres_meas = thres_meas_stg1; % Set threshold for measurements
else
    thres_meas = thres_meas_stg2;
end

figh = figure;
tiledlayout(figh, 4, 6);

nexttile(1, [1 3])
plot(Timestamps, Mediciones, "LineWidth", 2)
hold on
plot(Timestamps, clean_signal, 'LineWidth', 2, "Color", "black");
yline(thres_meas, "LineWidth", 3, "Color", "red")
title('Signal')
xlabel('Time (s)')
ylabel('Measurements')
legend('Original Signal', 'Clean Signal');
grid on
hold off

nexttile(7, [1 3])
plot(Timestamps, v_Norm_spline, "LineWidth", 2)
title("Planned Velocity Norm")
xlabel("Time")
ylabel("|v|")
grid on
hold on
yline(thres_vel, "LineWidth",2)
hold off

nexttile(13, [1 3])
plot(Timestamps, abs_derivative_signal, 'LineWidth', 2);
title('abs(Derivative)');
xlabel('Time');
ylabel('abs(D)');
grid on
hold on
plot(Timestamps, movingAvgAbsDerivative, 'LineWidth', 2);
yline(thres_absD, "Color", "green","LineWidth",2)
hold off

nexttile(19, [1 3])
plot(Timestamps, def_signal, "LineWidth", 2)
xlabel('Time (s)')
ylabel('Measurements')
legend('data for training')
grid on
xlim([0, 12])

nexttile(4, [4 3])
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
hold on
plot(Poses_x(1), Poses_y(1), "bsq", "MarkerSize",16)
hold off

set(findall(figh,'-property','Interpreter'),'Interpreter','latex') 
set(findall(figh,'-property','TickLabelInterpreter'), ...
    'TickLabelInterpreter','latex')
set(findall(figh, "-property", "FontSize"), "FontSize", 18)

end