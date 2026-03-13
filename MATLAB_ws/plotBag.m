function plotBag(Data_t_Xe_V, Thresholds, Preprocess_struct, X_e_d, flag_exp)

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

P_s = [-9.11, -20;
    -11.8, -19.7;
    -14.34, -18.81;
    -16.62, -17.38;
    -18.53, -15.47;
    -19.98, -13.19;
    -20.87, -10.65;
    -21.18, -7.97;
    -20.9, -5.28;
    -20.01, -2.73;
    -10.66, 16.89;
    -8.83, 19.65;
    -6.30, 21.80;
    -3.28, 23.16;
    0, 23.62;
    3.28, 23.16;
    6.30, 21.80;
    8.83, 19.65;
    10.66, 16.89;
    20.01, -2.73;
    20.9, -5.28;
    21.18, -7.97;
    20.87, -10.65;
    19.98, -13.19;
    18.53, -15.47;
    16.62, -17.38;
    14.34, -18.81;
    11.8, -19.7;
    9.11, -20]*1e-3;
vertcs_x = P_s(:,1) + X_e_d(:,1)';
vertcs_y = P_s(:,2) + X_e_d(:,2)';

nexttile(4, [4 3])
patch(vertcs_x, vertcs_y, "blue", "FaceAlpha", 0.03, "EdgeColor", "none")
hold on
patch([Poses_x; NaN], [Poses_y; NaN], [Mediciones; NaN],...
      'EdgeColor','interp',"LineWidth", 3)
cb = colorbar;
cb.Label.String = '$V_k$';
cb.Label.Interpreter = "latex";
xlabel('$x_1$')
ylabel('$x_2$')
grid on
axis equal
xlim([0, 0.28])
ylim([0, 0.2])
plot(Poses_x(1), Poses_y(1), "bsq", "MarkerSize",16)
hold off
legend("footprint",'$X_e(t)$', "$X_e(0)$")

set(findall(figh,'-property','Interpreter'),'Interpreter','latex') 
set(findall(figh,'-property','TickLabelInterpreter'), ...
    'TickLabelInterpreter','latex')
set(findall(figh, "-property", "FontSize"), "FontSize", 18)

end