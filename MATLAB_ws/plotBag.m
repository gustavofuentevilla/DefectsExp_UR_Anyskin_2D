function plotBag(Data_t_Xe_V, threshold)

Timestamps = Data_t_Xe_V(:, 1);
Poses_x = Data_t_Xe_V(:, 2);
Poses_y = Data_t_Xe_V(:, 3);
Mediciones = Data_t_Xe_V(:, 5);
clean_signal = Data_t_Xe_V(:, 6);

figh = figure;
tiledlayout(figh, 3, 6);

nexttile(1, [1 3])
plot(Timestamps, Mediciones, "LineWidth", 2)
hold on
plot(Timestamps, clean_signal, 'LineWidth', 2, "Color", "black");
yline(threshold, "LineWidth", 3, "Color", "red")
title('Signal')
xlabel('Time (s)')
ylabel('Measurements')
legend('Original Signal', 'Clean Signal');
grid on
hold off

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

end