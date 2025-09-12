function plotFrame(origin, R, varargin)
% plotFrame(origin, R, 'Name', frameName, 'Scale', scale)
% Grafica un marco de referencia 3D en la posición 'origin' 
% con orientación 'R' (3x3)

% origin: [x; y; z] posición del marco
% R: matriz de rotación 3x3, cada columna es un eje (X, Y, Z)
% Opcional: 'Name' para el texto, 'Scale' para la longitud de los ejes

% Parámetros opcionales
p = inputParser;
addParameter(p, 'Name', '', @ischar);
addParameter(p, 'Scale', 1, @isnumeric);
parse(p, varargin{:});
frameName = p.Results.Name;
scale = p.Results.Scale;

% Ejes del frame
colors = {'r', 'g', 'b'}; % X: rojo, Y: verde, Z: azul

hold on;
for i = 1:3
    quiver3(origin(1), origin(2), origin(3), ...
        R(1,i)*scale, R(2,i)*scale, R(3,i)*scale, ...
        'Color', colors{i}, 'LineWidth', 2, 'MaxHeadSize', 0.5);
end
if ~isempty(frameName)
    text(origin(1), origin(2), origin(3), [' ' frameName],...
        'FontSize', 14, 'FontWeight', 'bold');
end
hold off;
end