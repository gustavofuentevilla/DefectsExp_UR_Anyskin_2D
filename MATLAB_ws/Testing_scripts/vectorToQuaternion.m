function q = vectorToQuaternion(v, reference)
% vectorToQuaternion  Obtiene el cuaternión que rota el vector 'reference' al vector 'v'
% Ejemplo: q = vectorToQuaternion([0.2; 0.5; 0.8], [1; 0; 0]);
% El cuaternión está en formato q = [w; x; y; z]

if nargin < 2
    reference = [0; 0; 1]; % Por defecto, eje Z
end

v = v(:) / norm(v); % Normalizar
reference = reference(:) / norm(reference);

axis = cross(reference, v);
axis_norm = norm(axis);

if axis_norm < 1e-8
    % Vectores son paralelos: identidad o 180 grados
    if dot(reference, v) > 0
        q = [1; 0; 0; 0]; % Cuaternión identidad
    else
        % 180 grados: eje perpendicular cualquiera
        ortho = null(reference.');
        axis = ortho(:,1);
        q = [0; axis];
    end
    return
end

axis = axis / axis_norm;
angle = acos(dot(reference, v));
qw = cos(angle/2);
q_xyz = axis * sin(angle/2);
q = [qw; q_xyz];
end