function penetration = plane_penetration(xyPosition)
% CurrentPosition (2x1) Position on the x-y plane
% penetration     (1x1) z-coordinate needed to reach f = 5 N

x = xyPosition(1);
y = xyPosition(2);

% Empirical and tested points with the z-coordinate reaching f = 5 N
P_1 = [0.2; 0.05; -0.0056];
P_2 = [0.395; 0.115; -0.0075];
P_3 = [0.25; 0.24; -0.0105];

x_1 = P_1(1);
y_1 = P_1(2);
z_1 = P_1(3);

% Compute vectors on the unknown plane and normal vector
u_1 = P_2 - P_1;
u_2 = P_3 - P_1;
n_hat = cross(u_1, u_2);

n_x = n_hat(1);
n_y = n_hat(2);
n_z = n_hat(3);

penetration = z_1 - ( (x - x_1)*n_x + (y - y_1)*n_y )/n_z ;

end