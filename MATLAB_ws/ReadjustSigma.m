function Sigma_r = ReadjustSigma(Sigma, offset, AspectRatio)

% Sigma: (nxnxm) m Covariance matrixes of dimension n
% offset: (scalar) offset to readjust over the largest semiaxis 
%                   of the ellipsoids
% AspectRatio: (True/False) flag to respect aspect ratio when readjusting

% Sigma_r: (nxnxm) Readjusted Sigma
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

stdev = zeros(size(Sigma));
Sigma_ast = zeros(size(Sigma));

% Augmented covariance matrix (the one we want to compute)
Sigma_r = zeros(size(Sigma)); 
for i = 1:size(Sigma, 3)
    % Standard deviation
    stdev(:,:,i) = sqrtm(Sigma(:,:,i)); 
    % 3*Standard deviation that represents 99% of data
    Sigma_ast(:,:,i) = 3*stdev(:,:,i); 
    % Eigenvectors = V, Eigenvalues diagonal matriz = D
    [V, D] = eig(Sigma_ast(:,:,i));
    % Sorting with Max eigenvalue at last spot
    [r_j, ind] = sort(diag(D)); %r_j is the vector of elipse radius = eigenvalues
    D_sorted = D(ind, ind); 
    V_sorted = V(:,ind);

    % Respect aspect ratio (r_2 = Ratio*r_1)
    r_extension = zeros(size(r_j));
    if AspectRatio
        Ratio = r_j(end) / r_j(end-1);
    else
        Ratio = 1;
    end
    % Defining offset over largest axis and computing the rest
    r_extension(end) = offset;
    r_extension(end-1) = offset/Ratio;

    % New diagonal eigenvalues matriz with offset
    D_r = D_sorted + diag(r_extension);

    % Readjusted covariance matrix
    Sd_r = V_sorted*D_r*V_sorted' / 3;
    Sigma_r(:,:,i) = Sd_r * Sd_r;

end



end