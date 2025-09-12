function X = rest_to_rest_trajectory(P_i, P_f, t_i, mvr_time, t,n)
    
    % P_i (dim x 1):     initial pose
    % P_f (dim x 1):     final pose
    % t_i (1 x 1):       initial time of maneuver
    % mvr_time (1 x 1):  maneuver time
    % t (1 x 1):         current time value
    % n (1 x 1):         number of derivatives set to zero at initial and
    %                    final pose
    
    if n < 1
        error("Hijole mano, no se va a poder xd")
    end

    P_dim = length(P_i); %Pose dimension

    t_f = t_i + mvr_time; %Final time of maneuver
    
    %Inicializaciones
    A = zeros(n+1,n+1,P_dim);
    Alpha = zeros(n+1,1,P_dim);
    Lambda = zeros(n+1,1,P_dim);
    Lambda(1,1,:) = 1;
    f = zeros(P_dim,1);
    
    %Cálculos
    for r = 1:P_dim

        for i = 1:n+1
            for j = 1:n+1
                A(i,j,r) = ((-1)^(j-1))*factorial(n+j) /...
                         (((t_f - t_i)^(i-1))*factorial(n+j-i+1));
            end
        end

        Alpha(:,1,r) = A(:,:,r)\Lambda(:,1,r);
        
        for j = 0:n
            f(r) = f(r) + ((-1)^j).*Alpha(j+1,1,r).*((t-t_i)./(t_f-t_i)).^(n+1+j);
        end

    end

    X = P_i + (P_f - P_i).*f;

end