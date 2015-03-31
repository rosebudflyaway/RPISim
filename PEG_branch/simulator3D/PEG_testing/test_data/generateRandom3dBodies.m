
% Generates N bodies
function bodies = generateRandom3dBodies( N )


    num_points = 15;
    n = num_points*N;  
    values = rand(3*n,1);
    
    X = values(1:n);
    Y = values(n+1:n+n);
    Z = values(n+n+1:end);
    
    bodies = [];
    for i=1:N
        x = X(i*num_points-num_points+1:i*num_points);
        y = Y(i*num_points-num_points+1:i*num_points);
        z = Z(i*num_points-num_points+1:i*num_points);
        K = convhull([x y z],'simplify', true);
        newBody = Body_mesh([x y z],K);
        newBody.color = rand(1,3);
        newBody.quat = qt(rand(1,3),2*pi*rand); 
        bodies = [bodies; newBody];
    end

end

