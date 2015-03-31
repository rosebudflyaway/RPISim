
% Returns an equilateral triangle T. 
function T = body_triangle( )

    %      v1
    %      /\
    %     /  \
    %    /    \
    %  v2------v3

    r = sqrt(2); % radius of circle inscribed in triangle
    X = [0; -r*cos(pi/6); r*cos(pi/6)]; 
    Y = [r; -r*sin(pi/6); -r*sin(pi/6)];
    T = Body(X,Y); 

    b = max(X) - min(X);
    h = max(Y) - min(Y); 
    T.J = (b*h^3)/36;

end

