
function B = body_square( )

    
    X = [-0.5; -0.5;  0.5; 0.5]; 
    Y = [ 0.5; -0.5; -0.5; 0.5];
    B = Body(X,Y); 

    B.J = 1/12;

end

