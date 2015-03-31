
% W : width
% H : height
function R = body_rectangle( W, H )


    X = [W/2 W/2 -W/2 -W/2]';
    Y = [-H/2 H/2 H/2 -H/2]';
    
    R = Body(X, Y);


end

