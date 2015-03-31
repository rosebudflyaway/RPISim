


function body = Body_cylinder( height, radius )

    body = Body();
    body.type = 'cylinder'; 
    body.height = height; 
    body.radius = radius; 
    body.J(1,1) = (1/12) * body.mass * (3*radius^2*height^2);
    body.J(2,2) = body.J(1,1);
    body.J(3,3) = (body.mass * radius^2) / 2;

end


