
% Returns a polygon body struct with vertices defined by X,Y
function body = Body( X,Y )

    body.bodyID = 0; 
    body.bodyContactID = 0; 

    % Body data
    body.dynamic = true; 
    body.active = false; 
    body.mass = 1; 
    body.pos = [0; 0];
    body.rot = 0; 
    body.nu = zeros(3,1); 
    body.Fext = zeros(3,1);
    body.Aext = zeros(3,1); 
    body.mu = 0.5; 
    body.J = 1;         % TODO: properly set inertia
    
    % Vertex data
    body.verts_local = zeros(length(X),2);
    body.verts_local = [X Y]; 
    body.verts_world = body.verts_local; 
    body.num_verts = size( body.verts_local, 1 ); 
    
    % Graphics data
    body.graphicsHandle = [];  
    body.bboxHandle = []; 
    body.Xdata = []; 
    body.Ydata = [];
    body.color = [.6 .6 .8]; 
    body.faceAlpha = 0.7; 
    body.visible = true; 

    % Collision detection data
    body.AABB_min = 0;     
    body.AABB_max = 0; 
    body.doesNotCollideWith = []; 
    
    % Joint data
    body.numJoints = 0; 
    
end
