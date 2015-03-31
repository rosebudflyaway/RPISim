%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% Given a body B, draws it and returns the updated body with its graphics
% handle. 
function B = body_draw_init( B )

    h = fill(B.verts_local(:,1), B.verts_local(:,2), B.color);
    B.Xdata = get(h,'xdata');
    B.Ydata = get(h,'ydata'); 
    set(h,'FaceAlpha',B.faceAlpha); 
    
    set(h,'facealpha',B.faceAlpha);
    
    if ~B.visible, set(h,'visible','off'); end 
    B.graphicsHandle = h; 
    
    body_draw( B ); % Draw again at world coordinates

end

