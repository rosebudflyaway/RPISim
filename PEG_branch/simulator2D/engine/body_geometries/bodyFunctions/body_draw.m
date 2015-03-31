
% Draws a body, returns the graphics handle

function body_draw( B )

    %% Update graphics
    % The vert_world coordinates have already been updated, so we
    % just need to update the graphics data.
    set(B.graphicsHandle,'xdata',B.verts_world(:,1));
    set(B.graphicsHandle,'ydata',B.verts_world(:,2));
    % Here, we exploit MATLAB's non-case-sensitivity to match Octave's
    % case-sensitivity for 'xdata' etc. in the graphics data.  

end



