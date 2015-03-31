function drawScene(body)
persistent hereb4 body_hndl

if isempty(hereb4)
    hereb4 = 1;
    figure('Units', 'normalized', 'Position',[.59 .5 .4 .4]);
    
    % Draw coordinate axes
    min_max = [-10 10];
    axes('Position',[-0.2 -0.2 1.4 1.4], 'Visible','off');
    line(min_max,[0 0],[0 0],'Color','r');  % Inertial x axis
    line([0 0],min_max,[0 0],'Color','g');  % Inertial y axis
    line([0 0],[0 0],min_max,'Color','b');  % Inertial z axis
    view(20,40);    % Set the camera view point
    hold on;
    axis equal;
    
    % Create surfs for the planes of support
    planeNrml(:,1) = [0.1; 0; sqrt(1 - 0.1^2)];
    planeNrml(:,2) = [-0.1/2;  0.1*sqrt(3)/2; sqrt(1 - 0.1^2)];
    planeNrml(:,3) = [-0.1/2; -0.1*sqrt(3)/2; sqrt(1 - 0.1^2)];
    num_steps = 25;
    step = (min_max(2) - min_max(1)) / num_steps;
    for i = 1 : 3
        % Draw a plane
        pln_x = min_max(1) : step : min_max(2);
        pln_xx = reshape(repmat(pln_x, 1, num_steps+1), num_steps+1, num_steps+1);
        pln_yy = reshape(repmat(pln_x', 1, num_steps+1), num_steps+1, num_steps+1)';
        nrml = planeNrml(:,i);
        pln_zz = -(nrml(1) * pln_xx + nrml(2) * pln_yy) / nrml(3);
        hndl_plane(i) = surf(pln_xx, pln_yy, pln_zz);   % Draw the plane
        clr = (rand(3,1) + 1)/2;   % Choose random color
        set(hndl_plane(i), 'FaceColor', clr);    % Set face color
        clr(i) = 0;
        set(hndl_plane(i), 'EdgeColor', clr);
    end
    for i = 1 : length(body)
        % Draw a sphere
        num = 20;  % Discretization level
        pos = body(i).u(1:3);  % Get position of sphere center
        radii = body(i).radius * [1  1  1];
        quat = body(i).u(4:7);  % Get uQuat (aka Euler Parameters).
        [xx,yy,zz] = ellipsoid(num, pos, radii, quat);
        body_hndl(i) = surf(xx,yy,zz, 'FaceColor', rand(3,1), 'EdgeColor', rand(3,1));
    end
else
    % If figure already plotted, simply update sphere positions and
    % orientations
    for i = 1 : length(body);
        num = 20;
        pos = body(i).u(1:3);
        radii = body(i).radius * [1  1  1];
        quat = body(i).u(4:7);
        [xx,yy,zz] = ellipsoid(num, pos, radii, quat);        
        set(body_hndl(i), 'XData',xx, 'YData',yy, 'ZData',zz);
    end
end
drawnow;
end