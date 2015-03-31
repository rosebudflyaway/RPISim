

% Inputs:
%           Na : Number of sides for object A
%           Nb : Number of sides for object B
function test_polygon_polygon(Na, Nb)

    % GUI variables
    mousePressed = false;
    initMousePos = [0 0]; 
    activeObject = 1;       % 1 => A ; 2 => B
    Apos = [0 0];
    Bpos = [0 0];  % Offset in body B
    
    % A is regular polygon with Na sides
    A = zeros(Na,2);
    Adeg = 2*pi/Na;
    for i=1:Na
       A(i,:) = [cos((i-1)*Adeg) sin((i-1)*Adeg)]; 
    end

    % B is a regular polygon with Nb sides
    B = zeros(Nb,2);
    Bdeg = 2*pi/Nb;
    for i=1:Nb
       B(i,:) = Bpos + [cos((i-1)*Bdeg) sin((i-1)*Bdeg)]; 
    end
     
    % Plot A
    hA = fill(A(:,1),A(:,2),'r');  hold on;
    set(hA,'faceAlpha',0.7);

    % Plot B
    hB = fill(B(:,1),B(:,2),'b');
    set(hB,'faceAlpha',0.7);
    
    
    % GUI functions
    function mouseMove (object, eventdata)
        C = get (gca, 'CurrentPoint');
        x = C(1,1);
        y = C(1,2);
        %title(gca, ['(X,Y) = (', num2str(x), ', ',num2str(y), ')']);
        if mousePressed
            deltaPos = [x y] - initMousePos; 
            switch activeObject
                case 1 % Body A
                    set(hA,'Xdata', Apos(1) + A(:,1) + deltaPos(1));
                    set(hA,'Ydata', Apos(2) + A(:,2) + deltaPos(2));
                    %tic;
                    [collision coll_contact] = cd_poly_poly_with_correction( [Apos(1)+A(:,1)+deltaPos(1) Apos(2)+A(:,2)+deltaPos(2)], ...
                                                    [Bpos(1)+B(:,1) Bpos(2)+B(:,2)] );
                                                %toc;
                case 2 % Body B
                    set(hB,'Xdata', Bpos(1) + B(:,1) + deltaPos(1));
                    set(hB,'Ydata', Bpos(2) + B(:,2) + deltaPos(2));
                    [collision coll_contact] = cd_poly_poly_with_correction( [Apos(1)+A(:,1) Apos(2)+A(:,2)], ...
                                                    [Bpos(1)+B(:,1)+deltaPos(1) Bpos(2)+B(:,2)+deltaPos(2)] );
            end
            
            % Polygon of overlap
            overlap_poly = sutherlandHodgman( [Apos(1)+A(:,1)+deltaPos(1) Apos(2)+A(:,2)+deltaPos(2)], ...
                                              [Bpos(1)+B(:,1) Bpos(2)+B(:,2)] );
            display(overlap_poly);
            if ~isempty(overlap_poly)
                display(['   Area = ' num2str(polyarea(overlap_poly(:,1), overlap_poly(:,2)))]);
            end
            
        end
        if exist('collision','var') && collision
           title('Collision!');       
        else
           title('Free');
        end
        axis([-5 5 -5 5]); grid on;
    end

    function mouseDown (object, eventdata)
        mousePressed = true;
        C = get (gca, 'CurrentPoint');
        x = C(1,1);
        y = C(1,2);
        initMousePos = [x y]; 
        if inpolygon(x,y,Apos(1)+A(:,1),Apos(2)+A(:,2))
            activeObject = 1;
        elseif inpolygon(x,y,Bpos(1)+B(:,1),Bpos(2)+B(:,2)) 
            activeObject = 2;
        else
            activeObject = -1;  % User did not click an object.
        end
    end

    function mouseUp (object, eventdata)
        mousePressed = false; 
        C = get (gca, 'CurrentPoint');
        x = C(1,1);
        y = C(1,2);
        deltaPos = [x y] - initMousePos; 
        switch activeObject
            case 1
                Apos = Apos + deltaPos;
            case 2
                Bpos = Bpos + deltaPos;
        end
    end

    % Set graphics properties for beautiful GUI
    axis equal; axis([-5 5 -5 5]); grid on;  
    set(gcf, 'WindowButtonMotionFcn', @mouseMove); 
    set(gcf, 'WindowButtonDownFcn', @mouseDown); 
    set(gcf, 'WindowButtonUpFcn', @mouseUp);
    set(gcf, 'WindowKeyPressFcn', @keyPress); 
    drawnow; 

end

