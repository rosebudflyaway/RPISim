%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% sim_draw_collisions.m
%
% Draws contact points between bodies in collision

function sim_draw_collisions( obj )

    C = obj.Contacts;
    
    % Clear previous collisions
    if ~isempty(obj.GUI.Hg)
        obj.GUI.Hg(obj.GUI.Hg == 0) = [];
        delete(obj.GUI.Hg); 
    end
    
    figure(obj.GUI.SIM_FIG); 
    if strcmp(obj.SOLVER,'CDA')
        hi=0; % handle index
        % Re-initialize size of Hg
        obj.GUI.Hg = zeros(3*length(obj.Contacts),1);
        % Draw all subcontact collisions
        for c=1:length(C)
            
    % Only draw 'EE' edge-edge collisions ............................
%                 if strcmp(C(c).collision_type,'VF')
%                     continue;
%                 end
%                 disp(['e-e (' num2str(c) ') : ' num2str(C(c).psi_n) ', ' num2str(C(c).normal')]);
    
            p1 = obj.P{C(c).body_1}.u + C(c).p1;  
            for sc = 1:size(C(c).p2,2)
                hi=hi+1; 
                p2 = obj.P{C(c).body_2}.u + C(c).p2(:,sc);
                hold on;
                if C(c).psi_n(sc) < 0
                    %disp([' Neg Psi: ' num2str(C(c).psi_n(sc))]);
                    obj.GUI.Hg(hi) = plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'r');
                else
                    obj.GUI.Hg(hi) = plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'color',[.2 .5 1]);
                end
                % also plot squares around the contact points
                if strcmp(C(c).collision_type,'EE')
                    obj.GUI.Hg(hi+1) = plot3(p1(1),p1(2),p1(3),'go');
                    obj.GUI.Hg(hi+2) = plot3(p2(1),p2(2),p2(3),'go');
                else
                    obj.GUI.Hg(hi+1) = plot3(p1(1),p1(2),p1(3),'gs');
                    obj.GUI.Hg(hi+2) = plot3(p2(1),p2(2),p2(3),'gs');
                end
                hi = hi+2; 
            end
        end
    else
        hi=0;
        obj.GUI.Hg = zeros(3*length(C),1);
        % Draw all collisions
        for c=1:length(C)
            for sc = 1:size(C(c).p2,2)
                hi=hi+1; 
                p1 = obj.P{C(c).body_1}.u + C(c).p1; 
                p2 = obj.P{C(c).body_2}.u + C(c).p2(:,sc);
                hold on;
                obj.GUI.Hg(hi) = plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'r');
                obj.GUI.Hg(hi+1) = plot3(p1(1),p1(2),p1(3),'gs');
                obj.GUI.Hg(hi+2) = plot3(p2(1),p2(2),p2(3),'gs');
                hi = hi+2;
            end
        end
    end

end % End sim_draw_collisions()

