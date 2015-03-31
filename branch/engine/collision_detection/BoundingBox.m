

% Represents and Axis-Aligned Bounding Box AABB
classdef BoundingBox < handle
    properties 
        % Representation
        minimum     % Vector to minimum (x,y,z) corner
        maximum     % Vector to maximum (x,y,z) corner
        
        % Graphics handles
        BBcolor
        drawn
        L1
        L2
        L3
        L4
        L5
        L6
        L7
        L8
        L9
        L10
        L11
        L12
    end
        
    methods
        %% Constructor
        function obj = BoundingBox()
           minimum = [-1; -1; -1];
           maximum = [ 1;  1;  1];
           
           BBcolor = [0 1 0]; 
           drawn = false;
        end

        %% Modifiers
        function obj = setMin(obj, m)
         	obj.minimum = m; 
        end
        
        function obj = setMax(obj, m)
            obj.maximum = m;
        end
        
        % Given a mesh body, determines a non-tight BB
        function obj = boundMesh(obj, m)
            minimum = [min(m.Xdata); min(m.Ydata); min(m.Zdata)];
            maximum = [max(m.Xdata); max(m.Ydata); max(m.Zdata)];
            
            vec = maximum-minimum;
            length = sqrt(sum(vec.^2));
            obj.minimum = minimum -0.05*length*vec;     % Increase BB by 10%
            obj.maximum = maximum +0.05*length*vec;     % along diagonal 
        end
        
        
        function obj = draw(obj)
            if ~drawn
               m = obj.minimum;
               M = obj.maximum;
               L1 = plot3([m(1) M(1)],[m(2) m(2)],[m(3) m(3)],'LineColor',BBcolor);
                
            else
                
                
            end
        end
    end
end
        











        
        
        