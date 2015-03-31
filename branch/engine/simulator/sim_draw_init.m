%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% sim_draw_init.m
%
% Initialize drawing of a 3D scene from a simulation class.
% Keypress and mousepress events are also handled here.

function sim_draw_init( obj )

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Keypress event from run()
    function obj = figure_keypress(~, event, obj) 
        switch event.Key
            case 'escape'   % Quit
                obj.GUI.SIM_QUIT = true;  
            case 'c'        % Toggle visible contacts 
                obj.GUI.DRAW_COLLISIONS = ~obj.GUI.DRAW_COLLISIONS;
                if obj.GUI.DRAW_COLLISIONS
                    disp ('Collision drawing is: ON');
                else
                    disp ('Collision drawing is: OFF');
                end
                if ~isempty(obj.GUI.Hg)
                    delete(obj.GUI.Hg);  % Remove graphics 
                    obj.GUI.Hg = [];     % Remove handles
                end
            case 'g'        % Toggle draw grid
                if strcmp(get(gca,'XGrid'),'off') 
                  grid on;
                else
                  grid off; 
                end
            case {'i', 's'} % Increment simulation by single step
                obj.GUI.SIM_RUN = false; 
                obj.GUI.SIM_RUN_iters = 1; 
            case 'j'        % Toggle draw joints
                obj.GUI.DRAW_JOINTS = ~obj.GUI.DRAW_JOINTS; 
            case 'q'
                obj.GUI.SIM_QUIT = true;  
                close(obj.GUI.SIM_FIG);  % Closes simulation figure
            case 'space'    % Toggle run
                obj.GUI.SIM_RUN = ~obj.GUI.SIM_RUN;
                if obj.GUI.SIM_RUN
                  set(obj.GUI.SIM_FIG,'Name','RPI-MATLAB-Simulator (Running)');
                else
                  set(obj.GUI.SIM_FIG,'Name','RPI-MATLAB-Simulator');
                end

            
        end % End switch 
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Mouse event from run()
    function obj = fig_mouse_down(~,~,obj)
      xy = get(obj.GUI.SIM_FIG,'CurrentPoint');  % Mouse position on button down
      obj.GUI.m_xi = xy(1); 
      obj.GUI.m_yi = xy(2); 
      [obj.GUI.f_AZi obj.GUI.f_ELi] = view;         % View on button down  
      set(obj.GUI.SIM_FIG,'WindowButtonMotionFcn',{@fig_mouse_motion, obj});
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Mouse event from run()
    function obj = fig_mouse_up(~,~,obj)
      set(obj.GUI.SIM_FIG,'WindowButtonMotionFcn',[]);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Mouse event from run()
    function obj = fig_mouse_motion(~,~,obj)
        
      % Rotate 3D
      xy = get(obj.GUI.SIM_FIG,'CurrentPoint'); 
      az = obj.GUI.f_AZi + obj.GUI.m_xi - xy(1);
      el = obj.GUI.f_ELi + obj.GUI.m_yi - xy(2);
      view(az, el);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Mouse event from run()
    % ZOOM
    function obj = fig_mouse_scroll(src, evnt, obj) % "axis equal" is currently breaking this
        ai = axis;  
        da = .05;    % 2*5 percent change when zooming
        if (evnt.VerticalScrollCount < 1) % Zoom in
            x1 = ai(1) + da*(ai(2)-ai(1)); 
            x2 = ai(2) - da*(ai(2)-ai(1));
            y1 = ai(3) + da*(ai(4)-ai(3));
            y2 = ai(4) - da*(ai(4)-ai(3));
            z1 = ai(5) + da*(ai(6)-ai(5));
            z2 = ai(6) - da*(ai(6)-ai(5));
        else                              % Zoom out
            x1 = ai(1) - da*(ai(2)-ai(1)); 
            x2 = ai(2) + da*(ai(2)-ai(1));
            y1 = ai(3) - da*(ai(4)-ai(3));
            y2 = ai(4) + da*(ai(4)-ai(3));
            z1 = ai(5) - da*(ai(6)-ai(5));
            z2 = ai(6) + da*(ai(6)-ai(5));
        end
        axis([x1 x2 y1 y2 z1 z2]);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Initialize drawing
    disp('Starting simulation...');
    disp('     [space]: start / stop simulation');
    disp('     [Esc]: halt simulation');
    disp('     ''q'': halt, and close figure');
    disp('     ''i'' or ''s'': iterate a single step');
    disp('     ''g'' toggle grid');
    disp('     ''c'' toggle draw contacts');

            obj.GUI.SIM_FIG = figure; 
                hold on; 
                set(obj.GUI.SIM_FIG,...
                    'NumberTitle', 'off', ...
                    'Name', 'RPI-MATLAB-Simulator', ...
                    'ToolBar', 'none', ...
                    'Color', [1 1 1], ...
                    'KeyPressFcn',          {@figure_keypress, obj}, ...
                    'WindowButtonDownFcn',  {@fig_mouse_down, obj}, ...
                    'WindowButtonUpFcn',    {@fig_mouse_up, obj}, ...
                    'WindowScrollWheelFcn', {@fig_mouse_scroll, obj} ...
                    );
    view(3);             
    xlabel('x'); ylabel('y'); zlabel('z'); 
    
    % Additional GUI inits
    obj.GUI.hGUItitle = title('');
    obj.GUI.SIM_RUN_iters = 0;
    obj.GUI.SIM_RUN = false; 
    obj.GUI.SIM_QUIT = false;
    obj.GUI.Hg = [];
    obj.GUI.DRAW_COLLISIONS = false; 
    obj.GUI.DRAW_JOINTS = false; 
    
    %% Draw 
    for i=1:length(obj.P)       % Draw all objects
        obj.P{i,1}.draw(); 
    end
    
    axis equal; 
    

end % End sim_draw_init()

