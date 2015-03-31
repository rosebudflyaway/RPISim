% clear all;

% Define body edges here. 
B.x = [-2 -1 1 4];
B.y = [-1 1 1 -1];

V.x = .5;  % Initial point of 
V.y = 3;
V.m = 1;    % mass
V.nu = [-0.3; -1];  % velocity

V.Fext = [0; 0];

h = 0.01; % step size


test_CDA_edges_plot(B, V);


run_count = 0;

while 1==1
  
    if run_count <= 0
      [x,y,button] = ginput(1); 
    end

    if button == 1
      V.x = x;
      V.y = y;
    
      V.nu = [-0.3; -1];
      button = 0;
    end
    
    if button == 2
      % add a half second of iterations to run_count
      %run_count = floor(0.5 / h);
      run_count = 100; 
%         run_count = 10e5;
%         run_count = 1;
           button = 0;
    end
      
    if button == 3
        break; 
    end
    
    test_CDA_edges_plot(B, V);
    [V.nu, t_solver] = test_CDA_dynamics(B, V, h);
    
    if run_count > 0
      V.x = V.x + V.nu(1) * h;
      V.y = V.y + V.nu(2) * h;
    
      run_count = run_count - 1;
    
      % Saturate velocity when it blows up
      if (norm([V.x; V.y]) > 100)
        run_count = 0;
      end
    
      
    end
end
