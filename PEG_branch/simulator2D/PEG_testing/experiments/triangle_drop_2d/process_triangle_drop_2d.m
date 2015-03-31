

function process_triangle_drop_2d( )

    load('PEG_ground_truth_0001.mat');
    sim_truth = simExp;
    
    %% X Distance
    % Ground truth
    figure; 
    hold on;
    t = (1:sim_truth.MAX_STEP)*.0001;
    plot(t,sim_truth.userData.position(:,1),'linewidth',1);
    xlabel('Time (s)');
    ylabel('X position (m)'); 
    grid on;
    
    % ST
    h_count=0;
    for h = 0.001:0.001:0.016
       h_count = h_count+1; 
       h_str = num2str(h); 
       load(['PEG_testing/experiments/triangle_drop_2d/experiments/ST_tri_drop_' h_str(3:end) '.mat']);
       t = (1:simExp.MAX_STEP)*h;
       h_st = plot(t,simExp.userData.position(:,1),'linewidth',2,'color',[0 0 0.3+h_count*.0437]);
    end
    
        
    % Pen
    h_count=0;
    for h = 0.001:0.001:0.016
       h_count = h_count+1; 
       h_str = num2str(h); 
       load(['PEG_testing/experiments/triangle_drop_2d/experiments/pen_tri_drop_' h_str(3:end) '.mat']);
       t = (1:simExp.MAX_STEP)*h;
       h_pen = plot(t,simExp.userData.position(:,1),'linewidth',2,'color',[0.3+h_count*.0437 0 0]);
    end
    
    % PEG
    h_count=0;
    for h = 0.001:0.001:0.016
       h_count = h_count+1; 
       h_str = num2str(h); 
       load(['PEG_testing/experiments/triangle_drop_2d/experiments/PEG_tri_drop_' h_str(3:end) '.mat']);
       t = (1:simExp.MAX_STEP)*h;
       h_peg = plot(t,simExp.userData.position(:,1),'linewidth',2,'color',[0 0.3+h_count*.0437 0]);
    end
    legend([h_st h_pen h_peg],'Stewart-Trinkle','Penalty','PEG',2);
    
    %% Y Distance
    % Ground truth
    figure; 
    hold on;
    t = (1:sim_truth.MAX_STEP)*.0001;
    plot(t,sim_truth.userData.position(:,2),'linewidth',1);
    xlabel('Time (s)');
    ylabel('Y position (m)'); 
    grid on;
    
    % ST
    h_count=0;
    for h = 0.001:0.001:0.016
       h_count = h_count+1; 
       h_str = num2str(h); 
       load(['PEG_testing/experiments/triangle_drop_2d/experiments/ST_tri_drop_' h_str(3:end) '.mat']);
       t = (1:simExp.MAX_STEP)*h;
       h_st = plot(t,simExp.userData.position(:,2),'linewidth',2,'color',[0 0 0.3+h_count*.0437]);
    end
    
        
    % Pen
    h_count=0;
    for h = 0.001:0.001:0.016
       h_count = h_count+1; 
       h_str = num2str(h); 
       load(['PEG_testing/experiments/triangle_drop_2d/experiments/pen_tri_drop_' h_str(3:end) '.mat']);
       t = (1:simExp.MAX_STEP)*h;
       h_pen = plot(t,simExp.userData.position(:,2),'linewidth',2,'color',[0.3+h_count*.0437 0 0]);
    end
    
    % PEG
    h_count=0;
    for h = 0.001:0.001:0.016
       h_count = h_count+1; 
       h_str = num2str(h); 
       load(['PEG_testing/experiments/triangle_drop_2d/experiments/PEG_tri_drop_' h_str(3:end) '.mat']);
       t = (1:simExp.MAX_STEP)*h;
       h_peg = plot(t,simExp.userData.position(:,2),'linewidth',2,'color',[0 0.3+h_count*.0437 0]);
    end
    legend([h_st h_pen h_peg],'Stewart-Trinkle','Penalty','PEG',3);
    
    
    %% Rotation
    % Ground truth
    figure; 
    hold on;
    t = (1:sim_truth.MAX_STEP)*.0001;
    plot(t,sim_truth.userData.rotation,'linewidth',1);
    xlabel('Time (s)');
    ylabel('Angle of rotation (rad)'); 
    grid on;
    
    % ST
    h_count=0;
    for h = 0.001:0.001:0.016
       h_count = h_count+1; 
       h_str = num2str(h); 
       load(['PEG_testing/experiments/triangle_drop_2d/experiments/ST_tri_drop_' h_str(3:end) '.mat']);
       t = (1:simExp.MAX_STEP)*h;
       h_st = plot(t,simExp.userData.rotation,'linewidth',2,'color',[0 0 0.3+h_count*.0437]);
    end
    
        
    % Pen
    h_count=0;
    for h = 0.001:0.001:0.016
       h_count = h_count+1; 
       h_str = num2str(h); 
       load(['PEG_testing/experiments/triangle_drop_2d/experiments/pen_tri_drop_' h_str(3:end) '.mat']);
       t = (1:simExp.MAX_STEP)*h;
       h_pen = plot(t,simExp.userData.rotation,'linewidth',2,'color',[0.3+h_count*.0437 0 0]);
    end
    
    % PEG
    h_count=0;
    for h = 0.001:0.001:0.016
       h_count = h_count+1; 
       h_str = num2str(h); 
       load(['PEG_testing/experiments/triangle_drop_2d/experiments/PEG_tri_drop_' h_str(3:end) '.mat']);
       t = (1:simExp.MAX_STEP)*h;
       h_peg = plot(t,simExp.userData.rotation,'linewidth',2,'color',[0 0.3+h_count*.0437 0]);
    end
    legend([h_st h_pen h_peg],'Stewart-Trinkle','Penalty','PEG',2);
    

end

