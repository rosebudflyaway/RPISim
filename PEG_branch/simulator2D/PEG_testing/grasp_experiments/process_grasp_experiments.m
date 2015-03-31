


function process_grasp_experiments(  )

    % Load a ground truth
    load('grasp_groundTruth_b_1.mat');
    sim_truth = simTruth; 
    h_truth = sim_truth.h; 
    
    
    %% Load an experiment
    figure; 
    hold on; 
    xlabel('Time');
    ylabel('Euclidean distance position error'); 
    
    hdex = {'0001'; '0002'; '0003'; '0004'; '0005'; '0006'; '0007'; '0008'; '0009'; '001'; '002'; '003'; '004'; '005'; '006'; '007'; '008'; '009';'01'};
    %% STEWART-TRINKLE
    for hd = 10:14
        load(['grasp_ST_b_1_' num2str(cell2mat(hdex(hd))) '.mat']);  
        sim = simExp; 

        % Calculate error at each experiment timestep 
        time_values = 0:sim.h:sim.h*(sim.MAX_STEP-1); 
        body_error = zeros(size(time_values));
        for i=1:sim.MAX_STEP
           truth_index = round((i/sim.MAX_STEP)*sim_truth.MAX_STEP);
           truth_index = max(truth_index,1);
           truth_index = min(truth_index,sim_truth.MAX_STEP); 
           body_error(i) = norm(sim.userData.targetBodyPosition(i,:)-sim_truth.userData.targetBodyPosition(truth_index,:)); 
        end

        %h_st = plot(time_values,body_error,'--','color',[0 0 .4+((hd-10)*.15)],'linewidth',2);
        h_st = plot(time_values,body_error,'color',[0 0 .4+((hd-10)*.15)],'linewidth',2);
    end
    
    %% PENALTY
    hd_count = 1;
    for hd = [10:13 15 18]
        %try 
            sim = load(['grasp_Pen_b_1_' num2str(cell2mat(hdex(hd))) '.mat']);  
            sim = sim.simExp; 

            % Calculate error at each experiment timestep 
            time_values = 0:sim.h:sim.h*(sim.MAX_STEP-1); 
            body_error = zeros(size(time_values));
            for i=1:sim.MAX_STEP
               truth_index = round((i/sim.MAX_STEP)*sim_truth.MAX_STEP);
               truth_index = max(truth_index,1);
               truth_index = min(truth_index,sim_truth.MAX_STEP); 
               body_error(i) = norm(sim.userData.targetBodyPosition(i,:)-sim_truth.userData.targetBodyPosition(truth_index,:)); 
            end

            %h_pen = plot(time_values,body_error,'color',[.5+((hd-10)*.16) 0 0],'linewidth',1);
            h_pen = plot(time_values,body_error,'color',[.3+(hd_count)*.1 0 0],'linewidth',2);
            hd_count = hd_count+1;
        %catch
        %    disp(['No such file: ' 'grasp_Pen_b_1_' num2str(cell2mat(hdex(hd))) '.mat'])
        %end
    end
    
    %% PEG
    for hd = 10:19
        load(['grasp_PEG_b_1_' num2str(cell2mat(hdex(hd))) '.mat']);  
        sim = simExp; 

        % Calculate error at each experiment timestep 
        time_values = 0:sim.h:sim.h*(sim.MAX_STEP-1); 
        body_error = zeros(size(time_values));
        for i=1:sim.MAX_STEP
           truth_index = round((i/sim.MAX_STEP)*sim_truth.MAX_STEP);
           truth_index = max(truth_index,1);
           truth_index = min(truth_index,sim_truth.MAX_STEP); 
           body_error(i) = norm(sim.userData.targetBodyPosition(i,:)-sim_truth.userData.targetBodyPosition(truth_index,:)); 
        end

        h_peg = plot(time_values,body_error,'color',[0 .1+((hd-10)*.1) 0],'linewidth',2);
    end
    
    
    legend([h_st h_pen h_peg],'ST','Corrective','PEG',2);
    
end







