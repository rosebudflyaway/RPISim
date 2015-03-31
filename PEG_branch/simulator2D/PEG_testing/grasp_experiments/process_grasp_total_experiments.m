


function process_grasp_total_experiments(  )

    function hndl = makeBoxPlot(xval, data, clr)
        
        m1 = median(data);
        l1 = median(data(data<m1));
        u1 = median(data(data>m1));
        
        plot(xval,m1,'*','color',clr);
        plot([xval xval], [m1 l1],'color',clr,'linewidth',2);
        hndl = plot([xval xval], [m1 u1],'color',clr,'linewidth',2);
        
        % 1st and 3rd quartile lines
        dx = .0002;
        plot([xval-dx xval+dx], [u1 u1],'color',clr,'linewidth',2);
        plot([xval-dx xval+dx], [l1 l1],'color',clr,'linewidth',2);
    end


    timeSteps = {'001'; '002'; '003'; '004'; '005'; '006'; '007'; '008'; '009';'01'};
    ST_ERR = [];
    PEN_ERR = [];
    PEG_ERR = [];
    
    figure; 
    hold on;
    grid on;
    xlabel('Time step size (s)');
    ylabel('Position error'); 
    title('2D grasp experiment error');
    
    for trial = 1:1
       % Load ground truth
       sim = load(['grasp_groundTruth_PEG_b_' num2str(trial) '_0001.mat']);
       simExp = sim.simExp;
       simTruth = simExp; 
       truth_h = simTruth.h; 
       
       for time_step = 1:5
           %% Load ST
           sim = load(['grasp_ST_b_' num2str(trial) '_' num2str(cell2mat(timeSteps(time_step))) '.mat']);
           simExp = sim.simExp;
           sim_ST = sim.simExp;
           time_values = 0:simExp.h:simExp.h*(simExp.MAX_STEP-1); 
           ST_error = zeros(size(time_values));
           for i=1:simExp.MAX_STEP
              truth_index = round((i/simExp.MAX_STEP)*simTruth.MAX_STEP);
              truth_index = max(truth_index,1);
              truth_index = min(truth_index,simTruth.MAX_STEP); 
              ST_error(i) = norm(sim_ST.userData.targetBodyPosition(i,:)-simTruth.userData.targetBodyPosition(truth_index,:)); 
           end
%            h_st = errorbar(str2num(['.' num2str(cell2mat(timeSteps(time_step)))])+.00006,mean(ST_error),std(ST_error),'linewidth',2);
%                   errorbar(str2num(['.' num2str(cell2mat(timeSteps(time_step)))])+.00006,mean(ST_error),std(ST_error),'x','linewidth',2);
            h_st = makeBoxPlot(str2num(['.' num2str(cell2mat(timeSteps(time_step)))])-.00007,ST_error,[0 0 0.8]); 
       end
       
       for time_step = 1:10
           %% Load ST
           %load(['grasp_ST_b_' num2str(trial) '_' num2str(cell2mat(timeSteps(time_step))) '.mat']);
           %sim_ST = simExp;
           sim = load(['grasp_Pen_b_' num2str(trial) '_' num2str(cell2mat(timeSteps(time_step))) '.mat']);
           simExp = sim.simExp;
           sim_Pen = sim.simExp;
           sim = load(['grasp_PEG_b_' num2str(trial) '_' num2str(cell2mat(timeSteps(time_step))) '.mat']);
           simExp = sim.simExp;
           sim_PEG = sim.simExp;
           % Calculate error at each experiment timestep 
           time_values = 0:simExp.h:simExp.h*(simExp.MAX_STEP-1); 
           %ST_error = zeros(size(time_values));
           Pen_error = zeros(size(time_values));
           PEG_error = zeros(size(time_values));
           for i=1:simExp.MAX_STEP
              truth_index = round((i/simExp.MAX_STEP)*simTruth.MAX_STEP);
              truth_index = max(truth_index,1);
              truth_index = min(truth_index,simTruth.MAX_STEP); 
              %ST_error(i) = norm(sim_ST.userData.targetBodyPosition(i,:)-simTruth.userData.targetBodyPosition(truth_index,:)); 
              Pen_error(i) = norm(sim_Pen.userData.targetBodyPosition(i,:)-simTruth.userData.targetBodyPosition(truth_index,:)); 
              PEG_error(i) = norm(sim_PEG.userData.targetBodyPosition(i,:)-simTruth.userData.targetBodyPosition(truth_index,:)); 
           end
           
           %errorbar(str2num(['.' num2str(cell2mat(timeSteps(time_step)))]),mean(ST_error),std(ST_error));
%            h_pen = errorbar(str2num(['.' num2str(cell2mat(timeSteps(time_step)))]),mean(Pen_error),std(Pen_error),'r','linewidth',2);
%                    errorbar(str2num(['.' num2str(cell2mat(timeSteps(time_step)))]),mean(Pen_error),std(Pen_error),'rx','linewidth',2);
%            h_peg = errorbar(str2num(['.' num2str(cell2mat(timeSteps(time_step)))])-.00006,mean(PEG_error),std(PEG_error),'color',[0 .8 0],'linewidth',2);
            h_pen = makeBoxPlot(str2num(['.' num2str(cell2mat(timeSteps(time_step)))]),Pen_error,'red');
            h_peg = makeBoxPlot(str2num(['.' num2str(cell2mat(timeSteps(time_step)))])+.00007,PEG_error,[0 .8 0]); 
                   %errorbar(str2num(['.' num2str(cell2mat(timeSteps(time_step)))])-.00006,mean(PEG_error),std(PEG_error),'x','color',[0 .8 0],'linewidth',2);
           
       end
       
       
       
    end
    
    
    legend([h_st h_pen h_peg],'ST','Corrective','PEG',1);
    %axis([0 0.0110 -.4 .7])
    
end







