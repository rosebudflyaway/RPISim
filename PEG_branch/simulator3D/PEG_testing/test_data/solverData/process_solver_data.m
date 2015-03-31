

function process_solver_data( )


load('solver_data_peg_010.mat');
load('solver_data_Pen_010.mat');

t = 1:length(simPeg.userData.problemSize);
figure;
plot(t,simPen.userData.problemSize.^2,t,simPeg.userData.problemSize.^2);
grid on;
xlabel('Time step');
ylabel('Problem size'); 
legend('Corrective','PEG',2);
title('Problem size (rows \times columns)');

figure;
plot(t,simPen.userData.Ccount,t,simPeg.userData.Ccount);
grid on;
xlabel('Time step'); 
ylabel('Number of contacts');
legend('Corrective','PEG',2);
title('Number of contacts');

mean(simPeg.userData.solveTime)
mean(simPen.userData.solveTime)

figure;
plot(t,simPen.userData.solveTime,t,simPeg.userData.solveTime);
grid on;
xlabel('Time step'); 
ylabel('Solver time (s)');
legend('Corrective','PEG',2);
title('Time taken to find solution');

% figure;
% plot(   simPeg.userData.problemSize,...
%         simPeg.userData.solveTime, 'x', ...
%         simPen.userData.problemSize,...
%         simPen.userData.solveTime, 'o');
% grid on;
% xlabel('Problem size'); 
% ylabel('Solver time (s)');
% legend('PEG','Penalty',2);


figure;
plot( t, simPeg.userData.Ccount,'r','LineWidth',2); hold on;
plot( t, simPeg.userData.Icount,'LineWidth',2); 
grid on; 
xlabel('Time step');
ylabel('Count'); 
legend('Number of contacts', 'Number of ST-traps',2);
title('Number of contacts and number of ST-traps using PEG');


% 
%     H = figure; 
%     hold on; 
%     
%     %% PEG
%     trials = 'abcdefghij';  
%     HS = ['.001'; '.002'; '.003'; '.004'; '.005'; '.006'; '.007'; '.008'; '.009'; '.010'; '.011'; '.012'; '.013'; '.014'; '.015'; '.016'];
%     colors = distinguishable_colors(10); 
%     PEG_MEAN = [];
%         for h=1:16
%             try 
%                load(['solver_data_peg_' HS(h,2:4)]);
%                err = sim.userData.VolumeOverlap;
%                err(isnan(err)) = 0; 
%                x = str2double(HS(h,:)) +.00008; % + (t-5.5)*0.00005;
%                errorbar(x, mean(err), std(err), 'x', 'linewidth', 2, 'color','red' );
%                PEG_MEAN = [PEG_MEAN mean(err)]; 
%             catch
%             end
%         end
%     
%     %% Penalty
%     trials = 'abcdefghij';  
%     HS = ['.001'; '.002'; '.003'; '.004'; '.005'; '.006'; '.007'; '.008'; '.009'; '.010'; '.011'; '.012'; '.013'; '.014'; '.015'; '.016'];
%     colors = distinguishable_colors(10); 
%     PEG_MEAN = [];
%         for h=1:16
%             try 
%                load(['solver_data_Pen_001' HS(h,2:4)]);
%                err = sim.userData.VolumeOverlap;
%                err(isnan(err)) = 0; 
%                x = str2double(HS(h,:)); % + (t-5.5)*0.00005;
%                errorbar(x, mean(err), std(err), 'x', 'linewidth', 2, 'color',colors(1,:) );
%                PEG_MEAN = [PEG_MEAN mean(err)]; 
%             catch
%             end
%         end
%     
%     
%     %% Titles
%     title('Comparison of volume of overlap error in 3D');
%     xlabel('Timestep size h (s)');
%     ylabel('Volume of overlap (m^2)');
%     
%     grid on; 
%     
    
    

end

