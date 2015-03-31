

% Load some bodies
load('random_bodies_b.mat');
for i=1:length(B)
    B(i).numJoints = 0;
    B(i).pos = [0; .2];
    B(i).mass = 0.1;
    B(i).J = .0001;
    B(i).nu = [0;0;0]; 
    for v=1:B(i).num_verts
       B(i).verts_local(v,:) = 0.28*B(i).verts_local(v,:);  
    end
end

% % Stewart-Trinkle
% sim = robotArm2D( 1, B(1), .001 );
% 
% % Penalty
% sim = robotArm2D( 6, B(1), .001 );

% PEG
%  simTruth = robotArm2D( 5, B(1), .0001 );
%  save('PEG_testing/grasp_experiments/grasp_data/grasp_groundTruth_b_1.mat','simTruth'); 

% 
% for b_index = 1:10
%     for h = 0.011:.001:0.011
%         simExp = robotArm2D( 5, B(b_index), h); 
%         num = num2str(h);
%         num = num(3:end);
%         %save(['PEG_testing/grasp_experiments/grasp_data/friction/PEG/grasp_friction_PEG_b_' num2str(b_index) '_' num2str(num) '.mat'],'simExp');
%         close all;
%         clear simExp; 
%     end
% end

% for b_index = 1:10
%     for h = 0.001:.001:0.01
%         try
%             simExp = robotArm2D( 6, B(b_index), h); 
%             num = num2str(h);
%             num = num(3:end);
%             save(['PEG_testing/grasp_experiments/grasp_data/friction/Pen/grasp_friction_Pen_b_' num2str(b_index) '_' num2str(num) '.mat'],'simExp');
%             close all;
%             clear simExp; 
%         catch
%         end
%     end
% end

for b_index = 1:10
    for h = 0.001:.001:0.010
        try
            simExp = robotArm2D( 1, B(b_index), h); 
            num = num2str(h);
            num = num(3:end);
            save(['PEG_testing/grasp_experiments/grasp_data/friction/ST/grasp_friction_ST_b_' num2str(b_index) '_' num2str(num) '.mat'],'simExp');
            close all;
            clear simExp; 
        catch
        end
    end
end


% for b_index = 7:10
%     simExp = robotArm2D( 5, B(b_index), 0.0001); 
%     save(['PEG_testing/grasp_experiments/grasp_data/friction/grasp_friction_groundTruth_PEG_b_' num2str(b_index) '_0001.mat'],'simExp');
%     close all;
%     clear simExp; 
% end









