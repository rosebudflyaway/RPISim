


function processCountData(  )


    H = figure; 
    hold on; 
    
    %% PEG
    trials = 'abcdefghij';  
    HS = ['.001'; '.002'; '.003'; '.004'; '.005'; '.006'; '.007'; '.008'; '.009'; '.010'; '.011'; '.012'; '.013'; '.014'; '.015'; '.016'];
    colors = distinguishable_colors(10); 
    PEG_MEAN = [];
    for t = 1:10
        for h=1:16
            try
               trial = trials(t);  
               load(['sim_' HS(h,2:4) '_' trial '_STcount2']);
               err = Icount;
               err(isnan(err)) = 0; 
               x = str2double(HS(h,:)) + (t-5.5)*0.00005;
               errorbar(x, mean(err), std(err), 'x', 'linewidth', 2, 'color',colors(t,:) );
               PEG_MEAN = [PEG_MEAN mean(err)]; 
            catch
            end
        end
    end
    
    
    %% Titles
    title('Average number of traps identified by PEG at each step');
    xlabel('Timestep size h (s)');
    ylabel('Number of traps');
    
    %Ax1 = gca;
    %Ax2 = axes('Position',get(Ax1,'Position'),'XAxisLocation','top');
    %hLine2 = line(0,0,'color','k','parent',Ax2);


%     H_leg = legend('A','B','C','D','E','F','G','H','I','J',2);
%     leg_line=findobj(H_leg,'type','Line');
%     for i = 1:length(leg_line)
%         try
%          set(leg_line(i), 'LineStyle', '-');
%          set(leg_line(i), 'Color',colors(i,:)); 
%         catch
%         end
%     end
    
    grid on; 
    
    
    
%     %% Plot mean
%     h = 0.001 : 0.001 : 0.012;
%     err_mean = PEG_MEAN(1:12);
%     figure; plot(h,err_mean); 
%     
    


end

