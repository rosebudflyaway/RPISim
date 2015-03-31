

function H = processData( )



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

    H = figure; 
    hold on; 
    
    
    trials = 'abcdefghij';  
    HS = ['.001'; '.002'; '.003'; '.004'; '.005'; '.006'; '.007'; '.008'; '.009'; '.010'; '.011'; '.012'; '.013'; '.014'; '.015'; '.016'];
    colors = distinguishable_colors(10); 
    
    %% PENALTY
    colors = distinguishable_colors(10); 
    for t = 1 %1:10
        for h=1:16
            try
               trial = trials(t);  
               load(['pen_sim_' HS(h,2:4) '_' trial]);
               err(isnan(err)) = 0; 
               x = str2double(HS(h,:)) + (t-5.5)*0.00005;
               %H_PEN = errorbar(x, mean(err), std(err),'linewidth', 2, 'color',colors(t,:) );
               %     errorbar(x, mean(err), std(err), 'x', 'linewidth', 2, 'color',colors(t,:) );
               H_PEN = makeBoxPlot(x,err,colors(t,:)); 
            catch
            end
        end
    end
    
    
    %% PEG
    PEG_MEAN = [];
    for t = 2 %1:10
        for h=1:16
            try
               trial = trials(t);  
               load(['sim_' HS(h,2:4) '_' trial]);
               err(isnan(err)) = 0; 
               x = str2double(HS(h,:)) + (t-5.5)*0.00005;
%                H_PEG = errorbar(x, mean(err), std(err), 'linewidth', 2, 'color',[0 .7 0] );
%                     errorbar(x, mean(err), std(err), 'x', 'linewidth', 2, 'color',[0 .7 0] );
                H_PEG = makeBoxPlot(x,err,[0 .7 0]);
               PEG_MEAN = [PEG_MEAN mean(err)]; 
            catch
            end
        end
    end
            
    
    
%     
%     %% Titles
    title('Median, Q1, and Q3 of area of overlap error in 2D')
    xlabel('Timestep size h (s)');
    ylabel('Total area of overlap (m^2)');
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
    legend([H_PEN H_PEG],'Corrective','PEG',2);
    
    
    
%     %% Plot mean
%     h = 0.001 : 0.001 : 0.012;
%     err_mean = PEG_MEAN(1:12);
%     figure; plot(h,err_mean); 
%     
    
end









