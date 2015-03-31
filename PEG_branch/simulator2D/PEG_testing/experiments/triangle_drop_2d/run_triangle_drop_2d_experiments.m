


function run_triangle_drop_2d_experiments( )

    % PEG
    for h=0.011:0.001:0.016
        simExp = tri_drop(5,h);
        h_str = num2str(h); 
        save(['PEG_testing/experiments/triangle_drop_2d/experiments/PEG_tri_drop_' h_str(3:end) '.mat'],'simExp'); 
        clear simExp;
        close all;
    end
    
    % Pen
    for h=0.011:0.001:0.016
        simExp = tri_drop(6,h);
        h_str = num2str(h); 
        save(['PEG_testing/experiments/triangle_drop_2d/experiments/pen_tri_drop_' h_str(3:end) '.mat'],'simExp'); 
        clear simExp;
        close all;
    end
    
    % ST
    for h=0.011:0.001:0.016
        simExp = tri_drop(1,h);
        h_str = num2str(h); 
        save(['PEG_testing/experiments/triangle_drop_2d/experiments/ST_tri_drop_' h_str(3:end) '.mat'],'simExp'); 
        clear simExp;
        close all;
    end

end

