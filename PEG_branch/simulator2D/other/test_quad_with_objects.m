


function qTree = test_quad_with_objects( )

    %% Read in objects
    figure; hold on; grid on; view(3); 
    pcl_obs = read_pcl_data('./pcl_data_obs.txt',4,[]); 
    pcl_obj = read_pcl_data('./pcl_data_obj.txt',0.2,[]);
    axis equal; rotate3d; 

    pcl_obs_1 = pcl_obs(pcl_obs(:,2)<0,:);
    pcl_obs_2 = pcl_obs(pcl_obs(:,2)>0,:);
    conv_hull_obs_1 = convhull(pcl_obs_1(:,1), pcl_obs_1(:,2));
    conv_hull_obs_2 = convhull(pcl_obs_2(:,1), pcl_obs_2(:,2));
    conv_hull_obj = convhull(pcl_obj(:,1), pcl_obj(:,2));

    figure; hold on;
    plot(pcl_obs_1(:,1),pcl_obs_1(:,2),'*');
    plot(pcl_obs_1(conv_hull_obs_1,1),pcl_obs_1(conv_hull_obs_1,2),'r-');
    
    plot(pcl_obs_2(:,1),pcl_obs_2(:,2),'*');
    plot(pcl_obs_2(conv_hull_obs_2,1),pcl_obs_2(conv_hull_obs_2,2),'r-');

    plot(pcl_obj(:,1),pcl_obj(:,2),'g*'); 
    plot(pcl_obj(conv_hull_obj,1),pcl_obj(conv_hull_obj,2),'black-'); 
    
    axis equal;
    
    % Store object vertex data (in 2D)
    obs1 = [pcl_obs_1(conv_hull_obs_1,1),pcl_obs_1(conv_hull_obs_1,2)];
    obs2 = [pcl_obs_2(conv_hull_obs_2,1),pcl_obs_2(conv_hull_obs_2,2)];
    obj = [pcl_obj(conv_hull_obj,1),pcl_obj(conv_hull_obj,2)];
    
    
    %% Initialize quad tree
    minX = min( [obs1(:,1); obs2(:,1); obj(:,1)] );
    maxX = max( [obs1(:,1); obs2(:,1); obj(:,1)] );
    minY = min( [obs1(:,2); obs2(:,2); obj(:,2)] );
    maxY = max( [obs1(:,2); obs2(:,2); obj(:,2)] );
    
    qTree = quadNode( [mean([minX,maxX]) mean([minY,maxY])], ...   % Center of grid
                      1.2*max([abs(maxX-minX) abs(maxY-minY)])/2, ...   % The 1.2 increases the grid size by 20%
                      0 );                      % 0 marks this node as the root of the tree
    qTree = quadSplitNode( qTree );
    
    % Refine quad tree
    body_obj = Body(obj(:,1), obj(:,2));
    body_obs1 = Body(obs1(:,1), obs1(:,2));
    body_obs2 = Body(obs2(:,1), obs2(:,2));
    bodies = [body_obj body_obs1 body_obs2]; 
    for b=1:length(bodies)
       bodies(b).bodyID = b;  
    end
    qTree = refineQuadTree( bodies, [], qTree, 3 );     % 5 is the max depth of tree 
    
    displayQuadTree( qTree );          
    
    %% Run experiment 
    %experiment_withQuad( obs1, obs2, obj, qTree );

end










