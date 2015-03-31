%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_CDA_edges_plot.m
%
% 2D simulator for testing CDA method: plotting function, draw bodies

function test_CDA_edges_drawbodies(B, V)

% Calculate the epsilon values
cur_eps = test_CDA_calc_epsilon(V);
psi_eps = cur_eps / 10; % maximum penetration depth
% (penetration is ignored past this depth)


% number of manifolds (one manifold per body here)
for incr_b = 1:length(B)
  
    % number of subcontacts for this manifold
    for incr_sc = 1:length(B(incr_b).x) - 1
    
        % find unit vector of the tangent for this edge
        VP1 = [B(incr_b).x(incr_sc); B(incr_b).y(incr_sc)];
        VP2 = [B(incr_b).x(incr_sc+1); B(incr_b).y(incr_sc+1)];
        
        this_edge = VP2 - VP1;
        this_edge_tangent = this_edge / norm(this_edge);
      
        % calculate end points of halfspace boundary for edge constraint
%          hs_X = [B(incr_b).x(incr_sc) - cur_eps * this_edge_tangent(1); ...
%                  B(incr_b).x(incr_sc+1) + cur_eps * this_edge_tangent(1)];
%  
%          hs_Y = [B(incr_b).y(incr_sc) - cur_eps * this_edge_tangent(2); ...
%                  B(incr_b).y(incr_sc+1) + cur_eps * this_edge_tangent(2)];
                

        [ptA, ptB] = calc_intersect_points(VP1, VP2, cur_eps, B(incr_b));
        hs_X = [ptA(1); ptB(1)];
        hs_Y = [ptA(2); ptB(2)];
        
        % Plot extended edge (halfspace or whatever)
        plot(hs_X,hs_Y,'--', 'Color', [0.6 0.6 1.0]);
  
  
    %      % label the edge
    %      edge_text_str = strcat('e', incr_b, '_', incr_sc);
    %      edge_text_xpos = mean([B(incr_b).x(incr_sc+1) B(incr_b).x(incr_sc)]) + 0.2;
    %      edge_text_ypos = mean([B(incr_b).y(incr_sc+1) B(incr_b).y(incr_sc)]);
    %  
    %      text(edge_text_xpos, edge_text_ypos, edge_text_str);
  
  
        if (~(isfield(V, 'xlog')))
            % Determine gap distances and plot Psi_i
            v = [B(incr_b).y(incr_sc+1) - B(incr_b).y(incr_sc); ...
                 -(B(incr_b).x(incr_sc+1) - B(incr_b).x(incr_sc))];
            v = v / norm(v);
            r = [B(incr_b).x(incr_sc) - V.x; ...
                 B(incr_b).y(incr_sc) - V.y];
            psi_i = dot(v, r);     % Gap distance
  
  

            if psi_i > 0.0
                psi_color = [1 0 0];
            
            else
                psi_color = [0 1 0];
                            
            end
            
            % Only plot psi is it's within epsilon limits
            if ((psi_i >= -psi_eps) && (psi_i <= cur_eps))
                           
                plot([V.x, V.x + psi_i * v(1)], ...
                     [V.y, V.y + psi_i * v(2)], ...
                     'color', psi_color);
                
            end
  
        end
  
    end
  
    % Plot all edges in this body
    plot(B(incr_b).x, B(incr_b).y, 'linewidth', 4, 'color', [0 0 0]);
  
end
  
  
end % function test_CDA_edges_drawbodies

function [maxPT_A, maxPT_B] = calc_intersect_points(VP1, VP2, c_eps, this_B)

% Return the points defining the maximum extension of the halfspace for edge
% VP1, VP2 by c_eps



% Create a list of points where the halfspace (extended edge from VP1 and VP2)
% intersect all the circles with radius c_eps from each vertex point in the 
% current body.

pts = [];

for incr_v = 1:length(this_B.x)
    
    lc_int = line_circ_intersect([this_B.x(incr_v); this_B.y(incr_v)], ...
                                 VP1, ...
                                 VP2, ...
                                 c_eps);
    
%      rectangle('Curvature', [1 1], 'Position', [this_B.x(incr_v) - c_eps, this_B.y(incr_v) - c_eps, 2*c_eps, 2*c_eps], 'FaceColor', [1.0 0.95 0.95], 'EdgeColor', [0.6 0.5 0.5]);
    
    if (lc_int.discr >= 0)
        pts = [pts, lc_int.ptA];
    end
    
    if (lc_int.discr > 0)
        pts = [pts, lc_int.ptB];
    end
    
end

%  plot(pts(1,:), pts(2,:), 'o', 'MarkerSize', 4.0);

edge_dir = (VP2 - VP1) / norm(VP2 - VP1);


pts_proj = zeros(size(pts,2), 1);
for incr_p = 1:size(pts,2)
    pts_proj(incr_p) = dot(pts(:,incr_p) - VP2, edge_dir);
end



[max_val, max_idx] = max(pts_proj);
[min_val, min_idx] = min(pts_proj);


maxPT_A = pts(:, min_idx);
maxPT_B = pts(:, max_idx);

%  plot(maxPT_A(1), maxPT_A(2), 'o', 'MarkerSize', 8.0, 'Color', [1 0 0]);
%  plot(maxPT_B(1), maxPT_B(2), 'o', 'MarkerSize', 8.0, 'Color', [0 1 0]);

end % function calc_intersect_points
  
  
  
function out = line_circ_intersect(VPc, VP1, VP2, c_eps)

% VPc - [x;y] vertex point at the center of the circle
% VP1 - [x;y] first vertex point of the line
% VP2 - [x;y] second vertex point of the line
% c_eps - current epsilon / radius of the circle

% out .disc, .ptA, .ptB - the intersection points on the circle, and determinancy



% (From mathworld.wolfram.com/Circle-LineIntersection.html)

x1 = VP1(1) - VPc(1);
y1 = VP1(2) - VPc(2);

x2 = VP2(1) - VPc(1);
y2 = VP2(2) - VPc(2);

dx = x2 - x1;
dy = y2 - y1;

dr = sqrt(dx^2 + dy^2);

D = x1 * y2 - x2 * y1;


% discriminant:
out.discr = c_eps^2 * dr^2 - D^2;

if (out.discr >= 0)
    % single intersection point, if it's 0
    
    out.ptA = [(D * dy + sign(dy) * dx * sqrt(out.discr)) / (dr^2);
               (-D * dx + abs(dy) * sqrt(out.discr)) / (dr^2)] + VPc;
    
    
    if (out.discr > 0)
        % two intersection points
    
        out.ptB = [(D * dy - sign(dy) * dx * sqrt(out.discr)) / (dr^2);
                   (-D * dx - abs(dy) * sqrt(out.discr)) / (dr^2)] + VPc;
    
    end
end
   

end % function line_circ_intersect
