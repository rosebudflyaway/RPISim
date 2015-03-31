
% Calculate all the gap function values, then determine which edge is in 
% penetration

function psi = test_CDA_log_penetration(B, V)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FIXME: this is just a cheat for the sawtooth %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

psi = 0.0; % penetration depth
 
for incr_b = 1:length(B)
  for incr_sb = 2:length(B(incr_b).x)
    
    this_edge = [B(incr_b).x(incr_sb-1), ...
                 B(incr_b).y(incr_sb-1), ...
                 B(incr_b).x(incr_sb), ...
                 B(incr_b).y(incr_sb)];
                  
    if (V.x >= this_edge(1) && V.x <= this_edge(3))
      % particle is in the region of this edge
        
        m_denom = this_edge(3) - this_edge(1);
        
        if (0 ~= m_denom)
          
          m = (this_edge(4) - this_edge(2)) / m_denom;
          b = this_edge(2) - m * this_edge(1);
        
          if (V.y < m * V.x + b)
            % the particle is in penetration
            
            % calculate penetration distance
            this_normal = [this_edge(4) - this_edge(2); -(this_edge(3) - this_edge(1))];
            this_normal = this_normal / norm(this_normal);

            this_r = [this_edge(1) - V.x; this_edge(2) - V.y];
          
            psi = dot(this_normal, this_r);
            
          end % if (V.y <= m * V.x + b)
        
        end % if (0 ~= m_denom)
        
    end % if (V.x > this_edge(1) && V.x <= this_edge(3))
  end % for incr_sb
end % for incr_b


