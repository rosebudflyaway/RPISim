
% Metric from Gould and Scott, 
% ACM Transactions on Mathematical Software, Vol. 333, No. 2, Article 10,
% June 2007

%            SUM_{j \in T} k(s_ij, sh_j, a)
%  p_i(a) =  ------------------------------  , a >= 1
%                         |T|
%
%  where T is a set of test problems and
%                      
%  k(s_ij, sh_j, a) = { 1  if s_ij <= a*sh_j
%                     { 0  otherwise
% 

%% Main function
% For now, this function assumes that all tests have been executed, so 
% S_ij and Sh_j are vectors of the available values over examples j.  
% S_ij is the set of statistics for solver i on problem j.  
% Sh_j contains the best result (statistic) on problem j over all solvers.  
function prof = perf_profile(S_ij, Sh_j, A, T_size) 
    
    % For the profile, we find the value of p_i(a) over a range of A
    prof = zeros(length(A),1);
    for a = 1 : length(A)
        % k function
        if S_ij >0
            k = S_ij <= A(a)*Sh_j;
        else
            k = A(a)*abs(S_ij) >= abs(Sh_j);    % Attempt to allow negative metrics
        end
        
        % Record p_i(a) 
        prof(a) = sum(k) / T_size; 
    end

end %

