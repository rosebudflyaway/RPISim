function data = ProcessData(DATA, problem_indices)
    DATA = DATA.simulation_000000;
    N = length(fieldnames(DATA));
    % field names
    FN = fieldnames(DATA);
    
    if nargin < 2
        active = (1 : N);
    elseif nargin < 3
        active = problem_indices;
    end
    start = active(1);
    finish = active(end);
    % remove the 1st field (frame pictures) and 2nd field(mbs_frame000000)
    data = rmfield(DATA, FN(1:2));
    if start > 3
        data = rmfield(data, FN(3:start-1));
    end
    if finish < N
        data = rmfield(data, FN(finish+1:N));
    end
end