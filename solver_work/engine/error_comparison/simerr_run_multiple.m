%% Do multiple runs of simerr_evaluate
function simerr_run_multiple(num_to_run)


this_offset = 51;

for incr_o = 1:num_to_run
    
    simerr_evaluate(0.0005, 'multirun', this_offset);
    this_offset = this_offset + 50;

end


