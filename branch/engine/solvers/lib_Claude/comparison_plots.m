% 
% Copyright (2012)
% Author: Claude Lacoursiere, 
% UMIT/HPC2N
% Umeå University, 
% 901 87 Umeå, 
% Sweden
% claude@hpc2n.umu.se
%


function [b, z, l] = comparison_plots (b, z, l)

  % usage:  [b, z, l] = comparison_plots (b, z, l)
  %
  % Plot results of running the Zhang Gao and block pivot algorithms
  % 
  close all;
  NCOLS = 1;
  NROWS = 3;
  index = 1;
  N = length(z.it);

  % renormalize failures
  ix =  (b.it < 0); b.it(ix) = -5;
  ix =  z.it < 0; z.it(ix) = -5;
  ix =  l.it < 0; l.it(ix) = -5;
  max_it = max([z.it;b.it;l.it]);
  min_it = min([z.it;b.it;l.it]);
  range = max_it - min_it;
  n_bins = 20;
  stride = max(1, ceil(range/n_bins));
  centers = min_it +  stride*(1:n_bins)';
  figure(1);
  subplot(NROWS, NCOLS, index);
  index = index + 1;
  hist([b.it,z.it,l.it], centers, 100);
  set (gca (), 'xlim',[min_it, max_it]);
  title('Iterations histogram: failures are negative');
  legend(b.name, z.name, l.name, 'location','northwest');
  xlabel('iterations');
  ylabel('freq in %');

  subplot(NROWS, NCOLS, index);
  index = index + 1;

  rz = find(z.it > 0 ) ; 
  rb = find(b.it > 0 ) ; 
  rl = find(l.it > 0 ) ; 
  % filter out outliers
  zm = mean(z.timing(rz)); 
  rzz = z.timing(rz) < 2*zm;
  rz = rz(rzz); 
  bm = mean(b.timing(rb)); 
  rbb = b.timing(rb) < 2*bm;
  rb = rb(rbb); 
  lm = mean(l.timing(rl)); 
  rll = l.timing(rl) < 2*lm;
  rl = rl(rll); 
  
  plot(rz, 1000*sort(z.timing(rz)), rl, 1000*sort(l.timing(rl)), rb, 1000*sort(b.timing(rb)));
  legend(z.name, l.name, b.name)
  title('Timing results (sorted, failures ignored)')
  ylabel('ms');
  subplot(NROWS, NCOLS, index);
  r = (1:N)';
  plot( r, sort(log10(z.err(r))), r, sort(log10(l.err(r))), r, sort(log10(b.err(r))));
  ylabel('Log of error');
  legend(z.name, l.name, b.name, 'location', 'northwest');
  title('Log of error (sorted)');
end%function
