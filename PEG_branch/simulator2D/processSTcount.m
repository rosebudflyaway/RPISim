

load('sim_005_a_STcount.mat');
STa = percentST;
load('sim_005_b_STcount.mat');
STb = percentST;
load('sim_005_c_STcount.mat');
STc = percentST;
load('sim_005_d_STcount.mat');
STd = percentST; 
load('sim_005_e_STcount.mat');
STe = percentST;
load('sim_005_f_STcount.mat');
STf = percentST;
load('sim_005_g_STcount.mat');
STg = percentST;
load('sim_005_h_STcount.mat');
STh = percentST;
load('sim_005_i_STcount.mat');
STi = percentST;
load('sim_005_j_STcount.mat');
STj = percentST;

ST = [ STa
       STb
       STc
       STd
       STe
       STf
       STg
       STh
       STi
       STj ]; 


   
meanST = mean(ST); 
stdST = std(ST); 
plot(meanST); 
   
