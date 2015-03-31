function pg = projectedGradient(A,b,x)
% Evaluates the projected gradient as...
% pg = {dq/dx_i if x_i>0
%      {min(0,dq/dx_i) if x_i=0       
pg=A*x+b;
pg(x==0)=min(0,pg(x==0));