function pv = projectOp(v)
%Returns the projection of v on the feasible set, v>=0
pv=v;
pv(v<0)=zeros(length(nonzeros(v<0)),1);