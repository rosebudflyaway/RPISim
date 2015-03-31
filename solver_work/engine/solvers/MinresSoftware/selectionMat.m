function Z = selectionMat(x,A_set)
% Computes and returns the selection matrix Z for a given active set A_set.
% Z acts as the restriction and prolongation operator between the full set 
% of variables and the free/working variables at a given CG iteration

% A_set is that with x==0, which is Pn = 0;
m_k=length(x)-length(A_set); %number of free variables

% C = setdiff(A,B) values in A that are not in B. The values of C are in sorted order.
% freeVars are those with Pn greater than zero;
freeVars=setdiff((1:length(x)),A_set);

Z=sparse(freeVars',(1:m_k)',ones(m_k,1),length(x),m_k);
% Z=zeros(length(x),m_k);
% for i=1:m_k
%     Z(freeVars(i),i)=1.0;
% end
end