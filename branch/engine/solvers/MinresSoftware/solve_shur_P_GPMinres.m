function [v,x_k, resid, res_its, objs, Aset_size, pGrad_mean_std, res_story,its_story, dv_story] = solve_shur_P_GPMinres(M,D,kk,c,iters)
%   NOTE: all variables are bounded below by zero and unbounded above.
%some parameters for the method. I do not know what values these should
%have, so they are made-up for now.

%N_gpcg_max=100;%100
N_gpcg_max=iters;   % 50
N_minres_max=10; %10
N_gp_max=5; %20
mu=0.1; %0.1
eta1=0.01;%?? %0.01
N_PS_max=5; %20
% algorithm line 19
tau=1e-9;%?? %1e-6
gdiff= 0.000001;

% the size of D matrix is 6*num_of_bodies X 3*num_of_contacts
% of which this is the Gn and Gf altogether as G


% nvars = 6*num_of_bodies
% nconstr = 3*num_of_contacts
[nvars,nconstr] = size (D);

% new velocity to be returned
v = zeros(nvars,1);
% this is gamma_k (The contact force to be solved) 
% nconstr/3 = num_Of_contacts     
x_k = zeros(nconstr/3,1);
% maybe here is just the direction ???
% when calling this function in the main, it was -D, not D;
%D=-D;

% kk = fh + M*v_old;
% This is r on page 6, r = bi + D'inv(M)f. here c = psi/h;
% disp(size(c));
% disp(size(D'*(M\kk)));
% pause;
D=D(:,1:3:size(D,2));
b=c+D'*(M\kk);

% This is to get the 1st 4th 7th 10th (3*N+1)th element in the b matrix;
% which actually is the normal force 
%b=b(1:3:size(b,1));

% This is to get the 1st 4th 7th 10th column in the D matrix, of which
% actually is Gn

A=D'*(M\D);           % this is N on page 6

% v^(l+1) and v^(l) derived from the Newton-Euler equation
v=M\(kk + D*x_k);     % this is v^(l+1) = v^l + inv(M)*Gn*Pn + inv(M)*f_ext;

%resid=zeros(1,N_gpcg_max);
resid=[];
res_its=zeros(0,2);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
objs=[];%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Aset_size=[];%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pGrad_mean_std=zeros(0,2);

k=1;
while k<=N_gpcg_max  % 50
    x_start=x_k;
    y_j=x_k;
    obj_deltas=zeros(N_gp_max,1); % N_gp_max = 5;
    for j=1:N_gp_max
        grad_p=projectedGradient(A,b,y_j);
        alpha=grad_p'*grad_p/(grad_p'*A*grad_p);
        
        while (true)
            % equation (15)
            y_j1=projectOp(y_j-alpha*grad_p);
            
            % The next is equation (16)
            if obj_func(A,b,y_j1)<=obj_func(A,b,y_j)+mu*dot(grad_p,y_j1-y_j)% should the dot product use grad_p or p?
                break;
            end
            % if equation (16) is not satisfied, then alpha = 1/2 * alpha;
            alpha=alpha/2;
        end
        
        if isequal(Aset(y_j),Aset(y_j1))  % this is equation (17)
            break; %stuck on a face, switch to CG for faster search
        end
        obj_deltas(j)=obj_func(A,b,y_j)-obj_func(A,b,y_j1);
        
        % below is equation (18)
        if ( obj_deltas(j)<=eta1*max(obj_deltas(1:j-1)) )
            %convergence not peppy, bail out, move on to CG
            break;
        end
        y_j=y_j1;
    end
    % x_k here is the gamma_k
    x_k=y_j1;
    
    %proceed to unconstrained minres
    
    %Here the active set is the parts with x_k =0, which is the P_contact = 0;
    % the contact force is zero
    activeSet=Aset(x_k); %identify active set
    Aset_size=[Aset_size;length(activeSet)];
    
    % C = setdiff(A, B) values in A that are not in B, the values in C are in
    % sorted order
   % freeSet = setdiff(1:length(x_k),activeSet); %Free set
    
    Z_k = selectionMat(x_k,activeSet); %identify selection matrix
    
    % on page 8 definition of A_k
    A_k=Z_k'*A*Z_k;
    % Actually this is r_k on page 8, and (A*x_k + b) is the nabla_q(gamma_k)
    b_k=Z_k'*(A*x_k+b);
    
    %now solve some subproblem with minres
    
    % minres solvers A_k * w_k = -b_k  (w_k is the direction)
    % tau = tolerance, N_minres = max_iteration [] [] = M1, M2,
    % preconditioner  zeros(size(b_k)) is x0, the initial guess;
    [ww,FLAG,RELRES,ITER,RESVEC]=minres(A_k,-b_k,tau,N_minres_max,[],[],zeros(size(b_k)));
    %Conditionals
    
    % ww is the w_k actually, which is the direction: by  A_k * w_k = -b_k
    d_k=Z_k*ww; %prolongation  should be the length or dimension of delta x
    i=0;  
    alpha=1.0;
    while (true)
        x_k1=projectOp(x_k+alpha*d_k);
        i=i+1;
        gp=projectedGradient(A,b,x_k);%??remove?
        % this is equation(16)
        if( i==N_PS_max || obj_func(A,b,x_k1)<=obj_func(A,b,x_k)+mu*dot(gp,x_k1-x_k) )
            break;
        end
        alpha=alpha/2;
    end
    objs=[objs; obj_func(A,b,x_k1)];
    grad_p=projectedGradient(A,b,x_k1);
    pGrad_mean_std=[pGrad_mean_std;  mean(abs(grad_p)), std(abs(grad_p))];
    err=norm(A*x_k1+b);
    resid=[resid; RESVEC(2:numel(RESVEC)) ];
    if k==1
        res_its=[ITER RESVEC(ITER+1)];
    else
        res_its=[res_its; res_its(k-1,1)+ITER RESVEC(ITER+1)];
    end
    
    testres = A*x_k1+b;
    %res_story(k)=norm( (testres - projectOp(testres)), 2);
    res_story(k)=norm( (x_k1-projectOp(x_k1-gdiff*testres))/gdiff,2);
    its_story(k)=j+ITER;
    if err<tau
        x_k=x_k1;%???  
        break; %Approximation to solution x_star found
    end
    %dx=x_k1-x_k;
    dx=x_k1-x_start;
    dv_story(k)=norm(M\(D*dx),2);
    x_k=x_k1;
    k=k+1;
    vold=v;
    v=M\(kk + D*x_k);
    ddv=v-vold;
end %the GPCG loop
its_story=cumsum(its_story);
end


