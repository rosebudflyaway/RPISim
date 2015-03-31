
% Assumes that all fields have been pre-allocated 
function solution = updateSolutionData( solution, errParams, iter, A, z, b, s, U, pn, psi_n, pf, nd, Gf, Vel) 
Name = errParams.metricName;
lambda1 = errParams.lambda1;
lambda2 = errParams.lambda2;

nc = length(psi_n);
% THE PHYSICAL NORMAL ERROR
normal_error = abs(pn'*psi_n) /nc;
% THE TANGENTIAL VELOCITY
relative_velocity = Gf' * Vel;

% THE FRICTIONAL ERROR:  PfMag*VfMag*ANGLE (ANGLE is the angle of Pf and (-Vf))
if nd > 2    
%% CASE1: transform the nd friction magnitude into x, y direction 
%         transform the nd tangential velocity into x, y direction
%         compute the error. This is to transform the result into NCP form 
%         and compute the NCP errors.
  % [friction_error, pfMag] = updateFrictionError(nc, nd, pf, relative_velocity); 
   
%% CASE 2: COMPUTE THE ERROR OF FRICTION AND VELOCITY ALONG THE nd direction, then add the error together
%          This is the LCP error rather than NCP error
%          First get the vector of the frictional force along each of the
%          nd direction
   [fric_error, ~] = updateDiscretizeFrictionError(nc, nd, pf, relative_velocity);  
   % fric_error is already normalized
   friction_error  = norm(fric_error); 

elseif nd == 2            % NCP formulation
    alpha = zeros(nc, 1);
    pfMag = zeros(nc, 1);
    VfMag = zeros(nc, 1);
    FrictionError = zeros(nc, 1);
    for i = 1 : nc
            Current_f = [pf(2*i-1:2*i, 1); 0];
            Current_vel = [relative_velocity(2*i-1:2*i, 1); 0];
            alpha(i, 1) = atan2(norm(cross(Current_f, -Current_vel)), dot(Current_f, -Current_vel));
            pfMag(i, 1) = norm(Current_f);
            VfMag(i, 1) = norm(Current_vel);
            FrictionError(i, 1) = pfMag(i, 1)*VfMag(i, 1) * alpha(i, 1);
    end
    friction_error  = norm(sum(FrictionError)) / nc;
end

% COMPUTE THE TOTAL ERROR
if nd == 2  
     solution.total_error(iter) = (friction_error + normal_error)/nc;
     %solution.total_error(iter) = norm(z' *(A * z + b)) / nc;
elseif nd > 2  
    q = A * z + b;
    ErrorMetricHandle = str2func(Name);
    %% Here are the 4 different error metrics
    %  1. norm_CCK, 2. square_CCK,  3. norm_mCCK  4. square_mCCK
    if strcmp(Name, 'norm_CCK')
        Terror = feval(ErrorMetricHandle, z, q, lambda1);
    else if strcmp(Name, 'square_CCK')
            Terror = feval(ErrorMetricHandle, z, q, lambda1);
        else if strcmp(Name, 'norm_mCCK')% modified CCK function
                Terror = feval(ErrorMetricHandle, z, q, lambda1, lambda2, friction_error);
            else if strcmp(Name, 'square_mCCK')
                    Terror = feval(ErrorMetricHancle, z, q, lambda1, lambda2, friction_error);
                else
                    error('Unrecognized error metric, please define again!');
                end
            end
        end
    end
    total_error = Terror;  % evaluated by the merit function
    solution.total_error(iter) = total_error / nc; % normalized by the number of contact
end
 
solution.z(:, iter) = z;     % The solution
solution.iterations = iter;  % The total number of iterations
%solution.friction_error(iter) = friction_error;  
%solution.normal_error(iter) = normal_error;  
solution.stick_or_slide(iter) = length(abs(s) > 0.000001) / length(s);

% Refer to the documentation for derivations 
% COMPUTE THE NORMAL ERROR IN FORM OF FB/CCK function
nPN = length(pn);
nPF = length(pf); 
yPn = A(1:nPN, :) * z + b(1:nPN);  % this is rho_n
Normal_FB_error = Chen_Chen_Kanzow(pn, yPn, lambda1);
% NORMAL 
% COMPUTE THE FRICTION ERROR IN FORM OF FB/CCK function
yPf = A(nPN+1:nPN+nPF, :) * z + b(nPN+1:nPN+nPF);
Frictional_FB_error = Chen_Chen_Kanzow(pf, yPf, lambda1);
 
solution.normFBerror(iter) = norm(Normal_FB_error) / nc;
solution.fricFBerror(iter) = norm(Frictional_FB_error) / nc;
%solution.normFBerror(iter) = (1/2*(Normal_FB_error)' * Normal_FB_error) / nc;
%solution.fricFBerror(iter) = (1/2*(Frictional_FB_error)' * Frictional_FB_error) / (nc * nd);
end

%% The following are different NCP functions, FB, CCK, modified_CCK
function fb = fischer_burmeister(x, y)
% INPUT : 
% x ------------------------  n by 1 vector
% y ------------------------  n by 1 vector 

% OUTPUT:
% fb -----------------------  n by 1 vector with the result of
%    -------------------------Fischer-Burmeister function
fb = sqrt(x.* x + y.* y) - x - y;
end

function cck = Chen_Chen_Kanzow(x, y, lambda)
% INPUT: 
% x ------------------------ n by 1 vector
% y ------------------------ n by 1 vector 
% lambda ------------------- single real number to penalize the positive

% OUTPUT: 
% cck ---------------------- n by 1 vector with the result of CCK function
cck = lambda * fischer_burmeister(x, y) + (1 - lambda) * (max(0, x) .* max(0, y));
end

function mCCK = modified_CCK(x, y, lambda1, lambda2, friction_error)
%INPUT 
% x ----------------------- n by 1 vector 
% y ----------------------- n by 1 vector 
% lambda1 ----------------- single real number for ChenChenKanzow function
% lambda2 ----------------- single real number for modified CCK function

% OUTPUT 
% mCCK -------------------- n by 1 vector with the result of mCCK function
cck = Chen_Chen_Kanzow(x, y, lambda1);
mCCK = lambda2 * cck + (1 - lambda2) * friction_error;
end 

%% The following are the merit function which evaluates the total error, user could choose 
%  which metric to use when evaluating the total error
function [normCCK] = norm_CCK(x, y, lambda)
   cck = Chen_Chen_Kanzow(x, y, lambda);
   normCCK = norm(cck);
end

function [squareCCK] = square_CCK(x, y, lambda)
   cck = Chen_Chen_Kanzow(x, y, lambda);
   squareCCK = 1/2*(cck')*cck;
end

function [n_mCCK]  = norm_mCCK(x, y, lambda1, lambda2, friction_error) 
mCCK = modified_CCK(x, y, lambda1, lambda2, friction_error);
n_mCCK = norm(mCCK);
end

function [s_mCCK] = square_mCCK(x, y, lambda1, lambda2, friction_error)
mCCK = modified_CCK(x, y, lambda1, lambda2, friction_error);
s_mCCK = 1/2*(mCCK') * mCCK;
end
