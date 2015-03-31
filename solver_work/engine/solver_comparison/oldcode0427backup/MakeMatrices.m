% This function is mainly used to construct the matrices we need in the 
% solver. Especially the Gn and Gf matrix.
function obj = MakeMatrices(obj, formulation)
cur_fm = obj.cur_fm;
% bodies
Pos    = cur_fm.bodies.positions;
Quat   = cur_fm.bodies.quaternions;
Vel    = cur_fm.bodies.velocities;
Forces = cur_fm.bodies.forces;
Mass   = cur_fm.bodies.masses;
Inertia = cur_fm.bodies.inertia;

% contacts 
Points  = cur_fm.contacts.points;
Normals = cur_fm.contacts.normals;
% Data from ODE is using penetration depth. 
Gap     = -cur_fm.contacts.depth;
Mu      = cur_fm.contacts.mu_k;
Pairs   = cur_fm.contacts.pairs;
h = cur_fm.step_size;

% Include the part of constraints if there is 
% if ~isempty(cur_fm.constraints)
%     Jacob = cur_fm.constraints.Jacobian;
%     Bounds = cur_fm.constraints.bounds;
%     Cons_Pairs = cur_fm.constraints.pairs;
%     Rows = cur_fm.constraints.rows;
%     Time_deri = cur_fm.constraints.time_derivative;
%     Violation = cur_fm.constraints.violation;
% end

nb = length(Mass);
nc = length(Gap); 

% Forces
FX = Forces;
% U 
U = diag(Mu);
% NU
NU = Vel;
%PSI
PSI = Gap;

%% Gn Gf E 
if strcmp(formulation, 'mNCP')
    fricdirs = 2;
else
    fricdirs = 7;
end
Gn = zeros(6*nb, nc);
Gf = zeros(6*nb, fricdirs*nc);
E = zeros(nc*fricdirs, nc);
M = zeros(6*nb, 6*nb);

% Inertia M 
for i = 1 : nb
    M(6*i-5:6*i-3, 6*i-5:6*i-3) = eye(3) * Mass(i);
    M(6*i-2:6*i, 6*i-2:6*i) = Inertia(i*3-2:i*3,1:3);
end

for i = 1 : nc
    body1id = Pairs(i, 1);
    body2id = Pairs(i, 2);
    % active bodies not static ones
    if body1id > 0
        n1 = -Normals(i, :)';
        r1 = Points(i, :)' - Pos(body1id, :)';  % r1 in world frame
        Gn_i1 = [n1; cross(r1, n1)];
        Gn(6*body1id-5:6*body1id, i) = Gn_i1;
    end
    if body2id > 0
        n2 = Normals(i, :)';
        r2 = (Points(i, :)' + n2 * PSI(i)) - Pos(body2id, :)';  % p2 = p1 + n * gap in world frame
        Gn_i2 = [n2; cross(r2, n2)];
        Gn(6*body2id-5:6*body2id, i) = Gn_i2;
    end
    
    % Gf
    tangent = arbitraryTangent(n2);
    if strcmp(formulation, 'mNCP')
       if body1id > 0
           d = [tangent,  cross(n1, tangent)];
           Gf(6*body1id-5:6*body1id, 2*i-1:2*i) = [d; cross(r1, d(:, 1)), cross(r1, d(:, 2))];
       end 
       if body2id > 0 
           d = [tangent, cross(n2, tangent)];
           Gf(6*body2id-5:6*body2id, 2*i-1:2*i) = [d; cross(r2, d(:, 1)), cross(r2, d(:, 2))];
       end
           
    else  % linearized friction Gf
        for j = 1 : fricdirs
            d = rot(n2 ,((j-1)/fricdirs)*(2*pi)) * tangent;    % Friction direction d
            if body1id > 0
                Gf(6*body1id-5:6*body1id, fricdirs*(i-1)+j) = [d; cross3(r1,d)];
            end
            if body2id > 0
                Gf(6*body2id-5:6*body2id, fricdirs*(i-1)+j) = [d; cross3(r2,d)];
            end
        end
         % E
         E((i-1)*fricdirs + 1:i*fricdirs,i) = 1;
    end
end
obj.M = M;
obj.Gn = Gn;
obj.Gf = Gf;
obj.E = E;
obj.U = U;

obj.NU = NU;
obj.nb = nb;
obj.nc = nc;
obj.nd = fricdirs;  % friction_directions
obj.h = h;
obj.FX = FX;
obj.PSI = PSI;
obj.Pairs = Pairs;
% like for fixed_point iteration, there is also r 
end