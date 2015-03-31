function Dynamics = PreParticleDynamics(obj, formulation)
% CheckDimension(obj);
% After checking all the dimensions, we don't need to do any transpose
% afterwards and all the dimensions are already modified
Vel    = obj.bodies.velocities;  % 3*nb by 1
Pos    = obj.bodies.positions;   
Pos = Pos';                      % nb by 3
Forces = obj.bodies.forces;      % 3*nb by 1
Mass   = obj.bodies.masses;
position = obj.bodies.positions;

% contacts
Points  = obj.contacts.points;
Points = Points';
Normals = obj.contacts.normals;
Normals = Normals';
Gap     = obj.contacts.PSI;
Mu      = obj.contacts.mu_k;
Pairs   = obj.contacts.pairs;
Pairs = Pairs';
% For data that is not from agx, nbTotal = nb;
% For agx data,    nbTotal = length(Mass);
%                  nb      = length(Mass(states>0));
nb = length(Mass);  
nc = length(Gap); 
if isfield(obj, 'step_size')
    h = obj.step_size;
else if isfield(obj, 'timestep')
    h = obj.timestep;
    else
    h = obj.settings.step_size;
    end
end
% U 
U = spdiags(Mu(:, 1), 0, length(Mu), length(Mu));
%PSI
PSI = Gap;

%% Gn Gf E 
if strcmp(formulation, 'mNCP')
    nd = 2;
else
    nd = obj.num_fricdirs;
end
% U 
U = spdiags(Mu(:, 1), 0, length(Mu), length(Mu));
% PSI
PSI = Gap;

M = sparse(3*nb);
Minv = sparse(3*nb);
Gn = sparse(3*nb, nc);
Gf = zeros(3*nb, nd*nc); 
E = sparse(nc*nd, nc);

% Inertia M 
for i = 1 : nb
    M(3*i-2:3*i, 3*i-2:3) = eye(3) * Mass(i);
    Minv(3*i-2:3*i, 3*i-2:3*i) = eye(3) / Mass(i);
end

 
% unilateral contacts 
for i = 1 : nc
    %% This sort forces the invalid or ground bodies to be first and makes sure that we build the column in increasing row order
    bb = sort(Pairs(i, :));
    body1id = Pairs(i, 1);
    body2id = Pairs(i, 2);
    % active bodies not static ones
    n1 = -Normals(i, :)';
    n2 = Normals(i, :)';
   
    if body1id > 0
        %r1 = Points(i, :) - Pos(body1id, :);  % r1 in world frame   no cross product, so no need of r1 any more
        Gn(3*body1id-2:3*body1id, i) = n1;  % (-Normals)
    end
    if body2id > 0
        %r2 = (Points(i, :) + n2 * PSI(i)) - Pos(body2id, :);  % p2 = p1 + n * gap in world frame
        %r2 = Points(i, :) - Pos(body2id, :);
        Gn(3*body2id-2:3*body2id, i) = n2;  % (Normals)
    end
    % Gf
    tangent1 = arbitraryTangent(n1);   % -tangent = arbitraryTangent(n1)
    tangent2 = arbitraryTangent(n2);

    if strcmp(formulation, 'mNCP')
        if body1id > 0
            Gf(3*body1id-2:3*body1id, 2*i-1:2*i) = [tangent1, cross(n1, tangent1)]; 
        end 
        if body2id > 0
            Gf(3*body2id-2:3*body2id, 2*i-1:2*i) = [tangent2, cross(n2, tangent2)];
        end
        E((i-1)*nd+1 : i*nd, i) = 1;
    else  % linearized friction Gf
        for j = 1 : nd
            d = rot(n2 ,((j-1)/nd)*(2*pi)) * tangent2;    % Friction direction d
            if body1id > 0
                Gf(3*body1id-2:3*body1id, nd*(i-1)+j) = (d);
            end
            if body2id > 0
                Gf(3*body2id-2:3*body2id, nd*(i-1)+j) = (d);
            end
        end
        % E
        E((i-1)*nd + 1:i*nd,i) = 1;
    end
end

G = ConstructG(Gn, Gf, nc, nd);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% This part is to match the particle prarameters %%%%%%%%%%
Vel = [1; 0; -1];
M   = eye(3);
Minv = eye(3);
PSI = 0.15;
h = 0.1;
Forces = [0; 0; -10];  % only forces NOT impulses
U = 0.5;
% NU = [1; 0; -1];
% PSI = 0.15;
% h = 0.1;
% %g = sim.dynamics.Forces(3);  % g is already negative
% g = -5;
% FX = [0; 0; m*g];    % m=1 
% Minv = 1/m*[1 0 0; 0 1 0; 0 0 1];
% U = 0.5 * eye(size(U)); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Construct A and b as LCP 
MinvGn = Minv * Gn;
MinvGf = Minv * Gf;
MinvPext = Minv * (Forces * h);
A = [Gn'*MinvGn   Gn'*MinvGf  zeros(nc)
    Gf'*MinvGn   Gf'*MinvGf  E
    U            -E'         zeros(nc)];

b = [ Gn'*(Vel + MinvPext) + PSI/h;    % FX*h could be replaced if we stored Pext instead of Fext
    Gf'*(Vel + MinvPext);
    zeros(nc,1) ];
 
A_MCP = [-M                Gn        Gf  zeros(3*nb, nc);
         Gn'              zeros(nc, nc*(2+nd));
         Gf'              zeros(nd*nc, (1+nd)*nc)     E;
         zeros(nc, 3*nb)  U           -E'             zeros(nc)];
b_MCP = [M * Vel + Forces * h;
         PSI/h;
         zeros((nd+1)*nc, 1)];
     
     
Dynamics = struct();
Dynamics.M = M;
Dynamics.Minv = Minv;
Dynamics.Gn = Gn;
Dynamics.PSI = PSI; 
Dynamics.Gf = Gf;
Dynamics.E = E;
Dynamics.U = U;
Dynamics.Vel = Vel;
Dynamics.Forces = Forces;
Dynamics.Pos = Pos;
Dynamics.nd = nd;  % friction_directions
Dynamics.h = h;
Dynamics.G = G;
Dynamics.A_LCP = A;
Dynamics.b_LCP = b;
Dynamics.A_MCP = A_MCP;
Dynamics.b_MCP = b_MCP;
Dynamics.z0_LCP = zeros(length(b), 1);
Dynamics.z0_MCP = zeros(length(b_MCP), 1);
Dynamics.position = position;
end

function G = ConstructG(Gn, Gf, nc, nd)
for i = 1 : nc
    G(:, (1+nd)*(i-1)+1:(1+nd)*i) = [Gn(:, i), Gf(:, nd*(i-1)+1:nd*i)];
end
G = sparse(G);
% figure; 
% spy(Gn);
% title('Gn');
% figure;
% spy(Gf);
% title('Gf');
end
 