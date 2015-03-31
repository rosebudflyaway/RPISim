% This is the predynamics for NCP, LCP, mLCP, formulations, 
% mainly construct the Gn, Gf, M....
% Since Gb is already presented in the data file, we don't have to 
% construct it any more
function Dynamics = PreDynamicsWFormulation(obj, formulation)
%% Make sure the dimensions are matched 
% CheckDimension(obj);
% After checking all the dimensions, we don't need to do any transpose
% afterwards and all the dimensions are already modified
Vel    = obj.bodies.velocities;
Quat   = obj.bodies.quaternions;
Pos    = obj.bodies.positions;   
Forces = obj.bodies.forces;   
Mass   = obj.bodies.masses;
Inertia = obj.bodies.inertia;

% contacts
Points  = obj.contacts.points;
Normals = obj.contacts.normals;
Gap     = obj.contacts.PSI;
Mu      = obj.contacts.mu_k;
Pairs   = obj.contacts.pairs;
% For data that is not from agx, nbTotal = nb;
% For agx data,    nbTotal = length(Mass);
%                  nb      = length(Mass(states>0));
nbTotal = length(Mass);
nb = length(Mass);
% filter out the dead for the data from Agx
if obj.source > 1 
   states = obj.bodies.states;
   Mass = Mass(states > 0);
   [Inertia, ~] = h5_filter_dead(states, Inertia, 3, 'rows');
   [Quat, ~] = h5_filter_dead(states, Quat, 1, 'rows');
   [Pos, ~] = h5_filter_dead(states, Pos, 1, 'rows');
   [Vel, ~] = h5_filter_dead(states, Vel, 6, 'rows');
   [Forces, ~] = h5_filter_dead(states, Forces, 6, 'rows');
   % update to nb if there exists field "states"
   nb = length(Mass);
end 

nc = length(Gap);

nj = 0;
if isfield(obj, 'constraints')
    Jacobian = obj.constraints.Jacobian';
    Bounds   = obj.constraints.bounds';
    JointPairs = obj.constraints.pairs';
    Rows     = obj.constraints.rows';
    TimeDeri = obj.constraints.time_derivative';
    Violation = obj.constraints.violation';
    Bounds  = Bounds';
    JointPairs = JointPairs';
    nj = length(JointPairs); % number of bilateral joints 
end

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
    fricdirs = 2;
else
    fricdirs = 6;
end
Gn = sparse(6*nbTotal, nc);
Gf = zeros(6*nbTotal, fricdirs*nc); 
if isfield(obj, 'constraints')
    Gb = zeros(6*nb, sum(Rows));
end
E = sparse(nc*fricdirs, nc);
M = sparse(6*nb, 6*nb);
Minv = sparse(6*nb, 6*nb);

% Inertia M 
for i = 1 : nb
    M(6*i-5:6*i-3, 6*i-5:6*i-3) = eye(3) * Mass(i);
    M(6*i-2:6*i, 6*i-2:6*i) = Inertia(i*3-2:i*3,:);
    Minv(6*i-5:6*i-3, 6*i-5:6*i-3) = eye(3) / Mass(i);
    Minv(6*i-2:6*i, 6*i-2:6*i) = inv(Inertia(i*3-2:i*3, :));
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
        r1 = Points(i, :) - Pos(body1id, :);  % r1 in world frame
        r1 = r1';
        Gn_i1 = [n1; cross(r1, n1)];
        Gn(6*body1id-5:6*body1id, i) = Gn_i1;
    end
    if body2id > 0
        %r2 = (Points(i, :) + n2 * PSI(i)) - Pos(body2id, :);  % p2 = p1 + n * gap in world frame
        r2 = Points(i, :) - Pos(body2id, :);
        r2 = r2';
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
if isfield(obj, 'constraints')
    % bilateral constraints (joints)
    for k = 1 : nj
        % joint1id and joint2id formulate the jth joint
        joint1id = JointPairs(k, 1);
        joint2id = JointPairs(k, 2);
        % Start and Finish find the corresponding rows for each joint in the
        % Jacobian matrix
        Start = sum(Rows(1:k-1))+1;
        Finish = Start + Rows(k)-1 ;
        Gb(6*joint1id-5:6*joint1id, Start:Finish) = [Jacobian(1:3, Start:Finish) ; Jacobian(4:6, Start:Finish)];
        Gb(6*joint2id-5:6*joint2id, Start:Finish) = [Jacobian(7:9, Start:Finish) ; Jacobian(10:12, Start:Finish)];
    end
end
% Lastly, filter out the dead bodies in the Gn and Gf
if obj.source > 1
    Gn = h5_filter_dead(states, Gn, 6, 'rows');
    Gf = h5_filter_dead(states, Gf, 6, 'rows');
end
G = ConstructG(Gn, Gf, nc, fricdirs);

Dynamics = struct();
Dynamics.M = M;
Dynamics.Minv = Minv;
Dynamics.Gn = Gn;
Dynamics.PSI = PSI; 
Dynamics.Gf = Gf;
Dynamics.E = E;
Dynamics.U = U;
Dynamics.Quat = Quat;
Dynamics.Vel = Vel;
Dynamics.Forces = Forces;
Dynamics.Pos = Pos;
Dynamics.nd = fricdirs;  % friction_directions
Dynamics.nb = nb;
Dynamics.h = h;
Dynamics.G = G;

% bilateral constraints
if isfield(obj, 'constraints')
    Dynamics.Gb = Gb;
end

% like for fixed_point iteration, there is also r 
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

% Removed to the hdf5load part. 
% function [obj] = CheckDimension(obj)
% [~, FxColumn] = size(obj.bodies.forces);
% if FxColumn ~= 1 
%     obj.bodies.forces = obj.bodies.forces';
% end
% [~, InerColumn] = size(obj.bodies.inertia);
% if InerColumn ~= 3 
%     obj.bodies.inertia = obj.bodies.inertia';
%     disp(size(obj.bodies.inertia));
% end
% [~, MassColumn] = size(obj.bodies.masses);
% if MassColumn ~= 3 
%     obj.bodies.masses = obj.bodies.masses';
% end
% [~, PosColumn] = size(obj.bodies.positions);
% if PosColumn ~= 3 
%     obj.bodies.positions = obj.bodies.positions';
% end
% [~, QuatColumn] = size(obj.bodies.quaternions);
% if QuatColumn ~= 4
%     obj.bodies.quaternions = obj.bodies.quaternions';
% end
% [~, VelColumn] = size(obj.bodies.velocities);
% if VelColumn ~= 1
%     obj.bodies.velocities = obj.bodies.velocities';
% end
% [~, DepthColumn] = size(obj.contacts.depth);
% if DepthColumn ~= 1
%    obj.contacts.depth = obj.contacts.depth';
% end
% 
% [~, MuColumn] = size(obj.contacts.mu_k);
% if MuColumn ~= 1
%     obj.contacts.mu_k = obj.contacts.mu_k';
% end
% 
% [~, NormColumn] = size(obj.contacts.normals);
% if NormColumn ~= 3
%    obj.contacts.normals = obj.contacts.normals';
% end
% [~, PairColumn] = size(obj.contacts.pairs);
% if PairColumn ~= 2
%     obj.contacts.pairs = obj.contacts.pairs';
% end
% 
% [~, PointColumn] = size(obj.contacts.points);
% if PointColumn ~= 3
%    obj.contacts.points = obj.contacts.points';
% end
% end