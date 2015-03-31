function obj = read_data(obj, formulation)
Pos    = obj.bodies.positions;
Quat   = obj.bodies.quaternions;
Vel    = obj.bodies.velocities;
Forces = obj.bodies.forces;
Mass   = obj.bodies.masses;
Inertia = obj.bodies.inertia;

Points  = obj.contacts.points;
Normals = obj.contacts.normals;
Gap     = -obj.contacts.depth;
Mu      = obj.contacts.mu_k;
Pairs   = obj.contacts.pairs;

Jacobian = obj.constraints.Jacobian;
Bounds   = obj.constraints.bounds;
JointPairs = obj.constraints.pairs;
Rows     = obj.constraints.rows;
TimeDeri = obj.constraints.time_derivative;
Violation = obj.constraints.violation;

Pos = Pos';
Points = Points';
Quat = Quat';
Inertia = Inertia';

Normals = Normals';
Pairs   = Pairs';
Bounds  = Bounds';
JointPairs = JointPairs';

h = obj.step_size;
%h = obj.settings.h;

nb = length(Mass);
nc = length(Gap);
nj = length(JointPairs); % number of bilateral joints 

% U 
U = zeros(nc, nc);
Mu = Mu';
for i = 1 : nc
    U(i, i) = (1/2) * sum(Mu(i, :));
end
    
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
Gb = zeros(6*nb, sum(Rows));
E = zeros(nc*fricdirs, nc);
M = zeros(6*nb, 6*nb);


% Inertia M 
for i = 1 : nb
    M(6*i-5:6*i-3, 6*i-5:6*i-3) = eye(3) * Mass(i);
    M(6*i-2:6*i, 6*i-2:6*i) = Inertia(i*3-2:i*3,:);
end


% unilateral contacts 
for i = 1 : nc
    body1id = Pairs(i, 1);
    body2id = Pairs(i, 2);
    % active bodies not static ones
    n1 = -Normals(i, :);
    n2 = Normals(i, :);
   
    if body1id > 0
        r1 = Points(i, :) - Pos(body1id, :);  % r1 in world frame
        Gn_i1 = [n1'; cross(r1, n1)'];
        Gn(6*body1id-5:6*body1id, i) = Gn_i1;
    end
    if body2id > 0
        r2 = (Points(i, :) + n2 * PSI(i)) - Pos(body2id, :);  % p2 = p1 + n * gap in world frame
        Gn_i2 = [n2'; cross(r2, n2)'];
        Gn(6*body2id-5:6*body2id, i) = Gn_i2;
    end
    
    % Gf
    tangent = arbitraryTangent(n2);
    if strcmp(formulation, 'mNCP')
        if body1id > 0
            d = [tangent,  cross(n1', tangent)];
            Gf(6*body1id-5:6*body1id, 2*i-1:2*i) = [d; cross(r1', d(:, 1)), cross(r1', d(:, 2))];
        end
        if body2id > 0
            d = [tangent, cross(n2', tangent)];
            Gf(6*body2id-5:6*body2id, 2*i-1:2*i) = [d; cross(r2', d(:, 1)), cross(r2', d(:, 2))];
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

obj.dynamics.M = M;
obj.dynamics.Gn = Gn;
obj.dynamics.Gf = Gf;
obj.dynamics.E = E;
obj.dynamics.U = U;
obj.dynamics.Quat = Quat;
obj.dynamics.nd = fricdirs;  % friction_directions
obj.dynamics.h = h;

% bilateral constraints
obj.dynamics.Gb = Gb;
 
% like for fixed_point iteration, there is also r 
end