%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% sim_RecordContact.m
%
% Stores contact information at the current step.

% con.bodies     <-- Need to decide on all or just active bodies --save all in bodies and active ones in Contacts with their Ids

% con.Constraints   <-- Always empty for now -- maybe set up some  alternatively [mu_static(1,1) mu_kinetic(1,1)]
% con.Contacts 
% con.Settings 

% position [(1,3*nb)]
% quaternion [(1,4*nb)]
% velocities [(1,6*nb)]
% contactPairs [(1,3*nc)]    assumes sorted contacts based on pairs
% contactPoints [contactPoint(1,3) contactNormal(1,3) psi(1,1) mu(1,1)]  alternatively [mu_static(1,1) mu_kinetic(1,1)]

function sim_CollectData( obj )

step = obj.STEP;
strframe = strcat('frame', num2str(step));

%% bodies
% here we will save the info of dynamic bodies, for static ones, there is
% no need to save the mass, velocity, inertia and so on. 
% Assume that the static bodies are always BEFORE the dynamic bodies of the body objects array.
NTB  =  length(obj.P);
STA  =  obj.STATIC_BODY;  % number of static bodies
NB  = NTB - STA;          % number of dynamic bodies

NU  = zeros(NB*6, 1);
POSITION = zeros(NB, 3);
QUATERNION = zeros(NB, 4);
FORCE  =  zeros(6*NB, 1);
MASS = zeros(NB, 1);
INERTIA = zeros(3*NB, 3);

for i = 1 : NB
    POSITION(i, :) = obj.P{i+STA}.u;         % position:  n by 3 
    QUATERNION(i, :) = obj.P{i+STA}.quat;    % quaternions: n by 4 
    NU(6*i-5:6*i, 1) = obj.P{i+STA}.nu;      % velocities: 6*n columns 
    FORCE(6*i-5:6*i, 1) = obj.P{i+STA}.Fext; % forces: 6*n columns
    MASS(i, 1) = obj.P{i+STA}.mass;          % masses:  n columns
    INERTIA(3*i-2:3*i, :) = obj.P{i+STA}.J;  % Inertia_tensor: 3*n by 3
end

%obj.data_set.str2num(strcat('frame', num2str(step))) = struct('bodies', {}, 'contacts', {}, 'constraints', {}, 'settings', {});

obj.data_set.(strframe).bodies.positions   =  POSITION;
obj.data_set.(strframe).bodies.quaternions =  QUATERNION;
obj.data_set.(strframe).bodies.velocities  =  NU;
obj.data_set.(strframe).bodies.forces      =  FORCE;
obj.data_set.(strframe).bodies.masses      =  MASS;
obj.data_set.(strframe).bodies.inertia_tensors = INERTIA;

%% Constraints
obj.data_set.(strframe).constraints.size = 0;

%% Contacts
% Here I would like to follow Claude's format, by saving all the contact id
% in one long vector  and the corresponding id1 and id2 in the other vectors
NC = length(obj.Contacts);
GAP  = zeros(NC, 1);
MU_S = [];            % static coefficient of friction
MU_K = zeros(NC, 1);  % kinematic coefficient of friction
PAIRS = zeros(NC, 2);
NORMAL = zeros(NC, 3);
POINTS = zeros(NC, 3);

for i = 1: NC
    ID1 = obj.Contacts(i).body_1;    % The index, including the static bodies
    ID2 = obj.Contacts(i).body_2;    % The index, including the static bodies
    POINTS(i, :) = obj.Contacts(i).p1'; % points: n by 3 in world frame
    NORMAL(i, :) = obj.Contacts(i).normal(:, 1)';                    % normals: n by 3
    GAP(i, 1) = obj.Contacts(i, 1).psi_n;                            % gap: n column 
    MU_K(i, 1) = 0.5*obj.P{ID1}.mu * obj.P{ID2}.mu;                  % mu_k: n column
    
    % for static body, the body ID would be -1 
    if obj.P{obj.Contacts(i).body_1}.static
        ID1 = -1;
    else
        ID1 = ID1 - STA;
    end
    
    if obj.P{obj.Contacts(i).body_2}.static 
        ID2 = -1;
    else
        ID2 = ID2 - STA;
    end
    PAIRS(i, :) = [ID1, ID2];
end
obj.data_set.(strframe).contacts.points = POINTS;
obj.data_set.(strframe).contacts.normals = NORMAL;
obj.data_set.(strframe).contacts.gap = GAP;
obj.data_set.(strframe).contacts.mu_s = MU_S;
obj.data_set.(strframe).contacts.mu_k = MU_K;
obj.data_set.(strframe).contacts.pairs = PAIRS;

 
%% Settings 
obj.data_set.(strframe).settings.h = obj.h;  
end

