function NUnew = fix_point(ConsPerBody, Contacts, P, h)

%% useful varviables
activeBodies = find(ConsPerBody > 0);
num_bodies = length(activeBodies);
num_contacts = length(Contacts);

%% Initialize submatrices
M = zeros(6*num_bodies);
Gn = zeros(6*num_bodies, num_contacts);
U = eye(num_contacts);
NU = zeros(6*num_bodies, 1);
FX = zeros(6*num_bodies, 1);
PSI = zeros(num_contacts, 1);

%% Formulate submatrices
% M
% here num_bodies is the number of active bodies 
for i = 1 : num_bodies
    body = P{activeBodies(i)};
    if body.static > 0 
        continue;
    end
    M(6*i-5:6*i-3, 6*i-5:6*i-3) = body.mass*eye(3);
    M(6*i-2:6*i, 6*i-2:6*i) = body.J;
end

% Gn, U
for i=1:num_contacts
    C = Contacts(i);
    cID = C.id;

    % For every contact, there are two bodies which need Gn and Gf (unless
    % contact was with a static object).  
    r1 = C.p1;
    r2 = C.p2;
    body1id = find(activeBodies == C.body_1,1,'first');  % These bodyIDs could be an obj_ attribute
    body2id = find(activeBodies == C.body_2,2,'first');  % Thus avoiding having to use 'find' 
    
    % Gn and Gf
    Gn_i1 = [-C.normal; cross(r1,-C.normal)];
    Gn_i2 = [C.normal; cross(r2,C.normal)];
    
    Gf_i1 = zeros(6,2);
    Gf_i2 = zeros(6,2); 
    
    Gf_i1(1:3, 1) = arbitraryTangent(-C.normal);
    Gf_i2(1:3, 1) = arbitraryTangent(C.normal);
    Gf_i1(1:3, 2) = cross(-C.normal, Gf_i1(1:3, 1));
    Gf_i2(1:3, 2) = cross(-C.normal, Gf_i2(1:3, 1));
    
    Gf_i1(4:6, 1:2) = [cross(r1, Gf_i1(1:3, 1)), cross(r1, Gf_i1(1:3, 2))];
    Gf_i2(4:6, 1:2) = [cross(r2, Gf_i2(1:3.,1)), cross(r2, Gf_i2(1:3, 2))];
    
    % The following update should not include static bodies. 
    if P{C.body_1}.static == 0
        Gn(6*body1id-5:6*body1id,cID) = Gn_i1;
        Gf(6*body1id-5:6*body1id,2*cID-1:2*cID) = Gf_i1;
        NU(6*body1id-5:6*body1id) = P{C.body_1}.nu;
        FX(6*body1id-5:6*body1id) = P{C.body_1}.Fext;
    end
    
    if P{C.body_2}.static == 0
        Gn(6*body2id-5:6*body2id,cID) = Gn_i2; 
        Gf(6*body2id-5:6*body2id,2*cID-1:2*cID) = Gf_i2;
        NU(6*body2id-5:6*body2id) = P{C.body_2}.nu;
        FX(6*body2id-5:6*body2id) = P{C.body_2}.Fext;
    end

    % U
    U(cID,cID) = 0.5*(P{C.body_1}.mu + P{C.body_2}.mu);
    PSI(cID) = C.psi_n;                                 % Per contact!
end

%% construct the Minv MinvGn MinvGf
MinvGn = M \ Gn;
MinvGf = M \ Gf;
MinvPext = M \ (FX * h); 
%pn = zeros(num_contacts, 1);
%pf = zeros(num_contacts*2, 1); 
%pf_ellp1 = zeros(num_contacts*2, 1);
%pn_ellp1 = zeros(num_contacts, 1);
%conver = zeros(max_iter, 1);

%[pn_ellp1, pf_ellp1] = fixed_point(M, Gn, Gf, MinvPext, h, PSI, NU, num_contacts);

%[pn_ellp1, pf_ellp1, err] = method_fix(PSI, h, Gn, NU, MinvGn, MinvPext, pn);

% This method_fix() function only considers the normal force and it works
% now, next move to fixed_point() function and include the frictional force

[pn_ellp1, pf_ellp1, ~] = method_fix(M, Gn, MinvPext, h, PSI, NU, num_contacts);
NUnew = NU + MinvGn * pn_ellp1 + MinvGf * pf_ellp1 + MinvPext;

end
 
