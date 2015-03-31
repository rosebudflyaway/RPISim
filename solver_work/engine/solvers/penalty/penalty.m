%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% penalty.m
% penalty method is not so accurate as the LCP or MCP formulation because
% it just allows penetration and then correct it.

% we don't have to formulate the A and b matrix any more, just the function
% to define the normal force and frictional force 
% Fn = k * delta^(3/2) - Gn*vn*sqrt(delta)
% Fs = mu * Fn;

function [NUnew] = penalty(ConsPerBody, Contacts, P, h)

  %% Useful vars 
  activeBodies = find(ConsPerBody > 0);
  num_bodies = length(activeBodies);
  num_contacts = length(Contacts); 
  %k = 1.487e13;
  %g = 3.72e8;
  k = 0.002/(h*h);
  g = 5/h;
  
   %% Init submatrices 
  M = zeros(6*num_bodies);
  Gn = zeros(6*num_bodies,num_contacts);
  Gf = zeros(6*num_bodies,2*num_contacts);
  U = eye(num_contacts); 
  %b = zeros(2*num_bodies + num_bodies*fricdirs,1);  % We assign b a value below.  
  NU = zeros(6*num_bodies,1);   % Velocity, including angular
  %NUnew = zeros(6*num_bodies,1);% Return variable; the updated velocities
  FX = zeros(6*num_bodies,1);   % External force (not impulse!)    Fext should be updated to be Pext
  PSI = zeros(num_contacts,1);  % Distance per contact, psi_n  
  Fn = zeros(num_contacts, 1);
  Ft = zeros(num_contacts, 1);
  %% Formulate submatrices
  % M
  for i=1:num_bodies
     body = P{activeBodies(i)};                        % (activeBodies should never contain static bodies)
     if body.static > 0, continue; end                 % Don't incld static bds (this shouldn't happen)
     M(6*i-5:6*i-3,6*i-5:6*i-3) = body.mass*eye(3);    % 3x3 Mass matrix
     M(6*i-2:6*i,6*i-2:6*i) = body.J;                  % 3x3 Inertia matrix 
  end

   
  % Gn, U, and b
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
    % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    % This is the difference because for penalty method, we only have two
    % tangential directions, and the two direction is perpendicular to each
    % other 
    Gn_i1 = [-C.normal; cross3(r1,-C.normal)];
    Gn_i2 = [C.normal; cross3(r2,C.normal)];
    Gf_i1 = zeros(6,2);
    Gf_i2 = zeros(6,2); 
    
    Gf_i1(1:3, 1) = arbitraryTangent(-C.normal);
    Gf_i2(1:3, 1) = arbitraryTangent(C.normal);
    Gf_i1(1:3, 2) = cross(-C.normal, Gf_i1(1:3, 1));
    Gf_i2(1:3, 2) = cross(C.normal, Gf_i2(1:3, 1));
    
    Gf_i1(4:6, 1:2) = [cross(r1, Gf_i1(1:3, 1)), cross(r1, Gf_i1(1:3, 2))];
    Gf_i2(4:6, 1:2) = [cross(r2, Gf_i2(1:3.,1)), cross(r2, Gf_i2(1:3, 2))];
    % The following update should not include static bodies. 
    
    % body_1
    if P{C.body_1}.static == 0
        Gn(6*body1id-5:6*body1id,cID) = Gn_i1;
        Gf(6*body1id-5:6*body1id,2*cID-1:2*cID) = Gf_i1;
        NU(6*body1id-5:6*body1id) = P{C.body_1}.nu;
        FX(6*body1id-5:6*body1id) = P{C.body_1}.Fext;
    end
    
    % body_2
    if P{C.body_2}.static == 0
        Gn(6*body2id-5:6*body2id,cID) = Gn_i2; 
        Gf(6*body2id-5:6*body2id,2*cID-1:2*cID) = Gf_i2;
        NU(6*body2id-5:6*body2id) = P{C.body_2}.nu;
        FX(6*body2id-5:6*body2id) = P{C.body_2}.Fext;
    end

    % U
    U(cID,cID) = 0.5 * (P{C.body_1}.mu + P{C.body_2}.mu);
    PSI(cID) = C.psi_n;                                 % Per contact!
  end
%   disp('This is psi');
%   disp(PSI);
%   pause
    
  MinvGn = M\Gn; %for update of NUnew
  MinvGf = M\Gf; %for update of NUnew
 
  % normal relative velocity
  vn = Gn'*NU;
  gtzero = find(PSI > 0);
  ltzero = find(PSI < 0);
  PSI(gtzero) = zeros(length(gtzero), 1);
  temp = PSI(ltzero);
  PSI(ltzero) = -1 * temp;
%   disp('This is the PSI after convert to overlap')
%   disp(PSI);
   
  for i = 1 : length(PSI)
      if(PSI(i) < 0)
          disp('There is a PSI which is less than 0');
      end
  end  
  Fn = k * PSI.^(3/2) - (g * vn) .* sqrt(PSI);
  Fn = 0.05*Fn;
%   disp('This is Fn')
%   disp(Fn);
%   disp ('k PSI g vn ');
%   disp(k);
%   disp(PSI);
%   disp(g);
%   disp(vn);
%   for i = 1 : length(Fn) 
%       if(Fn(i) > 0)
%           %pause ;
%           disp('There is normal  force');
%       end
%   end


% This is the tangential relative direction
  vt = Gf' * NU; % [2*num_contacts, 1]
  for i = 1 : num_contacts
      if(norm(vt(i*2-1:i*2, 1)) > 1e-9)
          Ft_dir = vt(i*2-1:i*2, 1);    
          Ft_dir = Ft_dir/norm(Ft_dir);
      else
          Ft_dir = [0; 0];
      end
%       disp('This is the Ft_dir');
%       disp(Ft_dir);
%       pause 
      Ft(i*2-1:i*2, 1) = -U(i, i) * Fn(i) * Ft_dir;
  end
%   
  Pn = Fn * h;
%   pause 
% disp(Ft);
  Pf = Ft * h;
  %fprintf('This is the normal impulse: %f\n', Ft);
%   pause
  %fprintf('This is the tangential impulse %f \n', Pf);
%   pause 
  %% Calculate updated velocities. Quaternion updates occur in Simulation.m
 NUnew = NU + MinvGn*Pn + MinvGf*Pf + M\FX*h;  
%   NUnew = NU + MinvGn*Pn + M\FX*h;  
end 
