function [ A, b, z0 ] = makeLCP( obj )
    Minv = obj.dynamics.Minv;
    Gn = obj.dynamics.Gn;
    Gf = obj.dynamics.Gf;
    E  = obj.dynamics.E;
    U = obj.dynamics.U;

    NU = obj.dynamics.Vel;
    nb = obj.dynamics.nb;
    nc = length(obj.contacts.PSI);
    %fprintf('The number of contact is: %d \n', nc);
    nd = obj.dynamics.nd;
    if isfield(obj, 'constraints')
        % bilateral constraints
        nj = length(obj.constraints.pairs);
    end
    h = obj.dynamics.h;
    FX = obj.dynamics.Forces;
    PSI = obj.contacts.PSI;   % the depth is actually the penetration here 
    MinvGn = Minv * Gn;
    MinvGf = Minv * Gf;
    MinvFX = Minv * FX;
    A = [ Gn'*MinvGn   Gn'*MinvGf  zeros(nc);
          Gf'*MinvGn   Gf'*MinvGf  E;
          U            -E'         zeros(nc)];
   
   %% TODO: The smallest eigenvalue of A is too small 10^(-16)
    
    b = [ Gn'*(NU + MinvFX*h) + PSI/h;    % FX*h could be replaced if we stored Pext instead of Fext
              Gf'*(NU + MinvFX*h);
                zeros(nc,1) ];
    z0 = zeros(length(b),1); 
    % The order of small real number is related to the condition number of
    % A, when 10^(-4), cond(A) = 10^(6); when 10^(-5), cond(A) = 10^(7);
    % when 10^(-6), cond(A) = 10^(8);
end

