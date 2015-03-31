function [ A, b, z0 ] = makeMCP( obj )
    M = obj.dynamics.M;
    Minv = obj.dynamics.Minv;
    Gn = obj.dynamics.Gn;
    Gf = obj.dynamics.Gf;
    E  = obj.dynamics.E;
    U = obj.dynamics.U;

    NU = obj.dynamics.Vel;
    nb = obj.dynamics.nb;
    nc = length(obj.contacts.PSI);
    nd = obj.dynamics.nd;
    if isfield(obj, 'constraints')
        % bilateral constraints
        nd = length(obj.constraints.pairs);
    end
    h = obj.dynamics.h;
    FX = obj.dynamics.Forces;
    PSI = obj.contacts.PSI'; % the depth is actually the penetration here 
    A = [ -M                  Gn                    Gf  zeros(6*nb,nc);  % Note: first line is negated
          Gn'            zeros(nc,nc*(2+nd));
          Gf'            zeros(nd*nc,(1+nd)*nc)     E;
          zeros(nc,6*nb)      U          -E'          zeros(nc)];
 
    b = [ M*NU + FX*h
            PSI / h
        zeros((nd+1)*nc,1) ];
    z0 = zeros(length(b), 1);
end

