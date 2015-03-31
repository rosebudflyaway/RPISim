function simSphere_n_Pendulum
clear all

global M Gb Gn Gf D E U papp

%-----------------------------------------
% Set up params that define the simulation
sim.t0 = 0;     % Initial simluation time
sim.time = 0;   % Current simluation time
sim.h = 0.05;   % Simulation time step
sim.T = 100;     % Final simulation time  
sim.nd = 7;     % Number of friction directions.  CAN BE VARIED!!
sim.fps = 30;    % Number of frames to draw per second of simulation
sim.solver = 'lemke';
%sim.solver = 'proxPGS';
sim.hotStart = 0;

%-----------------------------------------
% Set up body data structure.  This also sets the initial conditions.
nBodies = 2;   % Fixed number of bodies
nConts = 7;    % Fixed number of contacts (i.e., number of body pairs)
[body] = initBodies();

nJnts = 2;
nu_ell = [body(1).nu; body(2).nu];
u_ell =  [body(1).u;  body(2).u];
drawScene(body);   % Draw initial configuration of system

%-----------------------------------------
% Create arrays of solution for ploting trajectories
%    1:1 ---------------- kinetic energy
%    2:2 ---------------- potential energy
%    3:14 --------------- nu
%   15:17 --------------- pb
%   18:24 --------------- pn
%   25:73 --------------- alpha
%   74:80 --------------- s
%   81:94 --------------- u
%   95:107 --------------- Psi_b
%   98:104 --------------- Psi_n
%  105:118 --------------- pf = D * alpha
nRows = 2 + 6*nBodies + 3*1 + nConts*(2+sim.nd) + 7*nBodies + 3*1 + nConts +  2*sim.nd;
time = sim.t0 : sim.h : sim.T;
nCols = length(time);
trajAll = zeros(nRows, nCols);

%-----------------------------------------
% Store the initial condition
% Kinetic energy
M = makeM(body); % Because bodies are spheres, M is constant for all time.
Minv = inv(M);   % Also constant
KE = nu_ell' * M * nu_ell / 2;

% Potential energy
grav = makeGrav;
PE = -grav' * (body(1).u(1:3) * body(1).mass ...
             + body(2).u(1:3) * body(2).mass);

% Distances at contacts
[Psi] = makePsi(body, sim);
Psin_ell = Psi.N;

%-----------------------------------------
% Start the simulation
% Because all contacts are tracked every time step, U and E are constant
U = makeU(body);  % U is constant for entire simulation
E = makeE(sim);   % E is constant for entire simulation
F = zeros(12,1);
lcpSolnLast = zeros(nConts*(2+sim.nd), 1);
mcpSolnLast = zeros(18 + nConts*(2+sim.nd), 1);
vn_ell = zeros(nConts,1);



frmTime = sim.t0;
for i = 1 : nCols 
     [Gn, Gf, D, Gb] = makeG(body, sim); 
     [Psi] = makePsi(body, sim);
     papp = makePapp(body, sim);       % Input to drive the dynamics
     V = makeV(body);                  % Needed for configuration update

    % Formulate standard LCP here
 
     QA = [[M;Gb'] [-Gb;zeros(3,3)]];
     QB = [[-Gn;zeros(3,nConts)] [-Gf*D;zeros(3,nConts*sim.nd)] [zeros(15, nConts)]];
     QE = [-M*nu_ell-papp+F; Psi.B/sim.h];
     QF = [Psi.N/sim.h;zeros(nConts*sim.nd, 1);zeros(nConts, 1)];
     QM = [[Gn'; D'*Gf'; zeros(nConts,12)] [zeros(nConts*(sim.nd+2),3)]];
     QN = [[zeros(nConts*(sim.nd+1), nConts); U] [zeros(nConts*(sim.nd+1), nConts*sim.nd); -E'] [zeros(nConts, nConts); E; zeros(nConts, nConts)]];

     QAinv = inv(QA);
     MM = QN - QM*QAinv*QB;
     qq = QF - QM*QAinv*QE;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                       
     % Guess solution
     if sim.hotStart
         lcpSoln0 = lcpSoln;
         mcpSoln0 = mcpSoln;
     else
         lcpSoln0 = zeros(nConts*(2+sim.nd), 1);
         mcpSoln0 = zeros(18 + nConts*(2+sim.nd), 1);
     end
     switch sim.solver
         case 'lemke'
             %[lcpSoln,err] = lemke(MM,qq,lcpSoln0); % Solve the subproblem
             [lcpSoln,err] = lemke(MM,qq,lcpSoln0); % Solve the subproblem
         case 'proxPGS'
             [lcpSoln,err] = proxPGS(MM,qq,lcpSoln0); % Solve the subproblem
     end
    % Check error
     if norm(err) > 1e-10
         fprintf(strcat(sim.solver, ':  LCP number = %5d    error = %12.6g\n'), i, err);
     end

     pn_ellp1 = lcpSoln(1:7);
     alpha_ellp1 = lcpSoln(8:56);
     s_ellp1  = lcpSoln(57:63);
    
    % Update velocities and configurations of the bodies and the time.
    % Also compute the impulses of the bilateral constraints
     QD = lcpSoln;
     QC = QAinv * (-QB*QD - QE);
     
     pf_ellp1 = D * alpha_ellp1;
     nu_ellp1 = QC(1:12);
     pb_ellp1 = QC(13:15);

     
     u_ellp1 = u_ell + V*nu_ellp1 * sim.h;

    % Update the body data structure
     for j = 1 : nBodies
         % Renormalize unit quaternions first
         quat = u_ellp1(j*7-3 : j*7);
         uQuat = quat / norm(quat);
         u_ellp1(j*7-3 : j*7) = uQuat;
         body(j).u  =  u_ellp1(j*7-6 : j*7);
         body(j).nu = nu_ellp1(j*6-5 : j*6);
     end
    
     % Update the trajectory information
     % Kinetic energy
     KE = nu_ellp1' * M * nu_ellp1 / 2;
 
     % Potential energy
     grav = makeGrav;
     PE = -grav' * (body(1).u(1:3) * body(1).mass ...
                  + body(2).u(1:3) * body(2).mass);
                 % + body(3).u(1:3) * body(3).mass);
    
     % Net contact impulses
     netNormImp = Gn * pn_ellp1;
     netFricImp = Gf * D * alpha_ellp1;
    
     % Distances at contacts
     [Psi] = makePsi(body, sim);
 
     % Prepare for next iteration of the simulation
     vn_ellm1 = vn_ell;  %%%% Added for bounce effect %%%%%
     vn_ell = Gn' * nu_ell;
     bounce = 0;
     delta_vn = zeros(nConts,1);
     scale = ones(12,1);
     for j = 1 : nConts
        if abs(vn_ell(j)) < 1e-2 && abs(vn_ellm1(j)) > 1e-1
	    bounce = 1;
	    delta_vn(j) = -vn_ellm1(j);
	    if j == 7
	      scale(1:6) = scale(1:6) * body(2).mass;
	      scale(7:12) = scale(7:12) * body(1).mass;
	      scale = scale ./ (body(1).mass+body(2).mass);
	    end
	end
      end
bounce = 0;
      if bounce == 1
	  vn_ell = vn_ell + delta_vn(j);
          nu_ellp1 = nu_ellp1 + (Gn * delta_vn) .* scale;
      end

     nu_ell = nu_ellp1;
     u_ell  = u_ellp1;
     Psin_ellm1 = Psin_ell;
     Psin_ell = Psi.N;
     
     trajAll(:,i) = [KE;  PE;  nu_ellp1;  pb_ellp1;  pn_ellp1;  alpha_ellp1;
                     s_ellp1;  u_ellp1;  Psi.B;  Psi.N;  pf_ellp1];

    sim.time = sim.time + sim.h;
    if  sim.time - frmTime > 1 / sim.fps
        drawScene(body);
        frmTime = sim.time;
    end

if max(abs(Psi.B)) > 0.2
	input('Psi.B > 0.2')
end
%    input('aa')
end
input('Press anykey to plot trajectory info');
close all;
plotAll(time, trajAll, sim);
end
