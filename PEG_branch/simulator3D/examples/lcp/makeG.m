function [Gn, Gf, D, Gb] = makeG(body, sim)
	Psi = makePsi(body, sim);
	planeNrml(:,1) = [0.1; 0; sqrt(1 - 0.1^2)];
	planeNrml(:,2) = [-0.1/2;  0.1*sqrt(3)/2; sqrt(1 - 0.1^2)];
	planeNrml(:,3) = [-0.1/2; -0.1*sqrt(3)/2; sqrt(1 - 0.1^2)];
	L = 5;
	Point = [0; 0; 3+L];

	numbodies = size(body, 2);
	numplanes = size(planeNrml, 2);
	numcontacts = numbodies * numplanes + numbodies*(numbodies-1)/2;

	D = zeros(2*numcontacts, numcontacts * sim.nd);
	Di = zeros(2, sim.nd);
	theta = -pi:2*pi/sim.nd:pi;
	for i = 1 : sim.nd
		Di(:,i) = [cos(theta(i));sin(theta(i))];
	end
	for i = 1 : numcontacts
		D(2*(i-1)+1 : (2*i), sim.nd*(i-1)+1 : (sim.nd*i)) = Di;
	end

	Gn = [];
	Gf = [];
	Gb = [];
	for i = 1 : numbodies
		Gni = [];
		Gfi = [];
		Gbi = [];
		% sphere and plane
		for j = 1 : numbodies
			if j == i
				for k = 1 : numplanes
					n = planeNrml(:,k);
					t = Psi.N((j-1)*numplanes + k) * (-n) + body(j).u(1:3);
					t = t / norm(t);
					o = hat(n) * t;
					r = -n * body(j).radius;
					Gni = [Gni [n;hat(r)*n]];
					Gfi = [Gfi [t;hat(r)*t] [o;hat(r)*o]];
				end
			else
				Gni = [Gni zeros(6,numplanes)];
				Gfi = [Gfi zeros(6,numplanes*2)];
			end
		end
		% sphere and sphere
		for j = 1 : (numbodies-1)
			for k = (j+1) : numbodies
				if j == i
					n = body(j).u(1:3) - body(k).u(1:3);
					r = -n * body(j).radius;
				elseif k == i
					n = body(k).u(1:3) - body(j).u(1:3);
					r = -n * body(k).radius;
				else
					Gni = [Gni zeros(6,1)];
					Gfi = [Gfi zeros(12,1)];
					continue
				end
				n = n / norm(n);
				if n(1) ~= 0 || n(2) ~= 0
					t = [-n(2); n(1); 0];
				else
					t = [0; -n(3); n(2)];
				end
				t = t / norm(t);
				o = hat(n) * t;
				Gni = [Gni [n;hat(r)*n]];
				Gfi = [Gfi [t;hat(r)*t] [o;hat(r)*o]];
			end
		end
		% bilateral
		if i == 1
			h = body(1).u(5:7);
			alpha = acos(body(1).u(4))*2;
			if any(h)
				R = rot(h, alpha);
			else
				R = eye(3);
			end
			R = ep2rot(body(1).u(4:7));
			%r = R * [0;0;L];
			r = Point - body(1).u(1:3);
			r = r * L / norm(r);
			n = -r / norm(r);
			if n(1) ~= 0 || n(2) ~= 0
				t = [-n(2); n(1); 0];
			else
				t = [0; -n(3); n(2)];
			end
			t = t / norm(t);
			o = hat(n) * t;
n = [1 0 0]';
t = [0 1 0]';
o = [0 0 1]';
			Gbi = [[n;hat(r)*n] [t;hat(r)*t] [o;hat(r)*o]];
		else
			Gbi = zeros(6, 3);
		end
		Gn = [Gn;Gni];
		Gf = [Gf;Gfi];
		Gb = [Gb;Gbi];
	end
end
