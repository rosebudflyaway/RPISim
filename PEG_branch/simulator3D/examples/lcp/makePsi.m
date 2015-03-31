function [Psi] = makePsi(body, sim)
	planeNrml(:,1) = [0.1; 0; sqrt(1 - 0.1^2)];
	planeNrml(:,2) = [-0.1/2;  0.1*sqrt(3)/2; sqrt(1 - 0.1^2)];
	planeNrml(:,3) = [-0.1/2; -0.1*sqrt(3)/2; sqrt(1 - 0.1^2)];
	L = 5;
	Point = [0;0;3+L];

	%Unilateral
	Psi.N = [];
	for i = 1:size(body, 2)
		for j = 1:3
			d = abs(body(i).u(1:3)' * planeNrml(:,j)) - body(i).radius;
			Psi.N = [Psi.N;d];
		end
	end
	for i = 1:(size(body, 2)-1)
		for j = (i+1) : size(body,2)
			d = norm(body(i).u(1:3) - body(j).u(1:3)) - body(i).radius - body(j).radius;
			Psi.N = [Psi.N;d];
		end
	end

	%Bilateral

	%R = ep2rot(body(1).u(4:7))
	%body(1).u(1:3) + R*[0;0;L];
	%vector = Point - (body(1).u(1:3)+R*[0;0;L]);
	%vector = (body(1).u(1:3)+R*[0;0;L])-Point;

	ll = body(1).u(1:3) - Point;
	vector = ll * (norm(ll) - L) / norm(ll);
	Psi.B = [vector];
end
