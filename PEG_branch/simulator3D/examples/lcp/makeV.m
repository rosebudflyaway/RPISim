function [V] = makeV(body)
	numbodies = size(body, 2);
	V = zeros(numbodies * 7, numbodies * 6);
	row = 1;
	col = 1;
	for i = 1 : numbodies
		Vi = zeros(7,6);
		Vi(1:3,1:3) = eye(3);
		[R,B] = ep2rot(body(i).u(4:7));
		Vi(4:7,4:6) = B * R';
		V(row:row+6, col:col+5) = Vi;
		row = row+7;
		col = col+6;
	end
end
