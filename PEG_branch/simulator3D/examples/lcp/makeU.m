function [U] = makeU(body)
	U = [];
	for i = 1 : size(body,2)
		for j = 1:3
			U = [U body(i).mu];
		end
	end
	for i = 1 : (size(body,2)-1)
		for j = (i+1) : size(body,2)
			U = [U sqrt(body(i).mu * body(j).mu)];
		end
	end
	U = diag(U);
end
