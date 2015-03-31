function [E] = makeE(sim)
	numplanes = 3;
	%numbodies = size(body,2);
	numbodies = 2;
	nc = numbodies * numplanes + numbodies * (numbodies-1) / 2;
	nd = sim.nd;
	E = zeros(nc * nd, nc);
	row = 1;
	col = 1;
	for i = 1 : numbodies
		for j = 1 : numplanes
			E(row:row+nd-1, col) = ones(nd, 1); 
			row = row+nd;
			col = col+1;
		end
	end
	for i = 1 : (numbodies-1)
		for j = (i+1) : numbodies
			E(row:row+nd-1, col) = ones(nd, 1);
			row = row+nd;
			col = col+1;
		end
	end
end
