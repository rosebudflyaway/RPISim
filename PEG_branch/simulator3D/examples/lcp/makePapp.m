function [Papp] = makePapp(body, sim)
	grav = makeGrav();
	Papp = [];
	for i = 1:size(body,2)
		p = body(i).mass * grav * sim.h;
		Papp = [Papp;p;0;0;0];
	end
end
