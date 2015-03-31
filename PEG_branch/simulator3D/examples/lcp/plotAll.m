function plotAll(time, trajAll, sim)
	nConts = 7;
	nBodies = 2;

	i = 1;
	figure(i);
	plot(time, trajAll(1,:),'r', time, trajAll(2,:), 'b', time, trajAll(1,:)+trajAll(2,:), 'g');
	title('KE PE and Total Energy');
	xlabel('Time');
	ylabel('Energy');
	legend('Kinetic energy','Potential energy','Total energy');

	for j = 1 : nConts
		i = i + 1;
		figure(i);
		nrmimpulse = abs(trajAll(18+j-1, :));
		frcimpulse = sqrt(trajAll(105+j*2-2, :).^2+trajAll(105+j*2-1,:).^2);
		frcimpulse(frcimpulse<1e-8) = 0;
		plot(time, nrmimpulse, 'r', time, frcimpulse,'b');
		tt =strcat('Impulses for contact  ',int2str(j));
		title(tt);
		xlabel('Time');
		ylabel('Impulse');
		legend('Normal impulse','Frictional impulse');
	end

	for j = 1 : nBodies
		i = i + 1;
		figure(i);
		speed = sqrt(trajAll(3+(j-1)*6,:).^2+trajAll(3+(j-1)*6+1,:).^2+trajAll(3+(j-1)*6+2,:).^2);
		plot(time, speed, 'r');
		tt = strcat('Scalar linear speed for body  ',int2str(j));
		title(tt);
		xlabel('Time');
		ylabel('Linear speed');
		legend('Linear spped');
	end
end
