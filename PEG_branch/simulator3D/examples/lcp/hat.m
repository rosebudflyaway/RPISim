function result = hat(x)
	if size(x) ~= 3
		disp 'Parameter for hat must have size 3'
		return
	end
	result = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
end
