function result = rot(h, theta)
	h = h / norm(h);
	h_cross = hat(h);
	result = eye(3) + sin(theta) * h_cross + (1-cos(theta)) * h_cross * h_cross;
end
