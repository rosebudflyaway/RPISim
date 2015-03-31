


function capsule = body_capsule( height, radius, num_segments )

	h = height;
	r = radius;
	n = num_segments;

	% Start at the heighest right most point, and generate vertices around
	verts = zeros(2*(n+1),2);	

	% Top of capsule
	for i=0:n
		verts(i+1,:) = [cos(i*(pi/n)) (h/2-r)+sin(i*(pi/n))];
	end

	% Bottom of capsule
	for i=n:-1:0
		verts(n+i+2,:) = [-cos(i*(pi/n)) -(h/2-r)-sin(i*(pi/n))];
    end
    
    capsule = Body(verts(:,1), verts(:,2));  

end

