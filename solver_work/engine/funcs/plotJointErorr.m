


pe = sim.dynamics.JointPosError;
ve = sim.dynamics.JointVelError; 

figure; hold on;
xlabel('Iteration'); ylabel('Distance Error');
title('Joint POSITION Error');
for j=1:size(sim.Joints,1)
   plot(pe(j,:),'Color',rand(1,3)); 
end

figure; hold on;
xlabel('Iteration'); ylabel('Velocity Error'); 
title('Joint VELOCITY Error');
for j=1:size(sim.Joints,1)
   plot(ve(j,:),'Color',rand(1,3)); 
end


