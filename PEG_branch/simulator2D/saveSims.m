


fig_title = 'box_stack_st';
index = 1;


frame_index = 100;
close all;
% Open a figure
h = figure('units','normalized','outerposition',[0 0 1 1]);
[im,map] = frame2im(sim.M(frame_index));
rgb = im;
image(rgb);
%axis([-100 100 -1000 220]);
axis equal; 
saveas(h,[fig_title '_' num2str(index)],'fig');
index = index + 1;





