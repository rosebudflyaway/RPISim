clear;clc;

%file_loc='inputData\';
%[M,D,fh,b,E,v_old,fric,v_s,l_s] = create_system_from_dumps(file_loc);


M=spconvert(load('dump_M.dat'));   % sparse convert
D=spconvert(load('dump_Cq.dat'))'; 
fh=load('dump_f.dat');  
b=load('dump_b.dat');
%E=spconvert(load('dump_E.dat'));
v_old=load('dump_v_old.dat');
fric=load('dump_fric.dat');

v_s=load('dump_solved_q.dat');
l_s=load('dump_solved_l.dat');

GPMiters=50;
k=fh+M*v_old;

disp('running solve_shur_P_GPMinres');
% [v_GPM,l_GPM,resid_GPM,resid_its_GPM,objs_GPM, Aset_GPM, pGrad_GPM, res_story_GPM,its_story_GPM,dv_story_GPM]...
%  = solve_shur_P_GPMinres(M,-D,k,b,E,GPMiters);

[v_GPM,l_GPM,resid_GPM,resid_its_GPM,objs_GPM, Aset_GPM, pGrad_GPM, res_story_GPM,its_story_GPM,dv_story_GPM]...
 = solve_shur_P_GPMinres(M,-D,k,b,GPMiters);
figure;
subplot(1,2,1);
% This is semilogy
semilogy(its_story_GPM,res_story_GPM,'.-');
legend('GPMinres');
set(gca,'fontsize',14);
xlabel('iterations','fontsize',14); ylabel('||\epsilon||_2','fontsize',14);
subplot(1,2,2);
% This is plot
plot(its_story_GPM,res_story_GPM,'.-');
legend('GPMinres');
set(gca,'fontsize',14);
xlabel('iterations','fontsize',14); ylabel('||\epsilon||_2','fontsize',14);

figure;
subplot(1,2,1);
plot(its_story_GPM,objs_GPM,'.-');
legend('GPMinres');
set(gca,'fontsize',14);
xlabel('iterations','fontsize',14);ylabel('objective function value','fontsize',14);
theMin=min(objs_GPM);
offset=1e-6-theMin;
obj_GPM=objs_GPM+offset;
subplot(1,2,2);
semilogy(its_story_GPM,obj_GPM,'.-');
legend('GPMinres');
set(gca,'fontsize',14);
xlabel('iterations','fontsize',14);ylabel('objective function value (shifted by constant)','fontsize',14);