function [M,D,fh,c,E,v0,fric,v_solved,l_solved] = create_system_from_dumps(path)

M=spconvert(load(strcat(path,'dump_M.dat')));
D=spconvert(load(strcat(path,'dump_Cq.dat')))';
fh=load(strcat(path,'dump_f.dat'));
c=load(strcat(path,'dump_b.dat'));
E=spconvert(load(strcat(path,'dump_E.dat')));
v0=load(strcat(path,'dump_v_old.dat'));
fric=load(strcat(path,'dump_fric.dat'));

v_solved=load(strcat(path,'dump_solved_q.dat'));
l_solved=load(strcat(path,'dump_solved_l.dat'));
end

%D=-D;
%c=-c;

