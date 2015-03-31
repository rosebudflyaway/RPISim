function q = obj_func(A,b,x)
%evaluates the merit/objective function as (1/2)*x'*A*x+b'*x
q=0.5*x'*A*x+b'*x;