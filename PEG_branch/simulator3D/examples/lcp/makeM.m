function [M] = makeM(body)
nBodies = length(body);
Mi = zeros(6,6);
M = zeros(6*nBodies, 6*nBodies);
for i = 1 : nBodies
   Mi(1:3, 1:3) = body(i).mass * eye(3,3);
   Mi(4:6, 4:6) = body(i).mom;
   M(i*6-5 : i*6, i*6-5 : i*6) = Mi;
end
end