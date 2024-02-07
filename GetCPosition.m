function [C,R,distance] = GetCPosition(O,N,M)
%O=[0,0,0]'; N=[1,0,0]'; M=[0,1,0]';
L = norm(N-M);
OM = norm(O-M);
NM = norm(N-M);
ON = norm(O-N);
OlM = (OM^2 + NM^2 - ON^2) / (2 * NM);
C = zeros(3,1);
if OlM > 0 && OlM < L
    dx = N(1) - M(1);
    dy = N(2) - M(2);
    dz = N(3) - M(3);
    u = (O(1) - N(1))*(N(1) - M(1)) + (O(2) - N(2))*(N(2) - M(2)) + (O(3) - N(3))*(N(3) - M(3));
	u = u/((dx*dx)+(dy*dy)+(dz*dz));
    C(1) = N(1) + u*dx;
    C(2) = N(2) + u*dy;
    C(3) = N(3) + u*dz;

elseif  OlM >= L
    C = N;
else
    C = M;
end

R = C - O;
distance = norm(C-O);