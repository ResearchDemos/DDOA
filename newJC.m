function J = newJC(R,JC)
    R = -4*sgn(R);
    J = zeros(3,6);
    for i = 1:3
        for j = 1:6
        J(i,j) = R(i) * JC(i,j);
        end
    end
end