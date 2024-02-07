function y = sgn(x)
y = x;
y(x>0) = 1;
y(x==0) = 0;
y(x<0) = -1;
end