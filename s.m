function y = s(d)
global obstacle_range_1 obstacle_range_2
if d >= obstacle_range_2
    y = 1;
elseif d > obstacle_range_1 && d < obstacle_range_2
    y = sin((pi*(d - obstacle_range_1)) / (2 * (obstacle_range_2 - obstacle_range_1)))^2;
else
    y = 0;
end
end